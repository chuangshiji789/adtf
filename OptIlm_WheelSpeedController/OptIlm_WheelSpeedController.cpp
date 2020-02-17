/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hart#$  $Date:: 2017-05-19 08:12:10#$ $Rev:: 63515   $
**********************************************************************/
#include <adtf3.h>
#include <stdlib.h>
#include "OptIlm_WheelSpeedController.h"
#include "ADTF3_helper.h"
#include <vector>

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_WHEELSPEEDCONTROLLER_DATA_TRIGGERED_FILTER,
                                    "OptIlm_WheelSpeedController",
                                    cOptIlm_WheelSpeedController,
                                    //adtf::filter::pin_trigger({"measured_vehicle_speed"}));
                                    adtf::filter::thread_trigger(true))

////adtf::filter::thread_trigger(tTrue));
#define MOTOR_MAX           50
#define MOTOR_MIN            7
#define MOTOR_NULL_POINT	 0 	//stop
#define MOTOR_START_PCT     12
#define MOTOR_BRAKE_LIMIT   20
//#define MS_TO_PRC           12



//CTOR of the TriggerFuntion
//This is to initialize the Trigger
cOptIlm_WheelSpeedController::cOptIlm_WheelSpeedController()
{
    // SetName("Timer_trigger");

    //Register Properties
    RegisterPropertyVariable("01.controller type", m_i32ControllerMode                              );
    RegisterPropertyVariable("02.proportional factor for PID Controller ", m_f64PIDKp               );
    RegisterPropertyVariable("03.integral factor for PID Controller", m_f64PIDKi                    );
    RegisterPropertyVariable("04.differential factor for PID Controller", m_f64PIDKd                );
    RegisterPropertyVariable("05.sampletime for the pid controller [S]", m_f64PIDSampleTime        );
    RegisterPropertyVariable("06.input factor for PT1", m_f64PT1OutputFactor                        );
    RegisterPropertyVariable("07.time constant for pt1 controller", m_f64PT1TimeConstant            );
    RegisterPropertyVariable("08.set point is multiplied with this factor", m_f64PT1CorrectionFactor);
    RegisterPropertyVariable("09.gain factor for PT1 controller", m_f64PT1Gain                      );
    RegisterPropertyVariable("10.the minimum output value for the controller [%]", m_f64PIDMinimumOutput);
    RegisterPropertyVariable("11.the maximum output value for the controller [%]", m_f64PIDMaximumOutput);
    RegisterPropertyVariable("12.show debug output", m_bShowDebug                                   );

    //Get Media Descriptions
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oInputMeasWheelSpeed, "measured_vehicle_speed", pTypeSignalValue);
    Register(m_oInputSetWheelSpeed,  "desired_vehicle_speed", pTypeSignalValue);
    Register(m_oInputSteering, "steering", pTypeSignalValue);//XH

    Register(m_oEmergencyBreakFlag,  "Emergency_Break_Flag", pTypeSignalValue);

    Register(m_oOutputActuator,      "actuator_output", pTypeSignalValue);

    star_flag = 1;
}
//implement the Configure function to read ALL Properties
tResult cOptIlm_WheelSpeedController::Configure()
{
    m_f64LastControllerOutput = 0;
    m_f64LastMeasuredError = 0;
    m_f64SetPoint = 0;
    m_lastSampleTime = 0;
    m_f64LastSpeedValue = 0;
    m_f64IntergalError = 0;

    EmergencyBreakFlag = 0;
    last_steering_angle = 0;
    difference_steering_angle = 0;

    //    RETURN_IF_FAILED(cTriggerFunction::Configure());
    switch (m_i32ControllerMode)
    {
    case 1:
        LOG_INFO(adtf_util::cString::Format("P controller"));
        LOG_INFO(adtf_util::cString::Format("Kp: %.4f",(float)m_f64PIDKp));
        /** Set Ki = 0 and Kd = 0, so the resuling controller is a P controller*/
        m_f64PIDKi = 0;
        m_f64PIDKd = 0;
        break;
    case 2:
        LOG_INFO(adtf_util::cString::Format("PI controller"));
        LOG_INFO(adtf_util::cString::Format("Kp: %g; Ki: %g",(float)m_f64PIDKp, (float)m_f64PIDKi));
        /** Set Kd = 0, so the resuling controller is a PI controller*/
        m_f64PIDKd = 0;
        break;
    case 3:
        LOG_INFO(adtf_util::cString::Format("PID controller"));
        LOG_INFO(adtf_util::cString::Format("Kp: %.4f; Ki: %.4f; Kd: %.4f",(float)m_f64PIDKp, (float)m_f64PIDKi, (float)m_f64PIDKd));
        break;
    case 4:
        LOG_INFO(adtf_util::cString::Format("PT1 controller"));
        LOG_INFO(adtf_util::cString::Format("input factor: %.4f; time constant: %.4f; set point is multiplied with this factor: %.4f; gain factor: %.4f",(float)m_f64PT1OutputFactor, (float)m_f64PT1TimeConstant, (float)m_f64PT1CorrectionFactor, (float)m_f64PT1Gain));
        break;
    case 5:
        LOG_INFO(adtf_util::cString::Format("PI-Fuzzy controller"));
        LOG_INFO(adtf_util::cString::Format("The Kp and Ki value will be interpolated between defined working points"));
        /** This are the defined and tested values for the PI-Fuzzy controller */
        PI_fuzzy_working_point_vector =   {-0.5,-0.25, 0.0, 0.25, 0.5, 0.75,  1.0, 1.25, 1.5, 1.75, 2.0};
        PI_fuzzy_working_point_Kp_value = {  50,   50,  50,   50,  50,   30,   25,   20,  18,   17,  16};
        PI_fuzzy_working_point_Ki_value = {  30,   30,  30,   30,  30,   25,   20,   18,  15,   15,  15};
        /** Set Kd = 0, so the resuling controller is a PI controller*/
        m_f64PIDKd = 0;
    default:
        break;
    }
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    LOG_INFO(adtf_util::cString::Format("Kp: %.4f; Ki: %.4f; Kd: %.4f",(float)m_f64PIDKp, (float)m_f64PIDKi, (float)m_f64PIDKd));
    RETURN_NOERROR;
}

///this funtion will be executed each time a trigger occured
///Due to the change of the data receive events it can be possible that more than one sample was pushed to the
/// Readers queue. So make sure the execution of this funtion will read ALL Samples of ALL Readers until the queues are empty.
tResult cOptIlm_WheelSpeedController::Process(tTimeStamp tmTimeOfTrigger)
{
    // avoid that the method triggers itself due to the feedback loop
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    tTimeStamp m_NowSampleTime = tmTimeOfTrigger;// GetTime();
    f64SampleTime = (tFloat64)(m_NowSampleTime - m_lastSampleTime);
    f64SampleTime *= 0.000001;

    if(f64SampleTime > m_f64PIDSampleTime)
    {
        m_lastSampleTime = m_NowSampleTime;
        object_ptr<const ISample> pSetWheelSpeedSample;
        if (IS_OK(m_oInputSetWheelSpeed.GetLastSample(pSetWheelSpeedSample)))
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32  Ui32TimeStamp = 0;

            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSetWheelSpeedSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

            // write to member variable
            m_f64SetPoint = static_cast<tFloat64>(f32Value);
            if (m_i32ControllerMode == 4)
                m_f64SetPoint = m_f64SetPoint * m_f64PT1CorrectionFactor;
        }
        // Read Output speed percent value because of Emergency Break
        object_ptr<const ISample> pEmergencyBreakFlaSample;
        if (IS_OK(m_oEmergencyBreakFlag.GetLastSample(pEmergencyBreakFlaSample)))
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32  Ui32TimeStamp = 0;

            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pEmergencyBreakFlaSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

            EmergencyBreakFlag = f32Value;
        }
        // Measured Wheel Speed
        object_ptr<const ISample> pWheelSpeedSample;
        if (IS_OK(m_oInputMeasWheelSpeed.GetLastSample(pWheelSpeedSample)))
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32  Ui32TimeStamp = 0;

            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pWheelSpeedSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

            // write to member variable
            m_f64MeasuredVariable = f32Value;
        }
        object_ptr<const ISample> pSampleFromSteering;
        if (IS_OK(m_oInputSteering.GetLastSample(pSampleFromSteering)))
        {
            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromSteering);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &steering_angle.f32Value));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &steering_angle.ui32ArduinoTimestamp));

            difference_steering_angle = fabs(last_steering_angle) - fabs(steering_angle.f32Value);
            last_steering_angle = steering_angle.f32Value;
            //debug
            //            LOG_INFO(cString::Format("Steering Angle: %g ", m_f64WheelAngle));

        }
        //calculation
        // if speed = 0 is requested output is immediately set to zero
        if (m_f64SetPoint == 0  /**|| EmergencyBreakFlag == 1 */ || (m_f64MeasuredVariable == 0 && star_flag == 0))
        {
            m_f64LastControllerOutput = 0;
            m_f64IntergalError = 0;
            m_f64LastMeasuredError = 0;

//            if(m_f64MeasuredVariable == 0)
                star_flag = 1;
        }
        //soft starter
        else if (m_f64SetPoint != 0 /**&& EmergencyBreakFlag == 0*/ && star_flag == 1)
        {
            m_f64IntergalError = 0;
            m_f64LastMeasuredError = 0;
            /** be carful with the direction: -1 means forward, +1 means backward driving */
            int direction = -1*Signum(m_f64SetPoint);
            m_f64LastControllerOutput = direction * (MOTOR_START_PCT + FeedforwardAngle(last_steering_angle));
            if((fabs(m_f64MeasuredVariable) > 0.4 && m_f64SetPoint != 0)){
                star_flag = 0;
            }
            //LOG_INFO(adtf_util::cString::Format("Percent: %f, set point: %f", m_f64LastOutput, m_f64SetPoint));
        }
        else
        {
            m_f64LastControllerOutput = calculateNextOutputValue();
            //LOG_INFO(adtf_util::cString::Format("Percent: %.3f, set point: %.2f, controller value: %.3f", m_f64LastControllerOutput, m_f64SetPoint, controllerValue));
        }

        // speed limit
        if(fabs(m_f64MeasuredVariable) > 2.5){
            m_f64LastControllerOutput = 0;
        };

        outputValue = static_cast<tFloat32>(m_f64LastControllerOutput);

        //LOG_INFO(adtf_util::cString::Format("Percent: %f, set point: %f, measured output: %f", outputValue, m_f64SetPoint, fabs(m_f64MeasuredVariable)));
        transmitSignalValue(m_oOutputActuator, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, outputValue);

        //for debug
        //LOG_INFO(adtf_util::cString::Format("Sample Time= %.3f  SetPoint= %.2f  Speed= %.2f  Emergency Break= %g  Output= %g", f64SampleTime, m_f64SetPoint, m_f64MeasuredVariable, EmergencyBreakFlag, outputValue));
        //LOG_INFO(cString::Format("Sample Time= %.3f  SetPoint= %.2f  Speed= %.2f  Steering angle= %g  Output= %g", f64SampleTime, m_f64SetPoint, m_f64MeasuredVariable, m_f64WheelAngle, outputValue));
    }
    RETURN_NOERROR;
}

tFloat64 cOptIlm_WheelSpeedController::getControllerValue(tFloat64 i_f64MeasuredValue){
    tFloat f64Result = 0;
    //the controller algorithms

    if (m_i32ControllerMode <= 3){
        /*********************************
        * P-, PI- or PID-Controller
        * Initialization depends on Mode
        * Mode 1:
        *  - use of user defined m_f64PIDKp
        *  - sets m_f64PIDKi = m_f64PIDKd = 0
        *
        * Mode 2:
        *  - use of user defined m_f64PIDKp
        *  - use of user defined m_f64PIDKi
        *  - sets m_f64PIDKd = 0
        *
        *
        * Mode 3:
        *  - use of user defined m_f64PIDKp
        *  - use of user defined m_f64PIDKi
        *  - use of user defined m_f64PIDKd
        **********************************/
        f64Result = PID(i_f64MeasuredValue, m_f64PIDKp, m_f64PIDKi, m_f64PIDKd);
    }
    else if (m_i32ControllerMode == 4)
    {
        /*********************************
        * PT 1 discrete algorithm
        *
        *               Tau
        *       In +    ---  * LastOut
        *             Tsample
        * Out = ---------------------
        *               Tau
        *       1 +     ---
        *             Tsample
        *
        *                           Tau
        * here with     PT1Gain =   ---
        *                         Tsample
        *
        *                  T
        * y(k) = y(k-1) + --- ( v * e(k)  - y(k-1))
        *                  T1
        *
        * e(k):  input
        * y(k-1) : last output
        * v : gain
        * T/T1: time constant
        **********************************/
        m_f64LastControllerOutput = m_f64LastControllerOutput + m_f64PT1TimeConstant * (m_f64PT1Gain *(m_f64SetPoint - i_f64MeasuredValue) - m_f64LastControllerOutput);
    }
    else if(m_i32ControllerMode == 5){
        /*********************************++++++++++
        * Interpolated PI-Controller
        *
        * Defined working set points:
        *  - from -0.5 m/s to 2 m/s with a step width of 0.25 m/s
        *
        * Calculates Kp and Ki between the two closest set points
        *
        * Sets the value Kd = 0
        ********************************************/
        int id1 = 0,id2 = 0;
        float min_dist = std::numeric_limits<int>::max();
        int l = PI_fuzzy_working_point_vector.size(); // vector length
        for(int i=0; i < l; i++){
           float temp = fabs(m_f64SetPoint-PI_fuzzy_working_point_vector.at(i));
           if( temp < min_dist){
               min_dist = temp;
               id1 = i;
           }
           else{
               break;
           }
        }
        tFloat64 Kp, Ki;
        if(id1 == 0 || id1 == l){
            // absolute value of set point
            float factor =  0.25 < min_dist ? 0 : (1-4*min_dist);
            Kp = factor * PI_fuzzy_working_point_Kp_value.at(id1);
            Ki = factor * PI_fuzzy_working_point_Ki_value.at(id1);
        }
        else{
            float min_dist_left = fabs(m_f64SetPoint-PI_fuzzy_working_point_vector.at((id1 - 1)));
            float min_dist_right = fabs(m_f64SetPoint-PI_fuzzy_working_point_vector.at((id1 + 1)));

            if(min_dist_left < min_dist_right){
                id2 = id1-1;  // second nearest working point for actual set point
            }
            else{
                id2 = id1+1;  // second nearest working point for actual set point
            }
            // if the closest set point is more than 0.25 [m/s] away, the set point is out of the definied working area
            float f =  0.25 < min_dist ? 0 : (1-4*min_dist);
            Kp = f * PI_fuzzy_working_point_Kp_value.at(id1) + (1-f) * PI_fuzzy_working_point_Kp_value.at(id2);
            Ki = f * PI_fuzzy_working_point_Ki_value.at(id1) + (1-f) * PI_fuzzy_working_point_Ki_value.at(id2);

        }
        //LOG_INFO(adtf_util::cString::Format("Kp: %.4f; Ki: %.4f; Kd: %.4f", Kp, Ki, 0));
        f64Result = PID(i_f64MeasuredValue, Kp, Ki, (tFloat64)0);
    }

    return f64Result;
}

tTimeStamp cOptIlm_WheelSpeedController::GetTime(){
    return adtf_util::cHighResTimer::GetTime();
}

tFloat64 cOptIlm_WheelSpeedController::calculateNextOutputValue(){
    tFloat64 output_percent = 0;
    /** be carful with the direction: -1 means forward, +1 means backward driving */
    /** the direction "dir" can't be zero, this is checked earlier */

    // direction
    int dir = -1 * Signum(m_f64SetPoint);
    // feedforward for set point
    float set_point_pct = -1 * FeedforwardSpeed(m_f64SetPoint);
    // feedforward for angle
    float angle = dir * FeedforwardAngle(last_steering_angle);// if we are driving forward ( dir == -1), the extra percentage as to be negative as well
    // controller value
    float controllerValue = 0;
    if (m_i32ControllerMode == 4){
        controllerValue  = -getControllerValue(m_f64MeasuredVariable)*m_f64PT1OutputFactor;
    }
    else{
        controllerValue  = -getControllerValue(m_f64MeasuredVariable);
    }

    // total output in percent
    output_percent = controllerValue + set_point_pct + angle;

    /** Set Limits */
    // driving forward
    if(dir == -1){
        // Minimu -50; Maximum = 20 (Motorbremse)
        output_percent = SetLimit(output_percent, -MOTOR_MAX, MOTOR_BRAKE_LIMIT);
    }
    // driving backward
    else{
       // Minimu -20 (Motorbremse); Maximum = 50
       output_percent = SetLimit(output_percent, -MOTOR_BRAKE_LIMIT, MOTOR_MAX);
    }
    return output_percent;
}

//tFloat64 cOptIlm_WheelSpeedController::ConvertSpeedToPercentOld(tFloat64 controllerOutput)
//{
//    tFloat64 output_percent = 0;
//    /** be carful with the direction: -1 means forward, +1 means backward driving, 0 means standing still */
//    int dir = -1*Signum(m_f64SetPoint);
    
//    /** standing still */
//    if(dir == 0){
//        output_percent = 0;
//    }
//    /** backward */
//    else if(dir > 0){
//          float set_point = -1*m_f64SetPoint*MOTOR_START_PCT;
//          float set_point_pct = -1*FeedforwardSpeed(m_f64SetPoint);
//          float angle = FeedforwardAngle(last_steering_angle);
//          float controller =-1*controllerOutput;
//          output_percent = controller + set_point + set_point_pct + angle;
////          if (last_steering_angle >= 0)
////          {
////              if(fabs(last_steering_angle) > 45 && fabs(last_steering_angle) <= 70)
////                  output_percent = controllerOutput + (m_f64SetPoint * (MS_TO_PRC + (fabs(last_steering_angle) * 0.02)));
////              else if(fabs(last_steering_angle) > 70 && fabs(last_steering_angle) <= 80)
////                  output_percent = controllerOutput + (m_f64SetPoint * (MS_TO_PRC + (fabs(last_steering_angle) * 0.024)));
////              else if(fabs(last_steering_angle) > 80)
////                  output_percent = controllerOutput + (m_f64SetPoint * (MS_TO_PRC + (fabs(last_steering_angle) * 0.028)));
////              else
////                  output_percent = controllerOutput + (m_f64SetPoint * (MS_TO_PRC));
////          }
////          else
////          {
////              if(fabs(last_steering_angle) > 60 && fabs(last_steering_angle) <= 80)
////                  output_percent = controllerOutput + (m_f64SetPoint * (MS_TO_PRC + 1));
////              else if(fabs(last_steering_angle) > 80)
////                  output_percent = controllerOutput + (m_f64SetPoint * (MS_TO_PRC + 2));
////              else
////                  output_percent = controllerOutput + (m_f64SetPoint * (MS_TO_PRC));
////          }
//          output_percent = SetLimit(output_percent, -MOTOR_BRAKE_LIMIT, MOTOR_MAX);
//          LOG_INFO(adtf_util::cString::Format("BACKWARD: Percent: %f, controller output: %f, set point: %f, set point to percent: %f, angle: %f", output_percent, controllerOutput, set_point, set_point_pct, angle));
//          }

//    else if(dir < 0)//forward
//    {
//        float set_point = -1*m_f64SetPoint*(MOTOR_START_PCT);
//        float set_point_pct = -1*FeedforwardSpeed(m_f64SetPoint);
//        float angle = -1*FeedforwardAngle(last_steering_angle);
//        float controller = -1*controllerOutput;
//        output_percent = controller + set_point + set_point_pct + angle;
//        output_percent = SetLimit(output_percent, -MOTOR_MAX, MOTOR_BRAKE_LIMIT);
//        //LOG_INFO(adtf_util::cString::Format("FORWARD: Percent: %f, controller output: %f, set point: %f, set point to percent: %f, angle: %f", output_percent, controllerOutput, set_point, set_point_pct, angle));

//    }

//    return output_percent;
//}


tFloat64 cOptIlm_WheelSpeedController::SetLimit(tFloat64 value, tFloat64 min, tFloat64 max){
    return value < min ? min : max < value ? max : value;
}

tInt cOptIlm_WheelSpeedController::Signum(tFloat setPoint){
    return setPoint < 0 ? -1 : 0 < setPoint ? 1 : 0;
}

tFloat cOptIlm_WheelSpeedController::FeedforwardSpeed(tFloat setPoint){
    return  0.01283 * pow(setPoint, 5) +
           -0.01193 * pow(setPoint, 4) +
           -0.2022  * pow(setPoint, 3) +
            0.1893  * pow(setPoint, 2) +
            5.738   * setPoint;// + 0;
}

tInt cOptIlm_WheelSpeedController::FeedforwardAngle(tFloat last_steering_angle){
    float a = fabs(last_steering_angle);
    float b = a < 50 ? 0 : a < 60 ? 2 : a < 70 ? 4 : a < 80 ? 6 : a < 90 ? 8 : a < 95 ? 10 : 11;
    return m_f64SetPoint <= 0.5 ? 2.0/3.0*b : b;
}

tFloat64 cOptIlm_WheelSpeedController::PID(tFloat64 measurement, tFloat64 Kp, tFloat64 Ki, tFloat64 Kd){
    // actual error
    tFloat64 e = m_f64SetPoint - measurement;

    // proportional term
    tFloat64 P = Kp*e;

    // integral term
    // adds the actual error to the integral error only if the allowable maximum/minimum has not yet been reached.
    if(-MOTOR_MAX < m_f64LastControllerOutput && m_f64LastControllerOutput < MOTOR_MAX){
        m_f64IntergalError += e * f64SampleTime;
    };
    tFloat64 I = Ki*m_f64IntergalError;

    // differential term
    tFloat64 D = 0;
    if(Kd != 0 && f64SampleTime != 0.0){
        tFloat64 dervative = (e - m_f64LastMeasuredError) / f64SampleTime;
        D = Kd * dervative;
    }

    tFloat64 controller_output = P+I+D;
    //LOG_INFO(adtf_util::cString::Format("controller output: %f, P: %f, I: %f, D: %f", controller_output, P, I, D));
    // set limits (Minimum = -50, Maximum = 50);
    controller_output = SetLimit(controller_output, -MOTOR_MAX, MOTOR_MAX);
    // save actual error for next time step
    m_f64LastMeasuredError = e;
    // save actual controller output for next time step
    m_f64LastControllerOutput = controller_output;
    return controller_output;
}
