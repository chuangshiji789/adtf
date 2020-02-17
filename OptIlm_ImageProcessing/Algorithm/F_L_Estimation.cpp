#include "InitialVariable.h"
#include "FunctionType.h"

void Upt_Coef_Fitting_CURVE(register double a[5],register double b[3], register double c[3])
{
	double a02 = a[0]*a[2];
	double a03 = a[0]*a[3];
	double a04 = a[0]*a[4];
	double a11 = a[1]*a[1];
	double a12 = a[1]*a[2];
	double a13 = a[1]*a[3];
	double a14 = a[1]*a[4];
	double a22 = a[2]*a[2];
	double a23 = a[2]*a[3];
	double a24 = a[2]*a[4];
	double a33 = a[3]*a[3];
	double detA = a[2]*(a04 + 2*a13 - a22) - a[0]*a33 - a[4]*a11;
	if(detA<1&&detA>-1)	
		return;
	c[0] = ( b[0]*(a24 - a33) + b[1]*(a23-a14) + b[2]*(a13-a22) ) / detA;
	c[1] = ( b[0]*(a23 - a14) + b[1]*(a04-a22) + b[2]*(a12-a03) ) / detA;
	c[2] = ( b[0]*(a13 - a22) + b[1]*(a12-a03) + b[2]*(a02-a11) ) / detA;
}

void Upt_Coef_Fitting_LINE(const double a[5], const double b[2], double c[2])
{
	double detA = a[0]*a[2] - a[1]*a[1];
	if(detA==0)
		return;
	c[0] = ( b[0]*a[2] - b[1]*a[1] ) / detA;
	c[1] = ( b[1]*a[0] - b[0]*a[1] ) / detA;
}

void L_DualLM_BuildModel(ITS *iTS)
{	
	double Cxy[3];


#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_CURVE)
	iTS->L_LaneModelType = CURVE;
#endif

#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_LINE)
	iTS->L_LaneModelType = LINE;
#endif

	if(iTS->L_LaneModelType == CURVE)
	{
		Upt_Coef_Fitting_CURVE(iTS->dU, iTS->U, Cxy);
         iTS->k = Cxy[0] / (double)iTS->L_WCur / (S_eu * S_eu);
	    iTS->m = Cxy[1] / S_eu;
        iTS->bm = Cxy[2] * iTS->L_WCur;
	}
	else if(iTS->L_LaneModelType == LINE)
	{
		Upt_Coef_Fitting_LINE(iTS->dU, iTS->Um, Cxy);
		iTS->k=0.0;
	   	iTS->m = Cxy[0] / S_eu;
       	iTS->bm = Cxy[1] * iTS->L_WCur;
	}
	iTS->K = iTS->k * S_eu * S_ev * S_HOCam;
	iTS->M = iTS->m * S_eu;
	iTS->Bm = iTS->bm / S_HOCam * (S_euv);
	iTS->LaneR.B_i = (iTS->bm + (iTS->L_WCur>>1)) / S_HOCam * (S_euv);
	iTS->LaneL.B_i = (iTS->bm - (iTS->L_WCur>>1)) / S_HOCam * (S_euv);



    Upt_Coef_Fitting_LINE(iTS->dU, iTS->Um, Cxy);
    iTS->Reference_LaneModel.k  = 0.0;
    iTS->Reference_LaneModel.m  = Cxy[0] / S_eu;
    iTS->Reference_LaneModel.bm = Cxy[1] * iTS->L_WCur;
    iTS->Reference_LaneModel.K = iTS->Reference_LaneModel.k * S_eu * S_ev * S_HOCam;
    iTS->Reference_LaneModel.M = iTS->Reference_LaneModel.m * S_eu;
    iTS->Reference_LaneModel.Bm = iTS->Reference_LaneModel.bm / S_HOCam * (S_euv);

}

short L_RoadModel(ITS *iTS)
{
	short tmpW;
	double RodSlp;
	double Czy[2];

	Upt_Coef_Fitting_LINE(iTS->dU, iTS->V, Czy);

	if(Czy[1] == 0)
    {
        iTS->dual_lane_debug = 4;
        return 0;
    }
	tmpW =  (short)((-(S_evu) * S_HOCam / Czy[1]) + TRANSFER_ERROR);

	iTS->L_evRodSlp = Czy[0];
#ifndef CAMERA_SETUP_FUNCTION_ON_OFF
    iTS->L_evRodSlp = EV_ROAD_SLOPE;   //TEST
#endif

	if(iTS->L_DetectMode==LTRACE)
	{
		if(tmpW < L_RW_MinLWInCtn || tmpW > L_RW_MaxLWInCtn)	
        {
            iTS->dual_lane_debug = 5;
            return 0;
        }
		tmpW = (short)((double)tmpW + (double)iTS->L_WAvg*(L_PW_SmoothLaneWidth-1));
		tmpW = (short)((double)tmpW /L_PW_SmoothLaneWidth);
	}
	else
	{
        if(tmpW < L_RW_MinLWInSig || tmpW > L_RW_MaxLWInSig)
        {
            iTS->dual_lane_debug = 6;
            return 0;
        }
	}
	iTS->L_WAvg = tmpW;
	iTS->L_WCur = tmpW;
	
	RodSlp = iTS->L_evRodSlp / S_ev;

	iTS->VanishingPoint_V = (short)((double)iTS->F_H - (double)S_IMGCH - iTS->L_evRodSlp);

    if (fabs(RodSlp) >= L_RT_RoadSlope)
    {
        iTS->dual_lane_debug = 77;
//        return 0;
    }

	return 1;
}

short L_DecideModeOrder(ITS *iTS)
{

        if(	  (iTS->LaneL.PixCtr > L_IB_ThChangeOrder && iTS->LaneR.PixCtr > (L_IB_ThChangeOrder >> 1)) ||
                  (iTS->LaneL.PixCtr > (L_IB_ThChangeOrder >> 1) && iTS->LaneR.PixCtr > L_IB_ThChangeOrder)	)
                iTS->L_LaneModelType = CURVE;
	else
                iTS->L_LaneModelType = LINE;  //CURVE

	return 1;
}

short L_DualLM_Estimation(ITS *iTS)
{
	if(!L_RoadModel(iTS))
		return 0;
	L_DecideModeOrder(iTS);
	L_DualLM_BuildModel(iTS);
	return 1;
}

