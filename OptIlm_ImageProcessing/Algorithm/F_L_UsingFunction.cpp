#include "InitialVariable.h"
#include "FunctionType.h"

void DrawShowLane_By_L_LandBound(short Colorflag,int r,short DrawCol,short size,ITS *iTS)
{
	short i;
	if(DrawCol==S_IMGLB||DrawCol==S_IMGRB)
		return;
	for ( i = -size; i <= size; i++)
	{
		if (DrawCol+i >= S_IMGLB && DrawCol+i <= S_IMGRB)
		{
			switch(Colorflag)
			{
				case 1:
					//OSD_Color_Setup(OCN_YELLOW, iTS);
					OSD_Color_Setup(OCN_BLUE, iTS);
					iTS->Showimage[r+DrawCol+i] = iTS->OSD_Color.Y;
					iTS->ShowUImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.U;
					iTS->ShowVImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.V;
					break;
				case 2:
					OSD_Color_Setup(OCN_RED, iTS);
					iTS->Showimage[r+DrawCol+i] = iTS->OSD_Color.Y;
					iTS->ShowUImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.U;
					iTS->ShowVImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.V;
					break;
				case 3:
					OSD_Color_Setup(OCN_YELLOW, iTS);
					iTS->Showimage[r+DrawCol+i] = iTS->OSD_Color.Y;
					iTS->ShowUImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.U;
					iTS->ShowVImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.V;
					break;

#ifdef L_PROTECTION_MAIN_ON_OFF
				case 4:
					OSD_Color_Setup(OCN_ORANGE, iTS);
					iTS->Showimage[r+DrawCol+i] = iTS->OSD_Color.Y;
					iTS->ShowUImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.U;
					iTS->ShowVImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.V;
					break;
				case 5:
					OSD_Color_Setup(OCN_PURPLE, iTS);
					iTS->Showimage[r+DrawCol+i] = iTS->OSD_Color.Y;
					iTS->ShowUImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.U;
					iTS->ShowVImg[((r + DrawCol+i)>>1)] = iTS->OSD_Color.V;
					break;
#endif    //L_PROTECTION_MAIN_ON_OFF
			}
		}
	}
}

void DrawLaneFromKMB_By_L_LaneBound(ITS *iTS)
{
        short row_;//col;
	short count=0;
	short final;
    short start = 0;
	int r;

	if(iTS->CAR->StableCtr > 5 && iTS->CARM.RealDistance < O_IS_TOO_CLOSE_STOP_LANE_DETECTION)
		return;

	final = L_IB_TB_DrawLane;

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	if(iTS->CARM.StableCtr>O_FC_GoingStable)
		final = iTS->CARM.CarInfo.Position.BB;
	else if(iTS->L_StbCtr>5)
		final = iTS->L_LastFinalRow_;
	else if(iTS->SL_LaneModel.L_StbCtr>10)
		final = iTS->SL_LaneModel.LastFinalRow_;
	else
		final = iTS->F_H_C;
#else
	if(iTS->CARM.StableCtr>O_FC_GoingStable)
		final = iTS->CARM.CarInfo.Position.BB;
	else if(iTS->L_StbCtr>5)
		final = iTS->L_LastFinalRow_;
	else
		final = iTS->F_H_C;
#endif

	if(final>L_IB_TB_DrawLane)
		final = L_IB_TB_DrawLane;
	else
		final = final;


//    start = S_IMGBB;
    start = L_IB_BB_SearchLane;

    r = SinkDataIndexStart(start);

    for ( row_ = start; row_ < final; row_++)
	{
		r = SinkDataIndexNextRow(r);
		count++;
#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
		if(iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_SL_LorR == SL_Right)
		{
			if(iTS->SolidlineR==1)
			{
				if(iTS->L_BiasWarn)	//Bias Right
                    DrawShowLane_By_L_LandBound(2,r,iTS->L_LaneMBound[row_].Scd,(short)(LineWidth(row_, 12)),iTS);  //8
				else
                    DrawShowLane_By_L_LandBound(1,r,iTS->L_LaneMBound[row_].Scd,(short)(LineWidth(row_, 12)),iTS);  //8
			}
		}
		if(iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_SL_LorR == SL_Left)
		{
			if(iTS->SolidlineL==1)
			{
				if(iTS->L_BiasWarn)	//Bias Left
                    DrawShowLane_By_L_LandBound(2,r,iTS->L_LaneMBound[row_].Frt,(short)(LineWidth(row_, 12)),iTS);
				else
                    DrawShowLane_By_L_LandBound(1,r,iTS->L_LaneMBound[row_].Frt,(short)(LineWidth(row_, 12)),iTS);
			}
		}
#endif
		if(iTS->L_DetectMode == LTRACE)
		{
			if(iTS->SolidlineL==1)
			{
				if(iTS->L_BiasWarn == 5) //Bias Left
                    DrawShowLane_By_L_LandBound(2,r,iTS->L_LaneMBound[row_].Frt,(short)(LineWidth(row_, 8)),iTS);
				else
                    DrawShowLane_By_L_LandBound(1,r,iTS->L_LaneMBound[row_].Frt,(short)(LineWidth(row_, 8)),iTS);
			}
			if(iTS->SolidlineR==1)
			{
				if(iTS->L_BiasWarn == -5) //Bias Right
                    DrawShowLane_By_L_LandBound(2,r,iTS->L_LaneMBound[row_].Scd,(short)(LineWidth(row_, 8)),iTS);
				else
                    DrawShowLane_By_L_LandBound(1,r,iTS->L_LaneMBound[row_].Scd,(short)(LineWidth(row_, 8)),iTS);
			}
		}
		if(count>=10){
			if(count==15){
				count=0;
			}
		}
		else
		{
#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
			if(iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_SL_LorR == SL_Right)
			{
				if(iTS->SolidlineR==0)
				{
					if(iTS->L_BiasWarn) //Bias Right
                        DrawShowLane_By_L_LandBound(2,r,iTS->L_LaneMBound[row_].Scd,(short)(LineWidth(row_, 12)),iTS);
					else
                        DrawShowLane_By_L_LandBound(1,r,iTS->L_LaneMBound[row_].Scd,(short)(LineWidth(row_, 12)),iTS);
				}
			}
			if(iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_SL_LorR == SL_Left)
			{
				if(iTS->SolidlineL==0)
				{
					if(iTS->L_BiasWarn) //Bias Left
                        DrawShowLane_By_L_LandBound(2,r,iTS->L_LaneMBound[row_].Frt,(short)(LineWidth(row_, 12)),iTS);
					else
                        DrawShowLane_By_L_LandBound(1,r,iTS->L_LaneMBound[row_].Frt,(short)(LineWidth(row_, 12)),iTS);
				}
			}
#endif

			if(iTS->L_DetectMode == LTRACE)
			{
				if(iTS->SolidlineL==0)
				{
					if(iTS->L_BiasWarn == 5) //Bias Left
                        DrawShowLane_By_L_LandBound(2,r,iTS->L_LaneMBound[row_].Frt,(short)(LineWidth(row_, 8)),iTS);
					else
                        DrawShowLane_By_L_LandBound(1,r,iTS->L_LaneMBound[row_].Frt,(short)(LineWidth(row_, 8)),iTS);
				}
				if(iTS->SolidlineR==0)
				{
					if(iTS->L_BiasWarn == -5) //Bias Right
                        DrawShowLane_By_L_LandBound(2,r,iTS->L_LaneMBound[row_].Scd,(short)(LineWidth(row_, 8)),iTS);
					else
                        DrawShowLane_By_L_LandBound(1,r,iTS->L_LaneMBound[row_].Scd,(short)(LineWidth(row_, 8)),iTS);
				}
               /*	if(row_ < (start + 40))
				{
					iTS->L_Vrow = iTS->L_evRodSlp - (row_ - iTS->F_H_C);
					col = iTS->F_W_C + LaneModel(iTS->L_Vrow, iTS->K, iTS->M, iTS->Bm);
					DrawShowLane_By_L_LandBound(1,r,col,(short)(LineWidth(row_, 1)),iTS);
				}  */
			}
		}




		double Vrow = iTS->L_evRodSlp - (LimitVP(row_) - iTS->F_H_C + 1 );
		iTS->L_W_test[row_][0] = (iTS->L_LaneMBound[row_].Scd - iTS->L_LaneMBound[row_].Frt);
		iTS->L_W_test[row_][1] = DItoDW((iTS->L_LaneMBound[row_].Scd - iTS->L_LaneMBound[row_].Frt),Vrow);
		iTS->L_W_test[row_][2] = DWtoDI(iTS->L_W_test[row_][1],Vrow);
		iTS->L_W_test[row_][3] = iTS->L_W_test[row_][0] - iTS->L_W_test[row_][2];
	}
}

void CalculateCenterLaneCurve(ITS *iTS)
{
	short index = 0;

	//double world_y = 130;
	double world_y = 0;

	for(index = 0; index < S_IMGCH; index++)
	{
		world_y = O_GetDistance(index, iTS);
		if(iTS->L_DetectMode == LTRACE)
			iTS->L_center_lane_curve[index][WORLD_X] = (float)((iTS->k * (world_y * world_y)) + (iTS->m * world_y) + iTS->bm);
        //	else if(iTS->L_DetectMode == SL_TRACE)
        //		iTS->L_center_lane_curve[index][WORLD_X] = (float)((iTS->SL_LaneModel.k * (world_y * world_y)) + (iTS->SL_LaneModel.m * world_y) + iTS->SL_LaneModel.bm);
		else
			iTS->L_center_lane_curve[index][WORLD_X] = 0;
		iTS->L_center_lane_curve[index][WORLD_Y] = (float)world_y;
		//world_y += 10;
	}
}

short LaneModel( double Vrow,  double _K,  double _M,  double _B)//inline
{	//	u_ = K/Vrow + M + B*Vrow
	if(Vrow == 0)
		return (short)(_M);
	else
		return (short)(_K / Vrow + _M + _B * Vrow);
}

void Draw_m(char Small_or_Big, short StartCol, short StartRow, ITS *iTS)
{
	short i;
	short Number_Width = 36;
	char NumberREC[36] = {0,0,0,0,0,0, 0,0,0,0,0,0, 0,1,1,1,1,1, 0,1,0,1,0,1, 0,1,0,1,0,1, 0,1,0,1,0,1};

	for(i = 0; i < Number_Width; i ++)
	{
		if(NumberREC[i] == 1)
		{
			if(Small_or_Big)
				DrawRect_Has_UV(StartRow+((i/6) << 2), StartCol+((i%6) << 2), 4, iTS);
			else
				DrawRect_Has_UV(StartRow+((i/6) << 1), StartCol+((i%6) << 1), 2, iTS);
		}
	}
}

void Draw_cm(char Small_or_Big, short StartCol, short StartRow, ITS *iTS)
{
    short i;
    short Number_Width = 36;
    char NumberREC_C[36] = {0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,1,1,1,0, 0,1,0,0,0,0, 0,1,0,0,0,0, 0,0,1,1,1,0};
    char NumberREC_M[36] = {0,0,0,0,0,0, 0,0,0,0,0,0, 0,1,1,1,1,1, 0,1,0,1,0,1, 0,1,0,1,0,1, 0,1,0,1,0,1};


    for(i = 0; i < Number_Width; i ++)
    {
        if(NumberREC_C[i] == 1)
        {
            if(Small_or_Big)
                DrawRect_Has_UV(StartRow+((i/6) << 2), StartCol+((i%6) << 2), 4, iTS);
            else
                DrawRect_Has_UV(StartRow+((i/6) << 1), StartCol+((i%6) << 1), 2, iTS);
        }
    }
    for(i = 0; i < Number_Width; i ++)
    {
        if(NumberREC_M[i] == 1)
        {
            if(Small_or_Big)
                DrawRect_Has_UV(StartRow+((i/6) << 2), StartCol+12+((i%6) << 2), 4, iTS);
            else
                DrawRect_Has_UV(StartRow+((i/6) << 1), StartCol+12+((i%6) << 1), 2, iTS);
        }
    }
}

short LaneDetection(ITS *iTS)
{
	if ( iTS->L_DetectMode == LSEARCH )
	{
		L_RstDualLM_Variables(iTS);

		if ( ! L_DualLM_Searching(iTS) )
		{
			return iTS->L_DetectionResultReturn = L_DUAL_FAIL;
		}

		iTS->L_DetectMode = LTRACE;
		return iTS->L_DetectionResultReturn = L_DUAL_SUCCESS;
	}
	else
	{
		if ( ! L_DualLM_Tracking(iTS) )
		{
			iTS->L_DetectMode = LSEARCH;
			iTS->L_StbCtr=0;
			return iTS->L_DetectionResultReturn = L_DUAL_FAIL;
		}

		if(iTS->L_StbCtr<L_FC_MaxLaneStableNumber)
			iTS->L_StbCtr++;
		return iTS->L_DetectionResultReturn = L_DUAL_SUCCESS;
	}
}

//#ifdef hmm
//#define SHOW_BIAS_DEBUG_NUMBER
//#endif
void L_GetBiasByImage(ITS *iTS)
{
	short BiasFlag = 0; //Mistake Avoid Flag

	//Bias Calculation
    iTS->L_Bias = (short) (iTS->k * BIAS_FORWARD_EXTEND * BIAS_FORWARD_EXTEND + iTS->m * BIAS_FORWARD_EXTEND + iTS->bm);
//    iTS->L_Bias = (short) (iTS->bm);

//#ifdef CAMERA_HORIZONTAL_AXIS_SHIFT
//        iTS->L_Bias += CAMERA_HORIZONTAL_SHIFT;
//#endif


	//Dual Lane Mark Tracking Mode
	if(iTS->L_DetectMode == LTRACE)
	{
		//Start Condition
		if(iTS->L_last_bias_flag == 0)
		{
			if(iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt > iTS->L_bias_vehicle_left_wheel_position)
			{
				iTS->L_BiasWarn = 5;
				iTS->L_last_bias_flag = 1;
				BiasFlag ++;
			}

			if(iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd < iTS->L_bias_vehicle_right_wheel_position)
			{
				iTS->L_BiasWarn = -5;
				iTS->L_last_bias_flag = 1;
				BiasFlag ++;
			}

			if(BiasFlag >= 2)
			{
				iTS->L_BiasWarn = 0;
				iTS->L_last_bias_flag = 0;
				iTS->L_Bias = 0;
			}

		}
		//End Condition and Re-initialize
		else
		{
			if(iTS->L_BiasWarn == 5 && (iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt > iTS->L_Bias_vehicle_right_wheel_position_correct_return_value ||
				iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt < iTS->L_Bias_vehicle_left_wheel_position_correct_return_value))
			{
				iTS->L_BiasWarn = 0;
				iTS->L_last_bias_flag = 0;
				iTS->L_Bias = 0;
			}
			if(iTS->L_BiasWarn == -5 && (iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd < iTS->L_Bias_vehicle_left_wheel_position_correct_return_value ||
			iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd > iTS->L_Bias_vehicle_right_wheel_position_correct_return_value))
			{
				iTS->L_BiasWarn = 0;
				iTS->L_last_bias_flag = 0;
				iTS->L_Bias = 0;
			}
		}
	}
#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	//Single Lane Mark Tracking Mode
	else if(iTS->L_DetectMode == SL_TRACE)
	{
		//Start Condition
		if(iTS->L_last_bias_flag == 0)
		{
			if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
			{
				if(iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt > iTS->L_bias_vehicle_left_wheel_position &&
				iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt < iTS->L_bias_vehicle_right_wheel_position)
				{
					iTS->L_BiasWarn = 5;
					iTS->L_last_bias_flag = 1;
					BiasFlag ++;
				}
			}

			if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
			{
				if(iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd < iTS->L_bias_vehicle_right_wheel_position &&
				iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd > iTS->L_bias_vehicle_left_wheel_position)
				{
					iTS->L_BiasWarn = -5;
					iTS->L_last_bias_flag = 1;
					BiasFlag ++;
				}
			}

			if(BiasFlag >= 2)
			{
				iTS->L_BiasWarn = 0;
				iTS->L_last_bias_flag = 0;
				iTS->L_Bias = 0;
			}
		}
		//End Condition and Re-initialize
		else
		{
			if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
			{
				if(iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt > iTS->L_Bias_vehicle_right_wheel_position_correct_return_value ||
					iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt < iTS->L_Bias_vehicle_left_wheel_position_correct_return_value)
				{
					iTS->L_BiasWarn = 0;
					iTS->L_last_bias_flag = 0;
					iTS->L_Bias = 0;
				}
			}

			if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
			{
				if(iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd < iTS->L_Bias_vehicle_left_wheel_position_correct_return_value ||
					iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd > iTS->L_Bias_vehicle_right_wheel_position_correct_return_value)
					{
						iTS->L_BiasWarn = 0;
						iTS->L_last_bias_flag = 0;
						iTS->L_Bias = 0;
					}
			}
		}
	}
#endif

	//Others (Not in Tracking Mode of Lane Detection Process)
	//Do Not Delete This Condition
	else
	{
		iTS->L_BiasWarn = 0;
		iTS->L_last_bias_flag = 0;
		iTS->L_Bias = 0;
	}

#ifdef SHOW_BIAS_DEBUG_NUMBER
	F_L_DrawWheelPositionInformationOnScreen(iTS);
#endif

}

short W_SetLaneWarring(ITS *iTS)
{

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	if(abs(iTS->L_BiasWarn) > 0 && (iTS->SL_LaneModel.L_StbCtr >= L_FC_InStable || iTS->L_StbCtr >= L_FC_InStable) &&
		iTS->L_DelayWarningCounter >= L_WARNING_TIME_DELAY)
#else

	if(abs(iTS->L_BiasWarn) > 0 && iTS->L_StbCtr >= L_FC_InStable && iTS->L_DetectMode == LTRACE &&
		iTS->L_DelayWarningCounter >= L_WARNING_TIME_DELAY)
#endif

		{
			iTS->L_DelayWarningCounter = 0;

			if(iTS->L_BiasWarn > 0)
				iTS->W_Lane = LeftDeparture;
			if(iTS->L_BiasWarn < 0)
				iTS->W_Lane = RightDeparture;
		}
		return 1;
}

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
void L_DetectionDecision(ITS *iTS)
{
    short BiasComparison = 10;//10;//iTS->L_WAvg >> 3;

#ifdef	L_LOSS_LANE_FOR_BIAS_TO_LARGE
	//單雙車道偵測到的第一個row之旗標
	iTS->L_SingleFlag = 1;
	iTS->L_DualLeftFlag = 1;
	iTS->L_DualRightFlag = 1;
#endif //L_LOSS_LANE_FOR_BIAS_TO_LARGE

#ifdef L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN
	L_DecisionSingleDirection(iTS);
#endif //L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN


    switch(iTS->L_DetectMode)
	{
		case LSEARCH:
			iTS->SL_LaneModel.L_StbCtr = 0;
            if(LaneDetection(iTS) == L_DUAL_FAIL)
                iTS->L_DetectMode = SL_SEARCH;
            else
				iTS->L_DetectMode = LTRACE;
			break;
		case LTRACE:
			iTS->SL_LaneModel.L_StbCtr = 0;
			if(LaneDetection(iTS) == L_DUAL_FAIL)
			{
//                iTS->L_DetectMode = LSEARCH;
                iTS->L_DetectMode = SL_SEARCH;
				iTS->L_StbCtr = 0;
				iTS->SupersedeCounter = 0;
			}
            else
            {
                iTS->L_DetectMode = LTRACE;
                if(iTS->L_StbCtr >= 10 || iTS->L_WAvg >= L_RW_MinLWInSig)
                {

#ifdef L_PROTECTION_MAIN_ON_OFF
#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
                    //於雙車道模式下,任一邊機制啟動,雙切單
                    if(iTS->L_Protection_SideBlockAdaptiveTH == SIDE_BLOCK_SINGLE_ON)
                    {
                        iTS->SL_SupersedeFlag = SINGLE_SUPERSEDE_DUAL;
                        if(Replaced_DualLane_With_SingleLane(iTS))
                        {
                            iTS->L_DetectMode = SL_TRACE;
                            iTS->SL_SupersedeFlag = REGULAR_PROCESS;
                            iTS->SL_LaneModel.L_StbCtr = L_SINGLE_LANE_IN_STABLE;
                            iTS->L_StbCtr = 0;
                            iTS->SL_LaneModel.evRodSlp = iTS->L_evRodSlp;
                        }
                    }
                    else
                    {
#endif //L_PROTECTION_SIDE_BLOCK_ON_OFF
#endif //L_PROTECTION_MAIN_ON_OFF

                        iTS->SupersedeCounter = LimitCount(++ iTS->SupersedeCounter, 30);
//                        OSD_Color_Setup(OCN_YELLOW, iTS);
//                        ScalableNumber((short) iTS->SupersedeCounter, 4, (S_IMGW >> 1) - 120, 55, iTS);
//                        ScalableNumber((short) abs(iTS->L_Bias), 4, (S_IMGW >> 1) - 60, 55, iTS);
//                        ScalableNumber((short) abs(iTS->L_BiasWarn), 4, (S_IMGW >> 1) + 60, 55, iTS);

                        double distance = 200;
                        double distance_bias = (iTS->k* (distance * distance)) + (iTS->m * distance) + iTS->bm;
//                        ScalableNumber((short) fabs(distance_bias), 4, (S_IMGW >> 1), 20, iTS);

                        if(((abs(iTS->L_Bias) > BiasComparison) || iTS->L_BiasWarn != 0 || fabs(distance_bias) > 100) && iTS->SupersedeCounter >= 15)//15)
                        {

                            iTS->SL_SupersedeFlag = SINGLE_SUPERSEDE_DUAL;
                            if(Replaced_DualLane_With_SingleLane(iTS))
                            {
                                iTS->L_DetectMode =SL_TRACE;
                                iTS->SL_SupersedeFlag = REGULAR_PROCESS;
                                iTS->SL_LaneModel.L_StbCtr = L_SINGLE_LANE_IN_STABLE;
                                iTS->L_StbCtr = 0;
                                iTS->SupersedeCounter = 0;
                                iTS->SL_LaneModel.evRodSlp = iTS->L_evRodSlp;
                            }
                            else
                            {
                                iTS->SupersedeCounter = LimitCount(-- iTS->SupersedeCounter, 30);
                            }
                        }

#ifdef L_PROTECTION_MAIN_ON_OFF
#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
                    }
#endif //L_PROTECTION_SIDE_BLOCK_ON_OFF
#endif //L_PROTECTION_MAIN_ON_OFF

                }
            }
            break;
        case SL_SEARCH:
            iTS->L_StbCtr = 0;
            if(L_SingleLMProcess(iTS) == L_SINGLE_FAIL)
                iTS->L_DetectMode = LSEARCH;
            break;
        case SL_TRACE:
            iTS->L_StbCtr = 0;
            if(L_SingleLMProcess(iTS) == L_SINGLE_FAIL)
            {
//                iTS->L_DetectMode = SL_SEARCH;
                iTS->L_DetectMode = LSEARCH;
                iTS->SL_SupersedeFlag = REGULAR_PROCESS;
                iTS->L_SL_PointGroup.SL_StableCounter = 0;
                iTS->SupersedeCounter = 0;
            }
            else
            {
//                OSD_Color_Setup(OCN_YELLOW, iTS);
//                ScalableNumber((short) iTS->SupersedeCounter, 4, (S_IMGW >> 1) - 90, 55, iTS);
//                ScalableNumber((short) abs(iTS->L_Bias), 4, (S_IMGW >> 1) - 60, 55, iTS);
//                ScalableNumber((short) abs(iTS->L_BiasWarn), 4, (S_IMGW >> 1) + 60, 55, iTS);
//                ScalableNumber((short)iTS->L_SL_PointGroup.SL_StableCounter, 4, (S_IMGW >> 1) + 90, 55, iTS);
//                ScalableNumber((short) (iTS->L_SL_PointGroup.RightCounter) , 4, S_IMGCW -90, S_IMGH - S_IMGCH, iTS);
//                ScalableNumber((short) (iTS->L_SL_PointGroup.FindLaneCounter) , 4, S_IMGCW -50, S_IMGH - S_IMGCH, iTS);



//                OSD_Color_Setup(OCN_RED, iTS);
//                ScalableNumber((short) (iTS->L_SL_PointGroup.mark_width*10) , 4, S_IMGCW, S_IMGH - S_IMGCH, iTS);
//                ScalableNumber((short) (iTS->debug_info) , 4, S_IMGCW -130, S_IMGH - S_IMGCH, iTS);




                if(iTS->SL_LaneModel.L_StbCtr >= 15)
                {
                    iTS->L_SL_PointGroup.SL_StableCounter = LimitCount(++ iTS->L_SL_PointGroup.SL_StableCounter, 20);
                    //if(iTS->SL_LaneModel.Flag_RideMarking == 0 /*&& iTS->L_BiasWarn == 0 */&& iTS->L_SL_PointGroup.SL_StableCounter == 20)
                    if(iTS->SL_LaneModel.Flag_RideMarking == 0 && iTS->L_BiasWarn == 0 && iTS->L_SL_PointGroup.SL_StableCounter >= 15) //20101225
                    {
                        iTS->SL_SupersedeFlag = DUAL_SUPERSEDE_SINGLE;

                        if(Replaced_SingleLane_With_DualLane(iTS) && iTS->SupersedeCounter >= 15)
                        {
                            iTS->L_DetectMode = LTRACE;
                            iTS->SL_SupersedeFlag = REGULAR_PROCESS;
                            iTS->L_StbCtr = L_FC_InStable;
                            iTS->L_SL_PointGroup.SL_StableCounter = 0;
                            iTS->SupersedeCounter = 0;
                            iTS->L_evRodSlp = iTS->SL_LaneModel.evRodSlp;
                        }
                    }
                }
                else
                    iTS->L_SL_PointGroup.SL_StableCounter = 0;
            }
            break;
	}

#ifdef L_LOSS_LANE_FOR_BIAS_TO_LARGE
	if(iTS->L_DetectMode == LTRACE || iTS->L_DetectMode == LSEARCH)
	{

#ifdef L_SHOW_FIRST_ROW
		OSD_Color_Setup(OCN_ORANGE, iTS);
		ScalableNumber(iTS->L_LeftFirstRow, 3, 20, S_IMGTB - 50, iTS);
		ScalableNumber(iTS->L_LeftFirstCol, 3, 20, S_IMGTB - 70, iTS);
		ScalableNumber(iTS->L_RightFirstRow, 3, S_IMGRB - 20, S_IMGTB - 50, iTS);
		ScalableNumber(S_IMGRB - iTS->L_RightFirstCol, 3, S_IMGRB - 20, S_IMGTB - 70, iTS);
#endif    //L_SHOW_FIRST_ROW

		if((iTS->L_LeftFirstRow > L_DUAL_FIRST_ROW_TO_UP && iTS->L_LeftFirstCol < L_DUAL_FIRST_ROW_TO_SIDE && iTS->L_RightFirstRow > 30 && iTS->L_RightFirstCol > (S_IMGRB - 30)) ||
		   (iTS->L_RightFirstRow > L_DUAL_FIRST_ROW_TO_UP && iTS->L_RightFirstCol > (S_IMGRB - L_DUAL_FIRST_ROW_TO_SIDE) && iTS->L_LeftFirstRow > 30 && iTS->L_LeftFirstCol < 30))
		{
			iTS->L_DetectMode = LSEARCH;
			iTS->L_StbCtr = 0;
		}
	}
#endif    //L_LOSS_LANE_FOR_BIAS_TO_LARGE





}
#endif

short ITSLANE_MAIN(ITS *iTS)
{
	memset(iTS->O_InfoPlane, 0, S_IMGW * S_IMGH * sizeof(char));
	memset(iTS->L_ColProjection, 0, S_IMGW * S_IMGH * sizeof(char));



#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF
	LM_GrayScale_Adaptive_Thresholding_Initial(&iTS->LM_R_GrayScale_AdaptiveThresholding);
	LM_GrayScale_Adaptive_Thresholding_Initial(&iTS->LM_L_GrayScale_AdaptiveThresholding);
#endif

#ifdef L_PROTECTION_MAIN_ON_OFF
	L_Protection_Initial(iTS);
#endif    //L_PROTECTION_MAIN_ON_OFF

    CreateEdge(iTS);
    F_O_CreateInfoPlan(iTS);


    DetectionFunctionDecision(iTS);

#if(SINGLE_OR_DUAL_LANE_DETECTION == DUAL_LANE_DETECTION_PROCESS)
	LaneDetection(iTS);
#endif

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS)
	L_SingleLMProcess(iTS);
#endif

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
//    L_DetectionDecision(iTS);
    if(iTS->function_switch.lane_detection == 0)
    {
        L_RstDualLM_Variables(iTS);
        L_RstSingleLM_Variables(iTS);
    }
    else if(iTS->function_switch.lane_detection == 1)
        L_DetectionDecision(iTS);
//    LaneDetection(iTS);


//    OSD_Color_Setup(OCN_YELLOW, iTS);
//    ScalableNumber((short) (iTS->single_lane_debug) , 4, S_IMGCW +90, S_IMGH - S_IMGCH, iTS);

//    OSD_Color_Setup(OCN_ORANGE, iTS);
//    ScalableNumber((short) (iTS->dual_lane_debug) ,  4, S_IMGCW +50, S_IMGH - S_IMGCH, iTS);


#endif

#ifdef CURVATURE_CALCULATION_ON_OFF
	Radius_of_Curvature(BIAS_FORWARD_EXTEND, iTS);
	OSD_Color_Setup(OCN_YELLOW, iTS);
	if((iTS->L_StbCtr >= 10 || iTS->SL_LaneModel.L_StbCtr > 10) && iTS->CurvatureStructure.Avg_Curvature < CURVATURE_CALCULATION_SHOW_NUMBER_LIMIT)
		ScalableNumber((int) ((double) iTS->CurvatureStructure.Avg_Curvature / 100), 3, 50, 50, iTS);
	else if(iTS->L_StbCtr >= 10 || iTS->SL_LaneModel.L_StbCtr > 10)
		ScalableNumber((int) 9999, 3, 50, 50, iTS); //Show in top and left of image
#endif

	SetLaneBnd(iTS); //ORG

#ifdef L_DRAW_LAND_BOUND_ON_OFF
	L_DrawLandBnd(iTS->Showimage,iTS);
#endif

#ifdef SHOW_BIG_F
	OSD_Color_Setup(OCN_YELLOW, iTS);
	DrawBigF(12, 176, 15, iTS);
#endif

	if(iTS->System_Parameter.Velocity != VELOCITY_OFF)
	{

#ifdef L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF
		L_Decision_CCD_Smear(iTS);
#endif    //L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF
#ifdef L_USING_PERCEPTION_POINT_SHIFT_ON_OFF
		//L_FourPointsShift(iTS);
#endif    //L_USING_PERCEPTION_POINT_SHIFT_ON_OFF
#ifdef L_USING_PERCEPTION_LANE_TILT_ON_OFF
		//L_ShowLaneTiltValue(iTS);
#endif    //L_USING_PERCEPTION_LANE_TILT_ON_OFF
#ifdef L_HORIZONTAL_MARKING_DETECTION_ON_OFF
		L_HorizontalMrakingDetection(iTS);
#endif    //L_HORIZONTAL_MARKING_DETECTION_ON_OFF
#ifdef L_PROTECTION_MAIN_ON_OFF
		L_DecisionForSystemProtection(iTS);
#endif    //L_PROTECTION_MAIN_ON_OFF

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION || SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS)

#ifndef PARAMETER_EXTRACTION
		if((iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10) || (iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > 10))
		{
#else
		if((iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > iTS->L_draw_dual_lane_mark) || (iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > iTS->L_draw_single_lane_mark))
		{
#endif
#ifdef L_PROTECTION_MAIN_ON_OFF
			if(iTS->L_Protection_Main == PROTECTION_ON || iTS->L_UsingPerception_Main == USING_PERCEPTION_ON)
			{
				//不顯示車道線
			}
			else
#endif    //L_PROTECTION_MAIN_ON_OFF
            if(iTS->function_switch.lane_detection == 1)
                DrawLaneFromKMB_By_L_LaneBound(iTS);
		}
#else
		if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10)
		{
#ifdef L_PROTECTION_MAIN_ON_OFF
			if(iTS->L_Protection_Main == PROTECTION_ON || iTS->L_UsingPerception_Main == USING_PERCEPTION_ON)
			{
				//不顯示車道線
			}
			else
#endif    //L_PROTECTION_MAIN_ON_OFF

				DrawLaneFromKMB_By_L_LaneBound(iTS);
		}
#endif
//		if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10)
//		{
//			CalculateCenterLaneCurve(iTS);
//		}
//		else
//		{
//			memset(&iTS->L_center_lane_curve[0][0], 0, sizeof(iTS->L_center_lane_curve));
//		}


#if(SINGLE_OR_DUAL_LANE_DETECTION == DUAL_LANE_DETECTION_PROCESS)
		if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > L_FC_InStable)
			L_GetBiasByImage(iTS);
#endif

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS)
		if(iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > L_SINGLE_LANE_IN_STABLE)
			L_GetBiasByImage(iTS);
#endif

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
		if((iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > L_FC_InStable) ||
			(iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > L_SINGLE_LANE_IN_STABLE))
		{
			L_GetBiasByImage(iTS);
		}
#endif



#ifdef DRAW_BIAS_RATIO_NUMBER
		ScalableNumber(iTS->L_Last_BiasRatio, 3, 120, 50, iTS);
#endif




#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF

		iTS->L_Cur_Left_Or_Right = 0;

		LM_GrayScale_Adaptive_Thresholding_Calculation(&iTS->LM_L_GrayScale_AdaptiveThresholding, iTS);

		iTS->L_Cur_Left_Or_Right = 1;

		LM_GrayScale_Adaptive_Thresholding_Calculation(&iTS->LM_R_GrayScale_AdaptiveThresholding, iTS);

#ifdef LM_GRAYSCALE_ADPTH_VISIBLE_DEBUGGING
		OSD_Color_Setup(OCN_PURPLE, iTS);
		ScalableNumber((short) iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold, 2, (S_IMGW >> 1) - 90, 55, iTS);
		ScalableNumber((short) iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold, 2, (S_IMGW >> 1) - 60, 55, iTS);
		ScalableNumber((short) iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold, 2, (S_IMGW >> 1) + 60, 55, iTS);
		ScalableNumber((short) iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold, 2, (S_IMGW >> 1) + 90, 55, iTS);

		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short) iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean, 4, (S_IMGW >> 1) - 90, 70, iTS);
		ScalableNumber((short) iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean, 4, (S_IMGW >> 1) + 60, 70, iTS);
#endif
#endif

#ifdef L_GROUND_REFLECTOR_PROTECTION_ON_OFF
		F_L_GroundReflector(iTS);
		F_L_DecisionGroundReflector(iTS);
#ifdef L_SHOW_PROTECTION_WARNING_SIGN
		L_ShowWarningForProtection(iTS->L_Protection_GroundReflector, iTS);
		OSD_Color_Setup(OCN_ORANGE, iTS);
		ScalableNumber((short) iTS->GroundReflector_Ctr, 2, 300, 30, iTS);
#endif    //L_SHOW_PROTECTION_WARNING_SIGN
#endif //L_GROUND_REFLECTOR_PROTECTION_ON_OFF

#ifdef L_WATER_STREAK_DETECTION_ON_OFF
		F_L_WaterStreakDetection(iTS);
#ifdef L_SHOW_PROTECTION_WARNING_SIGN
		L_ShowWarningForProtection(iTS->L_Protection_WaterStreak, iTS);
		OSD_Color_Setup(OCN_ORANGE, iTS);
		ScalableNumber((short) iTS->L_Water_Streak_Ctr, 2, 277, 30, iTS);
#endif    //L_SHOW_PROTECTION_WARNING_SIGN
#endif    //L_WATER_STREAK_DETECTION_ON_OFF

#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
		if(iTS->L_BiasWarn == 0 && ((iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10) || (iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > 10)))
		{
			L_DecisionSkyline(iTS);
			L_FindSideBlockMean(iTS->Skyline_V + L_BLOCK_ROW_DISTANCE_VANISHPOINT, iTS->L_SideBlock_L_Edge - L_PROTECTION_SIDE_BLOCK_COL_SIZE - L_BLOCK_COL_DISTANCE_LANE, iTS->L_SideBlock_R_Edge + L_BLOCK_COL_DISTANCE_LANE, L_PROTECTION_SIDE_BLOCK_ROW_SIZE, L_PROTECTION_SIDE_BLOCK_COL_SIZE, iTS);
			L_Protection_DecisionSide(iTS);
		}
		else //iTS->L_BiasWarn != 0
		{
			iTS->L_Protection_SideBlockAdaptiveTH = SIDE_BLOCK_OFF;
			iTS->L_Left_SideBlock_Counter = 0;
			iTS->L_Right_SideBlock_Counter = 0;
			iTS->Protection_Sideblock_Left_Draw = 1;
			iTS->Protection_Sideblock_Right_Draw = 1;
		}
#ifdef L_SHOW_PROTECTION_WARNING_SIGN
			L_ShowWarningForProtection(iTS->L_Protection_SideBlockAdaptiveTH, iTS);
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber((short) iTS->L_Left_SideBlock_Counter, 2, 200, 30, iTS);
			ScalableNumber((short) iTS->L_Right_SideBlock_Counter, 2, 215, 30, iTS);
#endif    //L_SHOW_PROTECTION_WARNING_SIGN
#endif    //L_PROTECTION_SIDE_BLOCK_ON_OFF

#ifdef L_PROTECTION_MIDDLE_BLOCK_ON_OFF
			L_Protection_MiddleBlock(iTS);
#endif    //L_PROTECTION_MIDDLE_BLOCK_ON_OFF

#ifdef SHOW_LANE_STABLE_COUNTER_ON_OFF
		OSD_Color_Setup(OCN_BLUE, iTS);
		ScalableNumber((short) iTS->L_StbCtr, 4, (S_IMGW >> 1), 70, iTS);
#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION || SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS)
                ScalableNumber((short) iTS->SL_LaneModel.L_StbCtr, 4, (S_IMGW >> 1), 100, iTS);
#endif
                ScalableNumber((short) iTS->L_DetectMode, 4, (S_IMGW >> 1), 40, iTS);

#endif

#ifdef GRAYSCALE_REMAPPING_ON_OFF
		GrayScaleRemapping(iTS);
#ifdef DRAW_MEAN_VALUE
		OSD_Color_Setup(OCN_PURPLE, iTS);
		ScalableNumber((short) iTS->L_Grayscale_Remapping.LastCorrectMarkMeanValue, 4, (S_IMGW >> 1) - 5, 55, iTS);
#endif
#endif


#ifdef OBSTACLE_DETECTION_ON_OFF
/*				//20111005 Jeremy Modify
#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
        if(iTS->SL_LaneModel.Flag_RideMarking == 0)// || iTS->L_Protection_Main == PROTECTION_ON || iTS->L_UsingPerception_Main == USING_PERCEPTION_ON)
			O_Main(iTS);
		else
		{
			O_FuzzyUpdatePerFrame(iTS);
			P_ResetShadowSpeedFlag(iTS);
			iTS->CAR->StableCtr = 0;
		}
#else*/
		O_Main(iTS);
//#endif
#endif
		SolidOrDashedLine(iTS);



#ifdef O_DRAW_VEHICLE_WARNING_BOUND
		//畫出前方30、15與10米線用來確認是否i入偵測範圍。
		OSD_Color_Setup(OCN_YELLOW, iTS);
		DrawColorLine(iTS->V_distance_number[2], 0, S_IMGW, iTS);
        ScalableNumber((short) (VEHICLE_WARNING_BOUND_3), 4, S_IMGW - 90, S_IMGH - iTS->V_distance_number[2] - 20, iTS);
        Draw_cm(0, S_IMGW - 43, S_IMGH - iTS->V_distance_number[2] - 13, iTS);
		OSD_Color_Setup(OCN_GREEN, iTS);
		DrawColorLine(iTS->V_distance_number[1], 0, S_IMGW, iTS);
        ScalableNumber((short) (VEHICLE_WARNING_BOUND_2), 4, S_IMGW - 90, S_IMGH - iTS->V_distance_number[1], iTS);
        Draw_cm(0, S_IMGW - 43, S_IMGH - iTS->V_distance_number[1] + 7, iTS);
		OSD_Color_Setup(OCN_RED, iTS);
		DrawColorLine(iTS->V_distance_number[0], 0, S_IMGW, iTS);
        ScalableNumber((short) (VEHICLE_WARNING_BOUND_1), 4, S_IMGW - 90, S_IMGH - iTS->V_distance_number[0], iTS);
        Draw_cm(0, S_IMGW - 43, S_IMGH - iTS->V_distance_number[0], iTS);


        DrawColorLine(190, S_IMGCW, S_IMGCW + DWtoDI(15.5,(0 - (190 - iTS->F_H_C))), iTS);
#endif    //O_DRAW_VEHICLE_WARNING_BOUND

		//F_V_VerticalProjection(S_IMGBB, S_IMGCH, S_IMGLB, S_IMGRB,iTS);
		//O_ShowVEVP(iTS->O_InfoPlane,iTS);  // for PC Debug   要把除2關掉

#ifdef DRAW_TB_AND_BB
                OSD_Color_Setup(OCN_YELLOW, iTS);
                DrawColorLine(L_IB_TB_TrackingLane, 0, S_IMGW, iTS);
                DrawColorLine(L_IB_BB_SearchLane, 0, S_IMGW, iTS);
#endif

	}

#ifdef L_AVG_ROAD_WIDTH
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short) iTS->L_WAvg, 4, S_IMGW >> 1, 200, iTS);
#endif


#ifdef CAMERA_SETUP_FUNCTION_ON_OFF
		// draw the Y axis fixed and variation vanishing point (line)
		OSD_Color_Setup(OCN_ORANGE, iTS);
		DrawColorLine((iTS->F_H - iTS->VanishingPoint_V), 0, iTS->F_W, iTS);
		OSD_Color_Setup(OCN_BLUE, iTS);
		DrawColorLine((iTS->F_H - iTS->F_H_C), 0, iTS->F_W, iTS);
		//memset(&iTS->Showimage[iTS->F_W * iTS->VanishingPoint_V], 64, iTS->F_W);
		//memset(&iTS->Showimage[iTS->F_W * (iTS->F_H - iTS->F_H_C)], 128, iTS->F_W);
        DrawRectangle(0, iTS->F_H, iTS->F_W_C, iTS->F_W_C, iTS);
        DrawRect(iTS->F_H - iTS->F_H_C - 10, (iTS->F_W_C-10), 20, 128, iTS);
		CameraSetUp(iTS);
#endif

#ifdef L_PROTECTION_MAIN_ON_OFF
		if(iTS->L_Protection_Main == PROTECTION_ON || iTS->L_UsingPerception_Main == USING_PERCEPTION_ON)
		{
			iTS->L_BiasWarn = 0;
			iTS->L_last_bias_flag = 0;
			iTS->L_Bias = 0;
		}
#endif   //L_PROTECTION_MAIN_ON_OFF

#ifdef SHOW_LDWS_WARNING_BLOCK
	L_DrawLaneDepartureWarningBlock(iTS);
#endif

#ifdef PARAMETER_EXTRACTION
	F_SetParameterExtraction(iTS);
#endif

#ifdef DRAW_VEHICLE_WIDTH_AND_CENTER
	F_L_DrawWheelPositionOnScreen(iTS);
#endif

	return 0;
}

#ifdef SHOW_LDWS_WARNING_BLOCK
void L_DrawLaneDepartureWarningBlock(ITS *iTS)
{
	if(iTS->L_BiasWarn < 0)//Right
	{
		OSD_Color_Setup(OCN_RED, iTS);
		DrawTriangle_Has_UV_Right(S_IMGCH - 20, S_IMGW - 30, 15, iTS);
		DrawTriangle_Has_UV_Right(S_IMGCH - 20, S_IMGW - 45, 15, iTS);
		DrawTriangle_Has_UV_Right(S_IMGCH - 20, S_IMGW - 60, 15, iTS);
	}
	if(iTS->L_BiasWarn > 0)//Left
	{
		OSD_Color_Setup(OCN_RED, iTS);
		DrawTriangle_Has_UV_Left(S_IMGCH - 20, 15, 15, iTS);
		DrawTriangle_Has_UV_Left(S_IMGCH - 20, 30, 15, iTS);
		DrawTriangle_Has_UV_Left(S_IMGCH - 20, 45, 15, iTS);
	}
}
#endif

void DrawDistance(short Value,short ROW_,short COL,ITS *iTS)
{
	Value /= 100;

	if(Value>=0 && Value<=100)
		Value = Value;
	else
		Value = 99;

	//ROW_ -= 40;

	if(iTS->W_Obstacle == SafeDistance)
	{
		OSD_Color_Setup(OCN_GREEN, iTS);
		ScalableNumber(Value, 2, COL - (10 + 5) , ROW_ + 20, iTS);

#ifdef SHOW_UNIT_OF_DISTANCE
		if(COL > S_IMGLB && COL < (S_IMGRB - 30))
		{
			Draw_m(0, COL + 15, ROW_ + 18, iTS);
		}
#endif

	}

	if(iTS->W_Obstacle == WarningDistance)
	{
		OSD_Color_Setup(OCN_YELLOW, iTS);
		ScalableNumber(Value, 4, COL - (10 + 3) , ROW_+10, iTS);

#ifdef SHOW_UNIT_OF_DISTANCE
		if(COL > S_IMGLB && COL < (S_IMGRB - 30))
		{
			Draw_m(0, COL + 15, ROW_ + 18, iTS);
		}
#endif

	}

	if(iTS->W_Obstacle == DangerousDistance)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber(Value, 4, COL - (10 + 3) , ROW_+10, iTS);

#ifdef SHOW_UNIT_OF_DISTANCE
		if(COL > S_IMGLB && COL < (S_IMGRB - 30))
		{
			Draw_m(0, COL + 15, ROW_ + 18, iTS);
		}
#endif

	}
}

short LimitW(short i)
{
	if( i<S_IMGLB )
		return S_IMGLB;
	if( i>S_IMGRB )
		return S_IMGRB;
	return i;
}

short LimitH(short i)
{
	if( i<L_IB_BB_TrackingLane )
		return L_IB_BB_TrackingLane;
	if( i>S_IMGTB )
		return S_IMGTB;
	return i;
}

short LimitVP(short i)
{
	if( i<L_IB_BB_TrackingLane)
		return L_IB_BB_TrackingLane;
	if( i>S_IMGCH-1 )
		return S_IMGCH-1;
	return i;
}

char LimitCount(char i,char Bound)
{
	if( i<0 )
		return 0;
	if( i>Bound )
		return (Bound);
	return i;
}

short LimitSearchingBoundary(short row, ITS *iTS)
{
	if( row < L_IB_BB_TrackingLane + 5)
		return L_IB_BB_TrackingLane + 5;

	if( row > L_IB_TB_TrackingLane - 5)
		return L_IB_TB_TrackingLane - 5;

	return row;
}

void DetectionFunctionDecision(ITS *iTS)
{
//#define LANE_DETECTION       0x01
//#define STOP_LINE_DETECTION  0x02
//#define ADULT_DETECTION      0x04
//#define CHILD_DETECTION      0x08

    if((iTS->function_switch.input_flag & LANE_DETECTION) == LANE_DETECTION)
    {
        iTS->function_switch.lane_detection = 1;
        OSD_Color_Setup(OCN_RED, iTS);
        DrawRect_Has_UV(20, 20, 20, iTS);
    }
    else
    {
        iTS->function_switch.lane_detection = 0;
        OSD_Color_Setup(OCN_YELLOW, iTS);
        DrawRect_Has_UV(20, 20, 20, iTS);
    }

    if((iTS->function_switch.input_flag & ADULT_DETECTION) == ADULT_DETECTION)
    {
        iTS->function_switch.adult_detection = 1;
        OSD_Color_Setup(OCN_RED, iTS);
        DrawRect_Has_UV(20, 60, 20, iTS);
    }
    else
    {
        iTS->function_switch.adult_detection = 0;
        OSD_Color_Setup(OCN_BLUE, iTS);
        DrawRect_Has_UV(20, 60, 20, iTS);
    }

    if((iTS->function_switch.input_flag & CHILD_DETECTION) == CHILD_DETECTION)
    {
        iTS->function_switch.child_detection = 1;
        OSD_Color_Setup(OCN_RED, iTS);
        DrawRect_Has_UV(20, 100, 20, iTS);
    }
    else
    {
        iTS->function_switch.child_detection = 0;
        OSD_Color_Setup(OCN_ORANGE, iTS);
        DrawRect_Has_UV(20, 100, 20, iTS);
    }

    //    if((iTS->function_switch.input_flag & STOP_LINE_DETECTION) == STOP_LINE_DETECTION)
    //    {
    //        iTS->function_switch.stop_line_detection = 1;
    //        OSD_Color_Setup(OCN_RED, iTS);
    //        DrawRect_Has_UV(20, 140, 20, iTS);
    //    }
    //    else
    //    {
    //        iTS->function_switch.stop_line_detection = 0;
    //        OSD_Color_Setup(OCN_GREEN, iTS);
    //        DrawRect_Has_UV(20, 140, 20, iTS);
    //    }
}


#ifdef DRAW_VEHICLE_WIDTH_AND_CENTER
#define RECTANGLE_MARK_SHIFT		20
#define MARK_RECTANGLE_SIZE		16
#define HALF_MARK_RECTANGLE_SIZE	8
#define LARGE_RECTANGLE_SIZE		8
#define HALF_LARGE_RECTANGLE_SIZE	4
#define SMALL_RECTANGLE_SIZE		4
#define HALF_SMALL_RECTANGLE_SIZE	2

void F_L_DrawWheelPositionOnScreen(ITS *iTS)
{
	//車道線在"追蹤模式"、"穩定"的情況下才顯示車道線位置方塊(移動顯示點)
	if(((iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > L_FC_InStable) ||
		(iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > L_SINGLE_LANE_IN_STABLE)) &&
		(iTS->L_DetectMode == LTRACE || iTS->L_DetectMode == SL_TRACE) &&
		((iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt > (S_IMGLB + RECTANGLE_MARK_SHIFT)) ||
		(iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd < (S_IMGRB - RECTANGLE_MARK_SHIFT))))
	{
		if(iTS->L_BiasWarn == 5 && iTS->SL_LaneModel.L_SL_LorR == SL_Left) //Left Bias Display
		{
			OSD_Color_Setup(OCN_YELLOW, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_MARK_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt - HALF_MARK_RECTANGLE_SIZE, MARK_RECTANGLE_SIZE, iTS);
			OSD_Color_Setup(OCN_RED, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);

			OSD_Color_Setup(OCN_YELLOW, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);
		}
		else if(iTS->L_BiasWarn == -5 && iTS->SL_LaneModel.L_SL_LorR == SL_Right) //Right Bias Display
		{
			OSD_Color_Setup(OCN_YELLOW, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);

			OSD_Color_Setup(OCN_YELLOW, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_MARK_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd - HALF_MARK_RECTANGLE_SIZE, MARK_RECTANGLE_SIZE, iTS);
			OSD_Color_Setup(OCN_RED, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
		}
		else
		{
			OSD_Color_Setup(OCN_YELLOW, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);

			OSD_Color_Setup(OCN_YELLOW, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);
		}
	}

	//警告線顯示(固定顯示點)
	// Show Left Correct Point
        OSD_Color_Setup(OCN_WHITE, iTS);
        DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_Bias_vehicle_left_wheel_position_correct_return_value - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
        OSD_Color_Setup(OCN_BLACK, iTS);
        DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_Bias_vehicle_left_wheel_position_correct_return_value - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);
	// Show Left Point
	OSD_Color_Setup(OCN_WHITE, iTS);
	DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_bias_vehicle_left_wheel_position - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
	OSD_Color_Setup(OCN_RED, iTS);
	DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_bias_vehicle_left_wheel_position - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);
	// Show Center Point
	OSD_Color_Setup(OCN_WHITE, iTS);
	DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_bias_vehicle_center_axis_position - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
	OSD_Color_Setup(OCN_BLACK, iTS);
	DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_bias_vehicle_center_axis_position - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);
	// Show Right Point
	OSD_Color_Setup(OCN_WHITE, iTS);
	DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_bias_vehicle_right_wheel_position - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
	OSD_Color_Setup(OCN_RED, iTS);
	DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_bias_vehicle_right_wheel_position - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);
	// Show Right Correct Point
	OSD_Color_Setup(OCN_WHITE, iTS);
	DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_LARGE_RECTANGLE_SIZE, iTS->L_Bias_vehicle_right_wheel_position_correct_return_value - HALF_LARGE_RECTANGLE_SIZE, LARGE_RECTANGLE_SIZE, iTS);
	OSD_Color_Setup(OCN_BLACK, iTS);
	DrawRect_Has_UV(S_IMGH - iTS->L_wheel_position_row - HALF_SMALL_RECTANGLE_SIZE, iTS->L_Bias_vehicle_right_wheel_position_correct_return_value - HALF_SMALL_RECTANGLE_SIZE, SMALL_RECTANGLE_SIZE, iTS);

}

#define WHEEL_POSITION_INFORMATION_NUMBER_SIZE			4
#define HALF_WHEEL_POSITION_INFORMATION_NUMBER_SIZE		2
void F_L_DrawWheelPositionInformationOnScreen(ITS *iTS)
{
	//上層數字
	//車道線模式：iTS->L_DetectMode
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber((short) iTS->L_DetectMode, WHEEL_POSITION_INFORMATION_NUMBER_SIZE, 60, S_IMGH - S_IMGCH, iTS);

	//保留狀態Flag：TS->L_last_bias_flag
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber((short) iTS->L_last_bias_flag, WHEEL_POSITION_INFORMATION_NUMBER_SIZE, 120, S_IMGH - S_IMGCH, iTS);

	//偏離Flag：iTS->L_BiasWarn
	if(iTS->L_BiasWarn > 0)
	{
		OSD_Color_Setup(OCN_GREEN, iTS);
		ScalableNumber((short) iTS->L_BiasWarn, WHEEL_POSITION_INFORMATION_NUMBER_SIZE, 180, S_IMGH - S_IMGCH, iTS);
	}
	else if(iTS->L_BiasWarn < 0)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short) abs(iTS->L_BiasWarn), WHEEL_POSITION_INFORMATION_NUMBER_SIZE, 180, S_IMGH - S_IMGCH, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_WHITE, iTS);
		ScalableNumber((short) iTS->L_BiasWarn, WHEEL_POSITION_INFORMATION_NUMBER_SIZE, 180, S_IMGH - S_IMGCH, iTS);
	}

	//Stable Counter：iTS->L_StbCtr
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber((short) iTS->L_StbCtr, WHEEL_POSITION_INFORMATION_NUMBER_SIZE, 240, S_IMGH - S_IMGCH, iTS);

	//Stable Counter：iTS->SL_LaneModel.L_StbCtr
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber((short) iTS->SL_LaneModel.L_StbCtr, WHEEL_POSITION_INFORMATION_NUMBER_SIZE, 300, S_IMGH - S_IMGCH, iTS);

	//下層數字
	//雙車道線
	if(iTS->L_DetectMode == LTRACE)
	{
		//世界座標中左車道線與左車輪之距離
		if(iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt != S_IMGLB)
		{
			OSD_Color_Setup(OCN_PURPLE, iTS);
		}
		else
		{
			OSD_Color_Setup(OCN_GRAY, iTS);
		}
		ScalableNumber((short) DItoDW(abs(iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt - iTS->L_bias_vehicle_left_wheel_position), S_IMGCH - iTS->L_wheel_position_row),
			HALF_WHEEL_POSITION_INFORMATION_NUMBER_SIZE, S_IMGCW - 70, S_IMGH - 60, iTS);

		//世界座標中右車道線與右車輪之距離
		if(iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd != S_IMGRB)
		{
			OSD_Color_Setup(OCN_PURPLE, iTS);
		}
		else
		{
			OSD_Color_Setup(OCN_GRAY, iTS);
		}
		ScalableNumber((short) DItoDW(abs(iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd - iTS->L_bias_vehicle_right_wheel_position), S_IMGCH - iTS->L_wheel_position_row),
			HALF_WHEEL_POSITION_INFORMATION_NUMBER_SIZE, S_IMGCW + 70, S_IMGH - 60, iTS);
	}
	//單車道線
	else if(iTS->L_DetectMode == SL_TRACE)
	{
		//世界座標中左車道線與左車輪之距離
		if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
		{
			OSD_Color_Setup(OCN_PURPLE, iTS);
		}
		else
		{
			OSD_Color_Setup(OCN_GRAY, iTS);
		}
		ScalableNumber((short) DItoDW(abs(iTS->L_LaneMBound[iTS->L_wheel_position_row].Frt - iTS->L_bias_vehicle_left_wheel_position), S_IMGCH - iTS->L_wheel_position_row),
			HALF_WHEEL_POSITION_INFORMATION_NUMBER_SIZE, S_IMGCW - 70, S_IMGH - 60, iTS);

		//世界座標中右車道線與右車輪之距離
		if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
		{
			OSD_Color_Setup(OCN_PURPLE, iTS);
		}
		else
		{
			OSD_Color_Setup(OCN_GRAY, iTS);
		}
		ScalableNumber((short) DItoDW(abs(iTS->L_LaneMBound[iTS->L_wheel_position_row].Scd - iTS->L_bias_vehicle_right_wheel_position), S_IMGCH - iTS->L_wheel_position_row),
			HALF_WHEEL_POSITION_INFORMATION_NUMBER_SIZE, S_IMGCW + 70, S_IMGH - 60, iTS);
	}

	//顯示車寬
	OSD_Color_Setup(OCN_GRAY, iTS);
	ScalableNumber((short) VEHICLE_WIDTH, HALF_WHEEL_POSITION_INFORMATION_NUMBER_SIZE, S_IMGCW - 10, S_IMGH - 60, iTS);

	//顯示平均車道寬
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber((short) iTS->L_WAvg, HALF_WHEEL_POSITION_INFORMATION_NUMBER_SIZE, S_IMGCW - 10, S_IMGH - 85, iTS);
}
#endif
