#include "InitialVariable.h"
#include "FunctionType.h"

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
void Dual_ReplacedBy_Single_Reset(ITS *iTS)
{
	iTS->L_StbCtr = 0;
	iTS->L_Ctr_LossBothLane =0;
	iTS->L_Ctr_LossSingleLane = 0;
	iTS->L_CurFinalRow_ = S_IMGCH;
	iTS->L_LastFinalRow_ = S_IMGCH;

	iTS->L_WCur = L_RW_DefaultLW;	//	road width
	iTS->L_WAvg = L_RW_DefaultLW;
    iTS->L_evRodSlp = EV_ROAD_SLOPE;			//	euslope = S_ev * slope


    iTS->k = iTS->SL_LaneModel.k;//L_MP_kDefault;					//	curvature
    iTS->m = iTS->SL_LaneModel.m;//L_MP_mDefault;					//	orientation
    iTS->bm = iTS->SL_LaneModel.bm;//L_MP_bDefault;				//	bias
    iTS->br = iTS->bm + (L_RW_ROILW >> 1);	//	bias
    iTS->bl = iTS->bm - (L_RW_ROILW >> 1);	//	bias


//    iTS->k = iTS->default_k;//L_MP_kDefault;					//	curvature
//    iTS->m = iTS->default_m;//L_MP_mDefault;					//	orientation
//    iTS->bm = iTS->default_b;//L_MP_bDefault;				//	bias
//    iTS->br = iTS->default_b + (L_RW_ROILW >> 1);	//	bias
//    iTS->bl = iTS->default_b - (L_RW_ROILW >> 1);	//	bias

    if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
      {
          if(iTS->L_LaneMBound[L_SINGLE_LANE_SEARCH_BB].Frt > S_IMGCW)
          {
              iTS->k = iTS->default_k;//L_MP_kDefault;							//	curvature
              iTS->m = iTS->default_m; //L_MP_mDefault;							//	orientation
              iTS->bm = iTS->default_b;//L_MP_bDefault;						//	bias
              iTS->br = iTS->default_b + (L_RW_ROILW >> 1);		//	bias
              iTS->bl = iTS->default_b - (L_RW_ROILW >> 1);			//	bias
          }
      }
      else if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
      {
          if(iTS->L_LaneMBound[L_SINGLE_LANE_SEARCH_BB].Scd < S_IMGCW)
          {
              iTS->k = iTS->default_k;//L_MP_kDefault;							//	curvature
              iTS->m = iTS->default_m; //L_MP_mDefault;							//	orientation
              iTS->bm = iTS->default_b;//L_MP_bDefault;						//	bias
              iTS->br = iTS->default_b + (L_RW_ROILW >> 1);		//	bias
              iTS->bl = iTS->default_b - (L_RW_ROILW >> 1);			//	bias
          }

      }
      else if(iTS->SL_LaneModel.L_SL_LorR == SL_NotFound)
      {
          iTS->k = iTS->default_k;//L_MP_kDefault;							//	curvature
          iTS->m = iTS->default_m; //L_MP_mDefault;							//	orientation
          iTS->bm = iTS->default_b;//L_MP_bDefault;						//	bias
          iTS->br = iTS->default_b + (L_RW_ROILW >> 1);		//	bias
          iTS->bl = iTS->default_b - (L_RW_ROILW >> 1);			//	bias
      }


	iTS->K = iTS->k * S_eu * S_ev * S_HOCam;		//	K = k * S_eu * S_ev * H
	iTS->M = iTS->m * S_eu;							//	M = m * S_eu
	iTS->Bm = iTS->bm / S_HOCam * (S_euv);			//	Bm = (bm / H) * (S_eu / S_ev)
	iTS->LaneR.B_i = iTS->br / S_HOCam * (S_euv);	//	Br = (br / H) * (S_eu / S_ev)
	iTS->LaneL.B_i = iTS->bl / S_HOCam * (S_euv);	//	Bl = (bl / H) * (S_eu / S_ev)

	memset(iTS->L_UVXY, 0, 11*sizeof(double));


#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_CURVE)
	iTS->L_LaneModelType = CURVE;
#endif

#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_LINE)
	iTS->L_LaneModelType = LINE;
#endif
}

short Replaced_SingleLane_With_DualLane(ITS *iTS)
{

#ifdef L_PROTECTION_MAIN_ON_OFF
#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
	//於單邊機制啟動(已切成單車道)的情況下,機制啟動側的車道線若不為零則會開始計數(--),
	//直到Left_Draw或Right_Draw為1時才不return 0,此時單車道切回雙車道
	if(iTS->Protection_Sideblock_Left_Draw == 0 || iTS->Protection_Sideblock_Right_Draw == 0)
	{
		return 0;
	}
#endif //L_PROTECTION_SIDE_BLOCK_ON_OFF
#endif //L_PROTECTION_MAIN_ON_OFF

	if(iTS->SupersedeCounter == 0)
	{
		Dual_ReplacedBy_Single_Reset(iTS);
		if(!L_DualLM_Searching(iTS))
			return 0;
		iTS->SupersedeCounter = LimitCount(++ iTS->SupersedeCounter, 20);
		return iTS->SupersedeCounter;
	}
	else
	{
		if(!L_DualLM_Tracking(iTS))
		{
			iTS->SupersedeCounter = LimitCount(-- iTS->SupersedeCounter, 20);
			return 0;
		}

		//iTS->SupersedeCounter = LimitCount(++ iTS->SupersedeCounter, 20); //20101111
		iTS->SupersedeCounter = LimitCount(iTS->SupersedeCounter += 3, 20);

		return iTS->SupersedeCounter;
	}
}

short Replaced_DualLane_With_SingleLane(ITS *iTS)
{
	iTS->SL_LaneModel.L_StbCtr = 20;

#ifdef L_PROTECTION_MAIN_ON_OFF
#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
	//根據Left_Draw與Right_Draw來判斷切左切右(切左)
	if(iTS->Protection_Sideblock_Left_Draw == 1 && iTS->Protection_Sideblock_Right_Draw == 0)
	{
		iTS->SL_LaneModel.L_SL_LorR = SL_Left;
		iTS->SL_LaneModel.LaneM.Sd = L_LtSd;
	}
	//根據Left_Draw與Right_Draw來判斷切左切右(切右)
	else if(iTS->Protection_Sideblock_Left_Draw == 0 && iTS->Protection_Sideblock_Right_Draw == 1)
	{
		iTS->SL_LaneModel.L_SL_LorR = SL_Right;
		iTS->SL_LaneModel.LaneM.Sd = L_RtSd;
	}
	else
	{
#endif //L_PROTECTION_SIDE_BLOCK_ON_OFF
#endif //L_PROTECTION_MAIN_ON_OFF

		//left bias and left side lane marking has enough edge point
		if(iTS->L_BiasWarn > 0 && iTS->LaneL.PixCtr > iTS->LaneR.PixCtr)
		{
			iTS->SL_LaneModel.L_SL_LorR = SL_Left;
			iTS->SL_LaneModel.LaneM.Sd = L_LtSd;
		}
		//right bias and right side lane marking has enough edge point
		else if(iTS->L_BiasWarn < 0 && iTS->LaneR.PixCtr > iTS->LaneL.PixCtr)
		{
			iTS->SL_LaneModel.L_SL_LorR = SL_Right;
			iTS->SL_LaneModel.LaneM.Sd = L_RtSd;
		}
		//others
		else
		{
            if(iTS->L_Bias > 5)
			{
				iTS->SL_LaneModel.L_SL_LorR = SL_Left;
				iTS->SL_LaneModel.LaneM.Sd = L_LtSd;
			}
            else if(iTS->L_Bias < -5)
			{
				iTS->SL_LaneModel.L_SL_LorR = SL_Right;
				iTS->SL_LaneModel.LaneM.Sd = L_RtSd;
			}
			else
				return 0;
		}

#ifdef L_PROTECTION_MAIN_ON_OFF
#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
	}
#endif //L_PROTECTION_SIDE_BLOCK_ON_OFF
#endif //L_PROTECTION_MAIN_ON_OFF

	iTS->SL_LaneModel.k = iTS->k;
	iTS->SL_LaneModel.m = iTS->m;
	iTS->SL_LaneModel.bm = iTS->bm;
	iTS->SL_LaneModel.br = iTS->br;
	iTS->SL_LaneModel.bl = iTS->bl;
	iTS->SL_LaneModel.K = iTS->K;
	iTS->SL_LaneModel.M = iTS->M;
	iTS->SL_LaneModel.Bm = iTS->Bm;
	iTS->SL_LaneModel.LaneR.B_i = iTS->LaneR.B_i;
	iTS->SL_LaneModel.LaneL.B_i = iTS->LaneL.B_i;

    iTS->SL_LaneModel.WAvg = iTS->L_WAvg;
    iTS->SL_LaneModel.WCur = iTS->L_WCur;

	iTS->SL_LaneModel.V[0] = iTS->V[0];
	iTS->SL_LaneModel.V[1] = iTS->V[1];

	iTS->SL_LaneModel.L_UVXY[0] = iTS->L_UVXY[0];
	iTS->SL_LaneModel.L_UVXY[1] = iTS->L_UVXY[1];
	iTS->SL_LaneModel.L_UVXY[2] = iTS->L_UVXY[2];
	iTS->SL_LaneModel.L_UVXY[3] = iTS->L_UVXY[3];
	iTS->SL_LaneModel.L_UVXY[4] = iTS->L_UVXY[4];
	iTS->SL_LaneModel.L_UVXY[5] = iTS->L_UVXY[5];
	iTS->SL_LaneModel.L_UVXY[6] = iTS->L_UVXY[6];
	iTS->SL_LaneModel.L_UVXY[7] = iTS->L_UVXY[7];
	iTS->SL_LaneModel.L_UVXY[8] = iTS->L_UVXY[8];
	iTS->SL_LaneModel.L_UVXY[9] = iTS->L_UVXY[9];
	iTS->SL_LaneModel.L_UVXY[10] = iTS->L_UVXY[10];


#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF

	//Right Side
	if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_0)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_0;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_0;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_1)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_1;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_1;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_2)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_2;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_2;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_3)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_3;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_3;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_4)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_4;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_4;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_5)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_5;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_5;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_6)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_6;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_6;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_7;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_7;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_8)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_8;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_8;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_9)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_9;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_9;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_10)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_10;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_10;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_11)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_11;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_11;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_12;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_12;
	}
	else if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean > LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12)
	{
		iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_13;
		iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_13;
	}

	//Left Side
	if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_0)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_0;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_0;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_1)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_1;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_1;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_2)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_2;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_2;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_3)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_3;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_3;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_4)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_4;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_4;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_5)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_5;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_5;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_6)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_6;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_6;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_7;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_7;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_8)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_8;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_8;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_9)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_9;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_9;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_10)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_10;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_10;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_11)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_11;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_11;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_12;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_12;
	}
	else if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean > LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12)
	{
		iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_13;
		iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_13;
	}
#endif

#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_CURVE)
	iTS->SL_LaneModel.LaneModelType = CURVE;
#endif

#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_LINE)
	iTS->SL_LaneModel.LaneModelType = LINE;
#endif

	iTS->SL_SupersedeFlag = REGULAR_PROCESS;

	return 1;
}

#endif
