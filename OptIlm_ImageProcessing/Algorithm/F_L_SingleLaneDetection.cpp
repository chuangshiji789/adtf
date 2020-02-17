#include "InitialVariable.h"
#include "FunctionType.h"

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
void L_RstSingleLM_Variables(ITS *iTS)
{
	//偏移警示 flag and counter
	iTS->L_BiasWarn = 0;
	iTS->L_last_bias_flag = 0;
	iTS->L_Bias = 0;

	//模式設定
	iTS->L_DetectMode = SL_SEARCH;

	//狀態
    iTS->SL_LaneModel.evRodSlp = EV_ROAD_SLOPE;
	iTS->SL_LaneModel.WAvg = L_RW_DefaultLW;
    iTS->SL_LaneModel.WCur = L_SINGLE_LANE_DEFAULT_L_W;

	//單車道線自己的狀態變數
	iTS->SL_LaneModel.L_SL_LorR = SL_NotFound;
	iTS->L_SL_PointGroup.LastUPosition = 0;
	iTS->L_SL_PointGroup.LastVPosition = 0;
	iTS->L_SL_PointGroup.RightCounter = 0;
	iTS->L_SL_PointGroup.FindLaneCounter = 0;
	iTS->L_SL_PointGroup.UpdateCounter = 0;
	iTS->L_SL_PointGroup.SL_StableCounter = 0;

	//車道模型的k,m,b值
    iTS->SL_LaneModel.k = iTS->default_k;//L_MP_kDefault;							//curvature
    iTS->SL_LaneModel.m = iTS->default_m;//L_MP_mDefault;							//orientation
    iTS->SL_LaneModel.bm = iTS->default_b;//L_MP_bDefault;						//shifting
    iTS->SL_LaneModel.br = iTS->default_b + (L_RW_ROILW >> 1);		//right lane mark shifting
    iTS->SL_LaneModel.bl = iTS->default_b - (L_RW_ROILW >> 1);		//left lane mark shifting
	iTS->SL_LaneModel.K = iTS->SL_LaneModel.k * S_eu * S_ev * S_HOCam;		//K = k * S_eu * S_ev * H
	iTS->SL_LaneModel.M = iTS->SL_LaneModel.m * S_eu;							//M = m * S_eu
	iTS->SL_LaneModel.BM = iTS->SL_LaneModel.bm / (S_HOCam * S_euv);			//Bm = (bm / H) * (S_eu / S_ev)
	iTS->SL_LaneModel.LaneR.B_i = iTS->SL_LaneModel.br / S_HOCam * (S_euv);	//Br = (br / H) * (S_eu / S_ev)
	iTS->SL_LaneModel.LaneL.B_i = iTS->SL_LaneModel.bl / S_HOCam * (S_euv);	//Bl = (bl / H) * (S_eu / S_ev)
	iTS->SL_LaneModel.LaneM.B_i = 0;	//Bm = (bm / H) * (S_eu / S_ev)

	//reset U, V, X, and Y 係數
	memset(iTS->SL_LaneModel.L_UVXY, 0, 11 * sizeof(double));


	iTS->SL_LaneModel.V[0] = 0;
	iTS->SL_LaneModel.V[1] = 0;

	iTS->MaximumErrorDistance = 0;

#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF
	//Very very important!!!
	/*if(iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean == 0 || iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean > 255)
		iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean = LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_3;
	if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean == 0 || iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean > 255)
		iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean = LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_3;*/

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

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	iTS->SL_SupersedeFlag = REGULAR_PROCESS;
	iTS->SupersedeCounter = 0;
#endif

	iTS->SL_LaneModel.Flag_RideMarking = 0;

	iTS->SL_LaneModel.Estimation_Last_u1 = 0;
	iTS->SL_LaneModel.Estimation_Last_u2 = 0;
	iTS->SL_LaneModel.Estimation_Last_u3 = 0;
	iTS->SL_LaneModel.Estimation_Last_u4 = 0;
}

void L_RstSingleLM_Tracking_Variables(ITS *iTS)
{
	iTS->L_SL_PointGroup.LastUPosition = 0;
	iTS->L_SL_PointGroup.LastVPosition = 0;
	iTS->L_SL_PointGroup.RightCounter = 0;
	iTS->L_SL_PointGroup.FindLaneCounter = 0;
	iTS->L_SL_PointGroup.UpdateCounter = 0;
	iTS->MaximumErrorDistance = 0;
    iTS->SL_LaneModel.evRodSlp = EV_ROAD_SLOPE;
}

//指定單車道線搜尋ROI範圍
void L_SingleLM_SpecifyROI(short ROI_Mode, LaneInfo *Lane, ITS *iTS)
{
        double dBU = 0;
        double P = 0;


	switch(ROI_Mode)
	{
		case SL_SCH_DEFAULT_ROI:
			P = iTS->F_W_C+ LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneM.B_i);
			dBU = iTS->SL_LaneModel.Vrow* L_SINGLE_LANE_DEFAULT_REGION / S_HOCam * S_euv;
			break;
		case SL_SCH_MAIN_ROI:
            P = Lane->LstCol + LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneM.B_i);
			dBU = iTS->SL_LaneModel.Vrow* L_SINGLE_LANE_MAIN_REGION / S_HOCam * S_euv;
			break;
		case SL_SCH_SUB_ROI:
			P = Lane->LstCol + LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneM.B_i);
			dBU = iTS->SL_LaneModel.Vrow* L_SINGLE_LANE_SUB_REGION / S_HOCam * S_euv;
			break;
		default:
			//什麼都不做
			break;
	}

    SetROI(&Lane->SchRoi,S_IMGLB + 1, S_IMGRB - 1, (short)P, (short)(dBU));

 #ifdef DRAW_LANE_MARK_ROI
	//Draw ROI Debug
	short row_;
	row_ = (short)(iTS->L_evRodSlp -iTS->SL_LaneModel.Vrow) +iTS->F_H_C;

        switch(ROI_Mode)
        {
                case SL_SCH_DEFAULT_ROI:
                        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Frt]=255;
                        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Scd]=255;
                        break;
                case SL_SCH_MAIN_ROI:
                        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Frt]=128;
                        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Scd]=128;
                        break;
                case SL_SCH_SUB_ROI:
                        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Frt]=0;
                        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Scd]=0;
                        break;
                default:
                        //什麼都不做
                        break;
        }


#endif
}

void SingleLane_Tracking_SpecifyROI(short ROI_Mode, LaneInfo *Lane, ITS *iTS)
{
	double dBU;
	double P = 0;

	switch(ROI_Mode)
	{
		case SL_TCK_MAIN_ROI:
			if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
				P = iTS->F_W_C+ LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneR.B_i);
			if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
				P = iTS->F_W_C+ LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneL.B_i);
			dBU = iTS->SL_LaneModel.Vrow* L_SINGLE_TRACE_MAIN_REGION / S_HOCam * S_euv;
			break;
		case SL_TCK_SUB_ROI:
			if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
				P = iTS->F_W_C+ LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneR.B_i);
			if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
				P = iTS->F_W_C+ LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneL.B_i);
			dBU = iTS->SL_LaneModel.Vrow* L_SINGLE_TRACE_SUB_REGION / S_HOCam * S_euv;
			break;
		default:
			if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
				P = iTS->F_W_C+ LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneR.B_i);
			if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
				P = iTS->F_W_C+ LaneModel(iTS->SL_LaneModel.Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, iTS->SL_LaneModel.LaneL.B_i);
			dBU = iTS->SL_LaneModel.Vrow* L_SINGLE_TRACE_MAIN_REGION / S_HOCam * S_euv;
			break;
	}

	SetROI(&Lane->SchRoi,S_IMGLB + 1, S_IMGRB - 1, (short)P, (short)(dBU));

#ifdef DRAW_LANE_MARK_ROI
	//Draw ROI Debug
		short row_;
        row_ = (short)(iTS->SL_LaneModel.evRodSlp -iTS->SL_LaneModel.Vrow) +iTS->F_H_C;
                iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Frt]=0;
                iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Scd]=0;
#endif
}

short IsSingleMark(LaneInfo *Lane,short row_,UBYTE *pSrc, ITS *iTS)
{
	short i;
	short begin, end;
	short MinMW = DWtoDI(L_RW_MinMarkW, iTS->SL_LaneModel.Vrow);
	short MaxMW = DWtoDI(L_RW_MaxMarkW, iTS->SL_LaneModel.Vrow);
    short mark_width = 0;

    if(MaxMW <= 2)
		return 0;

	if(MinMW <= 1)
        return 0;

    if ((Lane->SchRoi.Scd - Lane->SchRoi.Frt + 1) <= MinMW )
    {
        return 0;
    }
#if(L_SINGLE_LANE_SEARCH_DIRECTION == L_SINGLE_LANE_SEARCH_LEFT_START)		 	//left to right
	begin = Lane->SchRoi.Frt;
	end = Lane->SchRoi.Scd;
#endif

#if(L_SINGLE_LANE_SEARCH_DIRECTION == L_SINGLE_LANE_SEARCH_RIGHT_START) 		//right to left
	begin = Lane->SchRoi.Scd;
	end = Lane->SchRoi.Frt;
#endif

#if(L_SINGLE_LANE_SEARCH_DIRECTION == L_SINGLE_LANE_SEARCH_MERGE_DIRECTION)     //merge direction
	if(iTS->SingleLaneDirectionSwitch == 0)    //left to right
	{
		begin = Lane->SchRoi.Frt;
		end = Lane->SchRoi.Scd;
	}
	else									   //right to left
	{
		begin = Lane->SchRoi.Scd;
		end = Lane->SchRoi.Frt;
	}
#endif

#if(L_SINGLE_LANE_SEARCH_DIRECTION != L_SINGLE_LANE_SEARCH_MERGE_DIRECTION)    //not merge direction
#if(L_SINGLE_LANE_SEARCH_DIRECTION == L_SINGLE_LANE_SEARCH_RIGHT_START)        //left to right
	for(i = begin; i > end; i--)
#endif

#if(L_SINGLE_LANE_SEARCH_DIRECTION == L_SINGLE_LANE_SEARCH_LEFT_START)    //right to left
	for(i = begin; i < end; i++)
#endif

	{
		if (MarkSearch(i,row_,Lane,MaxMW,MinMW,pSrc,iTS ))
		{
			iTS->L_SL_PointGroup.FindLaneCounter ++;
			Lane->CurCol = i;

			if(iTS->L_SL_PointGroup.LastUPosition == 0)
			{
				iTS->L_SL_PointGroup.RightCounter ++;
				iTS->L_SL_PointGroup.LastUPosition = Lane->CurCol;
				iTS->L_SL_PointGroup.LastVPosition = row_;
			}
			else
			{
				if(abs(iTS->L_SL_PointGroup.LastUPosition - Lane->CurCol) < L_SINGLE_LANE_U_SHIFT_TH)
				{
					iTS->L_SL_PointGroup.RightCounter ++;
					iTS->L_SL_PointGroup.LastUPosition = Lane->CurCol;
					iTS->L_SL_PointGroup.LastVPosition = row_;
					return 1;
				}
			}
		}
	}
#else 																	  //merge direction
	if(iTS->SingleLaneDirectionSwitch == 0)    //left to right
	{
        for(i = begin; i < end; i++)
		{
			if (MarkSearch(i,row_,Lane,MaxMW,MinMW,pSrc,iTS))
			{
                mark_width = abs(Lane->MrkEdg.Frt - Lane->MrkEdg.Scd);
                if(iTS->L_SL_PointGroup.FindLaneCounter == 0)
                    iTS->L_SL_PointGroup.mark_width = (DItoDW((double)mark_width, iTS->L_evRodSlp - (LimitVP(row_) - iTS->F_H_C + 1 )) - 1);
                else
                {
                    iTS->L_SL_PointGroup.mark_width += (DItoDW((double)mark_width, iTS->L_evRodSlp - (LimitVP(row_) - iTS->F_H_C + 1 )) - 1);
                    iTS->L_SL_PointGroup.mark_width *= 0.5;
                }

                iTS->L_SL_PointGroup.FindLaneCounter ++;
				Lane->CurCol = i;

#ifdef	L_LOSS_LANE_FOR_BIAS_TO_LARGE
				if((iTS->L_DetectMode == SL_TRACE || iTS->L_DetectMode == SL_SEARCH) && iTS->L_SingleFlag == 1)
				{

#ifdef	L_SHOW_FIRST_ROW
					if(iTS->L_SingleFlag == 1)
					{
						OSD_Color_Setup(OCN_ORANGE, iTS);
						ScalableNumber(row_, 3, S_IMGRB - 10, S_IMGTB - 10, iTS);
					}
#endif    //L_SHOW_FIRST_ROW

					if(row_ >= L_SINGLE_FIRST_ROW_TO_UP && (i >= S_IMGRB - L_SINGLE_FIRST_ROW_TO_SIDE || i <= S_IMGLB + L_SINGLE_FIRST_ROW_TO_SIDE))
					{
						iTS->L_DetectMode = SL_SEARCH;
						iTS->SL_LaneModel.L_StbCtr = 0;
					}
					iTS->L_SingleFlag = 2;
				}
#endif    //L_LOSS_LANE_FOR_BIAS_TO_LARGE

				if(iTS->L_SL_PointGroup.LastUPosition == 0)
				{
                    iTS->L_SL_PointGroup.RightCounter = 1;
					iTS->L_SL_PointGroup.LastUPosition = Lane->CurCol;
					iTS->L_SL_PointGroup.LastVPosition = row_;
				}
				else
				{
					if(abs(iTS->L_SL_PointGroup.LastUPosition - Lane->CurCol) < L_SINGLE_LANE_U_SHIFT_TH)
					{
						iTS->L_SL_PointGroup.RightCounter ++;
						iTS->L_SL_PointGroup.LastUPosition = Lane->CurCol;
						iTS->L_SL_PointGroup.LastVPosition = row_;
						return 1;
					}
//                    else
//                    {
//                        if(iTS->L_SL_PointGroup.RightCounter > iTS->L_SL_PointGroup.MaxRightCounter)
//                            iTS->L_SL_PointGroup.MaxRightCounter = iTS->L_SL_PointGroup.RightCounter;

//                        iTS->L_SL_PointGroup.RightCounter = 1;
//                        iTS->L_SL_PointGroup.LastUPosition = Lane->CurCol;
//                        iTS->L_SL_PointGroup.LastVPosition = row_;
//                        return 0;
//                    }
				}
			}
		}
	}
	else									   //right to left
	{
        for(i = begin; i > end; i--)
		{
			if (MarkSearch(i,row_,Lane,MaxMW,MinMW,pSrc,iTS ))
			{
                mark_width = abs(Lane->MrkEdg.Frt - Lane->MrkEdg.Scd);
                if(iTS->L_SL_PointGroup.FindLaneCounter == 0)
                    iTS->L_SL_PointGroup.mark_width = (DItoDW((double)mark_width, iTS->L_evRodSlp - (LimitVP(row_) - iTS->F_H_C + 1 )) - 1);
                else
                {
                    iTS->L_SL_PointGroup.mark_width += (DItoDW((double)mark_width, iTS->L_evRodSlp - (LimitVP(row_) - iTS->F_H_C + 1 )) - 1);
                    iTS->L_SL_PointGroup.mark_width *= 0.5;
                }


                iTS->L_SL_PointGroup.FindLaneCounter ++;


				Lane->CurCol = i;

#ifdef	L_LOSS_LANE_FOR_BIAS_TO_LARGE
				if((iTS->L_DetectMode == SL_TRACE || iTS->L_DetectMode == SL_SEARCH) && iTS->L_SingleFlag == 1)
				{

#ifdef	L_SHOW_FIRST_ROW
					if(iTS->L_SingleFlag == 1)
					{
						OSD_Color_Setup(OCN_ORANGE, iTS);
						ScalableNumber(row_, 3, S_IMGRB - 10, S_IMGTB - 10, iTS);
					}
#endif    //L_SHOW_FIRST_ROW

					if(row_ >= L_SINGLE_FIRST_ROW_TO_UP && (i >= S_IMGRB - L_SINGLE_FIRST_ROW_TO_SIDE || i <= S_IMGLB + L_SINGLE_FIRST_ROW_TO_SIDE))
					{
						iTS->L_DetectMode = SL_SEARCH;
						iTS->SL_LaneModel.L_StbCtr = 0;
					}
					iTS->L_SingleFlag = 2;
				}
#endif    //L_LOSS_LANE_FOR_BIAS_TO_LARGE

				if(iTS->L_SL_PointGroup.LastUPosition == 0)
				{
                    iTS->L_SL_PointGroup.RightCounter = 1;
					iTS->L_SL_PointGroup.LastUPosition = Lane->CurCol;
					iTS->L_SL_PointGroup.LastVPosition = row_;
				}
				else
				{
					if(abs(iTS->L_SL_PointGroup.LastUPosition - Lane->CurCol) < L_SINGLE_LANE_U_SHIFT_TH)
					{
						iTS->L_SL_PointGroup.RightCounter ++;
						iTS->L_SL_PointGroup.LastUPosition = Lane->CurCol;
						iTS->L_SL_PointGroup.LastVPosition = row_;
						return 1;
					}
//                    else
//                    {
//                        if(iTS->L_SL_PointGroup.RightCounter > iTS->L_SL_PointGroup.MaxRightCounter)
//                            iTS->L_SL_PointGroup.MaxRightCounter = iTS->L_SL_PointGroup.RightCounter;

//                        iTS->L_SL_PointGroup.RightCounter = 1;
//                        iTS->L_SL_PointGroup.LastUPosition = Lane->CurCol;
//                        iTS->L_SL_PointGroup.LastVPosition = row_;
//                        return 0;
//                    }

				}
			}

		}
	}
#endif

    //TEST 12112017
    if((abs(iTS->L_SL_PointGroup.LastVPosition - row_) > L_SINGLE_LANE_V_SHIFT_TH) &&
		iTS->L_SL_PointGroup.RightCounter > 0)
    {
		iTS->L_SL_PointGroup.RightCounter --;
        //iTS->im_adebug = 55;
    }
    else{
		iTS->L_SL_PointGroup.LastUPosition = 0;
        //iTS->im_debug = 66;
    }

	return 0;
}

short L_SingleLM_UpdateModel(short row_,LaneInfo *Lane,ITS *iTS)
{
    short temp; //difference side lane marking position

    if(iTS->L_DetectMode == SL_SEARCH)
    {
        if(iTS->L_SL_PointGroup.FindLaneCounter >= L_SINGLE_LANE_CHANGE_ROI_TH)
        {
            if(Lane->CurCol >S_IMGCW)	//find right lane
            {
                iTS->SL_LaneModel.L_SL_LorR = SL_Right;
                Lane->Sd = L_RtSd;
            }
            else 						//find left lane
            {
                iTS->SL_LaneModel.L_SL_LorR = SL_Left;
                Lane->Sd = L_LtSd;
            }

            if(iTS->L_SL_PointGroup.FindLaneCounter > 50)
            {
                iTS->SL_LaneModel.L_SL_LorR = SL_Right;
                Lane->Sd = L_RtSd;
            }


        }
        else if(iTS->L_SL_PointGroup.FindLaneCounter < L_SINGLE_LANE_CHANGE_ROI_TH)
        {
            iTS->SL_LaneModel.L_SL_LorR = SL_NotFound;
            return 0;
        }

        iTS->debug_info = 0;
    }

    if(iTS->L_DetectMode == SL_TRACE)
    {

        if(iTS->L_SL_PointGroup.FindLaneCounter > 50)
        {
            iTS->SL_LaneModel.L_SL_LorR = SL_Right;
            Lane->Sd = L_RtSd;
        }


        iTS->debug_info = 0;
    }

//    if(iTS->L_SL_PointGroup.RightCounter >= L_SINGLE_LANE_CORRECT_POINT_TH && iTS->L_DetectMode == SL_SEARCH && iTS->SL_LaneModel.L_SL_LorR == SL_NotFound )
//    {
//        if(Lane->CurCol > S_IMGCW || iTS->L_SL_PointGroup.FindLaneCounter > 60)	//find right lane
//        {
//            iTS->SL_LaneModel.L_SL_LorR = SL_Right;
//            Lane->Sd = L_RtSd;
//        }
//        else						//find left lane
//        {
//            iTS->SL_LaneModel.L_SL_LorR = SL_Left;
//            Lane->Sd = L_LtSd;
//        }
//    }
//    else if(iTS->L_SL_PointGroup.RightCounter < L_SINGLE_LANE_CORRECT_POINT_TH && iTS->L_DetectMode == SL_SEARCH)
//    {
//        iTS->SL_LaneModel.L_SL_LorR = SL_NotFound;
//        return 0;
//    }
//    else if(iTS->L_DetectMode == SL_TRACE)
//    {
//        if(iTS->L_SL_PointGroup.mark_width > 2.3 /*&& iTS->L_SL_PointGroup.FindLaneCounter > 60*/)  //find right lane
//        {
//            iTS->SL_LaneModel.L_SL_LorR = SL_Right;
//            Lane->Sd = L_RtSd;

//            iTS->debug_info = 1;
//        }
////        else if(Lane->CurCol > S_IMGCW && iTS->L_SL_PointGroup.FindLaneCounter > 65)	//find right lane
////        {
////            iTS->SL_LaneModel.L_SL_LorR = SL_Right;
////            Lane->Sd = L_RtSd;

////            iTS->debug_info = 2;
////        }
//        else						//find left lane
//        {
//            iTS->SL_LaneModel.L_SL_LorR = SL_Left;
//            Lane->Sd = L_LtSd;

//            iTS->debug_info = 3;
//        }
//    }


    if(iTS->L_SL_PointGroup.FindLaneCounter > L_SINGLE_LANE_CHANGE_ROI_TH &&
        iTS->L_SL_PointGroup.RightCounter > L_SINGLE_LANE_CORRECT_POINT_TH)
//    if(iTS->L_SL_PointGroup.RightCounter > L_SINGLE_LANE_CHANGE_ROI_TH)
	{
		if(iTS->SL_LaneModel.L_SL_LorR == SL_Right) //find right lane
		{
			temp = Lane->CurCol - DWtoDI(iTS->SL_LaneModel.WCur, iTS->SL_LaneModel.Vrow);
			L_SingleLM_AddLanMdl(1, row_, temp, Lane->CurCol, iTS);
		}
		if(iTS->SL_LaneModel.L_SL_LorR == SL_Left) //find left lane
		{
			temp = Lane->CurCol + DWtoDI(iTS->SL_LaneModel.WCur, iTS->SL_LaneModel.Vrow);
			L_SingleLM_AddLanMdl(1, row_, Lane->CurCol, temp, iTS);
		}

		iTS->L_SL_PointGroup.UpdateCounter ++;

#ifdef DRAW_LANE_MARK
        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->CurCol]=0;
#endif

		return 1;
	}

	return 0;
}

short L_SingleLM_MarkSearch(short row_,ITS *iTS)
{

#ifdef DRAW_LANE_MARKING_AND_ROAD_SURFACE_AREA
	iTS->LM_R_GrayScale_AdaptiveThresholding.ScanRow = (int) (iTS->F_H-row_ - 1) * iTS->F_W;
	iTS->LM_L_GrayScale_AdaptiveThresholding.ScanRow = (int) (iTS->F_H-row_ - 1) * iTS->F_W;
#endif

	if (IsSingleMark(&iTS->SL_LaneModel.LaneM, row_, &iTS->YImg[(iTS->F_H-row_-1) * iTS->F_W], iTS)) //lane marking searching function
	{
		L_SingleLM_UpdateModel(row_, &iTS->SL_LaneModel.LaneM, iTS);
		return 1;
	}

	return 0;
}

short L_SingleLM_RoadModel(ITS *iTS)
{
	short u1 = 0;				//first row u position
	short u2 = 0;				//last row u position
	short u3 = 0;				//first row u position
	short u4 = 0;				//last row u position
    short v1 = L_SINGLE_LANE_SEARCH_BB;//30;				//first row
    short v2 = L_SINGLE_LANE_SEARCH_TB;//S_IMGCH - 30;	//last row

    short v3 = S_IMGCH - 10;
    short u5 = 0;
	short u6 = 0;
    short uvc = 0;

	short Return_Value = 1;


//    if(iTS->SL_LaneModel.K >= 3000.0 || iTS->SL_LaneModel.K <= -3000.0)
//    {
//        iTS->single_lane_debug = 9;
////        Return_Value = 0;
//    }
    if(fabs(iTS->SL_LaneModel.M) > 500.0)  //360
    {
        iTS->single_lane_debug = 9;
        Return_Value = 0;
    }
//    if(iTS->SL_LaneModel.bm >= 50 || iTS->SL_LaneModel.bm <= -50)    //12112017
    if(fabs(iTS->SL_LaneModel.bm) > 50)    //12112017
    {
        iTS->single_lane_debug = 2;
        Return_Value = 0;
    }
    if(iTS->MaximumErrorDistance >= 80) //200 Pixels
    {
        iTS->single_lane_debug = 3;
        Return_Value = 0;
    }
	u1 = iTS->L_LaneMBound[v1].Frt;	//Left
	u2 = iTS->L_LaneMBound[v2].Frt; //Left
	u3 = iTS->L_LaneMBound[v1].Scd; //right
	u4 = iTS->L_LaneMBound[v2].Scd; //right
    u5 = iTS->L_LaneMBound[v3].Frt; //Left
	u6 = iTS->L_LaneMBound[v3].Scd; //right
    uvc = (u5 + u6) >> 1; 			//Find center point

	//Reset searching direction
	//Find left marking
	if(iTS->L_BiasWarn == 0 && u1 <= (S_IMGLB + 10) && iTS->SL_LaneModel.L_SL_LorR == SL_Left)
		iTS->SingleLaneDirectionSwitch = 0;
	//Find right marking
	if(iTS->L_BiasWarn == 0 && u3 >= (S_IMGRB - 10) && iTS->SL_LaneModel.L_SL_LorR == SL_Right)
		iTS->SingleLaneDirectionSwitch = 1;

	//Ride marking
	if(((u1 >= (S_IMGCW - (S_IMGCW >> 3)) && u1 <= (S_IMGCW + (S_IMGCW >> 3))) && iTS->SL_LaneModel.L_SL_LorR == SL_Left) || //Left marking
		((u3 >= (S_IMGCW - (S_IMGCW >> 3)) && u3 <= (S_IMGCW + (S_IMGCW >> 3))) && iTS->SL_LaneModel.L_SL_LorR == SL_Right)) //Right marking
		iTS->SL_LaneModel.Flag_RideMarking = 1;
	else
		iTS->SL_LaneModel.Flag_RideMarking = 0;


	if(iTS->SL_LaneModel.Estimation_Last_u1 == 0 && iTS->SL_LaneModel.Estimation_Last_u2 == 0 &&
		iTS->SL_LaneModel.Estimation_Last_u3 == 0 && iTS->SL_LaneModel.Estimation_Last_u4 == 0)
	{
		iTS->SL_LaneModel.Estimation_Last_u1 = u1;
		iTS->SL_LaneModel.Estimation_Last_u2 = u2;
		iTS->SL_LaneModel.Estimation_Last_u3 = u3;
		iTS->SL_LaneModel.Estimation_Last_u4 = u4;
	}

	if(abs(u1 - iTS->SL_LaneModel.Estimation_Last_u1) > (S_IMGCW >> 2) ||
        abs(u2 - iTS->SL_LaneModel.Estimation_Last_u2) > (S_IMGCW >> 2) ||
        abs(u3 - iTS->SL_LaneModel.Estimation_Last_u3) > (S_IMGCW >> 2) ||
        abs(u4 - iTS->SL_LaneModel.Estimation_Last_u4) > (S_IMGCW >> 2))
    {
        iTS->single_lane_debug = 4;
        Return_Value = 0;
    }

//    if(abs(uvc - S_IMGCW) > (S_IMGW / 6))
//    {
//        iTS->single_lane_debug = 8;
////        Return_Value = 0;
//    }

    if(abs(u1 - u2) > S_IMGCW)
    {
        iTS->single_lane_debug = 5;
        Return_Value = 0;
    }
    if(abs(u3 - u4) > S_IMGCW)
    {
        iTS->single_lane_debug = 6;
        Return_Value = 0;
    }

	if(!Return_Value) //Estimation failure
	{
		if(iTS->SL_LaneModel.L_StbCtr <= 10)
		{
			iTS->L_BiasWarn = 0;
			iTS->L_last_bias_flag = 0;
			iTS->L_Bias = 0;
		}

		//Reset last position of image
		iTS->SL_LaneModel.Estimation_Last_u1 = 0;
		iTS->SL_LaneModel.Estimation_Last_u2 = 0;
		iTS->SL_LaneModel.Estimation_Last_u3 = 0;
		iTS->SL_LaneModel.Estimation_Last_u4 = 0;
	}
	else
	{
		iTS->SL_LaneModel.Estimation_Last_u1 = u1;
		iTS->SL_LaneModel.Estimation_Last_u2 = u2;
		iTS->SL_LaneModel.Estimation_Last_u3 = u3;
		iTS->SL_LaneModel.Estimation_Last_u4 = u4;
	}




#ifdef SINGLE_LANE_ROAD_MODEL_DEBUG
    DrawRect(iTS->F_H - v1, u1, 10, 255, iTS);  //first row u position
    DrawRect(iTS->F_H - v2, u2, 10, 128, iTS);    //last row u position
    DrawRect(iTS->F_H - v1, u3, 10, 255, iTS);  //first row u position
    DrawRect(iTS->F_H - v2, u4, 10, 0, iTS);    //last row u position

    DrawRect(iTS->F_H - v3, u5, 10, 255, iTS);
    DrawRect(iTS->F_H - v3, u6, 10, 255, iTS);
    DrawRect(iTS->F_H - v3, uvc, 10, 80, iTS);   //Find center point
#endif


	return Return_Value;
}

short L_SingleLM_DecideModeOrder(ITS *iTS)
{

    if(iTS->L_SL_PointGroup.FindLaneCounter > L_SINGLE_LANE_LINE_CURVE_TH)
        iTS->SL_LaneModel.LaneModelType = CURVE;
    else
        iTS->SL_LaneModel.LaneModelType = LINE;

	return 1;
}

void L_SingleLM_MeanSquareAddingData(double weight,double x,double y,double MatrixA[5],double VectorB[3])
{
	short i;

	for(i = 0;i < 5; i++)
	{
		MatrixA[i] += weight;
		if(i < 3)
			VectorB[i] += weight * y;
		weight *=x;
	}
}

void L_SingleLM_AddLanMdl(register short weight,register short row_,register short L,register short R, ITS *iTS)
{
	double du_,u_,v_;

	if((R-L)>0)
		du_=(R-L+1);
	else
		du_=(L-R+1);
	u_ = (((R+L)>>1)-iTS->F_W_C) * du_;
	v_ = row_ - iTS->F_H_C-1;

	L_SingleLM_MeanSquareAddingData(weight,du_,u_,iTS->SL_LaneModel.dU,iTS->SL_LaneModel.U);
	iTS->SL_LaneModel.Um[0]+=weight*(((R+L)>>1)-iTS->F_W_C);
	iTS->SL_LaneModel.V[0] += weight * v_;
	iTS->SL_LaneModel.V[1] += weight * v_ * du_;
}

void L_SingleLM_BuildLaneModel(ITS *iTS)
{
	double Cxy[3];

#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_CURVE)
	iTS->SL_LaneModel.LaneModelType = CURVE;
#endif

#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_LINE)
	iTS->SL_LaneModel.LaneModelType = LINE;
#endif

	if(iTS->SL_LaneModel.LaneModelType == CURVE)
	{
		Upt_Coef_Fitting_CURVE(iTS->SL_LaneModel.dU, iTS->SL_LaneModel.U, Cxy);

		iTS->SL_LaneModel.k = Cxy[0] / (double)iTS->SL_LaneModel.WCur / (S_eu * S_eu);
	    iTS->SL_LaneModel.m = Cxy[1] / S_eu;
        iTS->SL_LaneModel.bm = Cxy[2] * iTS->SL_LaneModel.WCur;
	}
	else if(iTS->SL_LaneModel.LaneModelType == LINE)
	{
		Upt_Coef_Fitting_LINE(iTS->SL_LaneModel.dU, iTS->SL_LaneModel.Um, Cxy);

		iTS->SL_LaneModel.k=0.0;
	   	iTS->SL_LaneModel.m = Cxy[0] / S_eu;
       	iTS->SL_LaneModel.bm = Cxy[1] * iTS->SL_LaneModel.WCur;
	}

	iTS->SL_LaneModel.K = iTS->SL_LaneModel.k * S_eu * S_ev * S_HOCam;
	iTS->SL_LaneModel.M = iTS->SL_LaneModel.m * S_eu;
	iTS->SL_LaneModel.Bm = iTS->SL_LaneModel.bm / S_HOCam * (S_euv);
	iTS->SL_LaneModel.LaneR.B_i = (iTS->SL_LaneModel.bm + (iTS->SL_LaneModel.WCur>>1)) / S_HOCam * (S_euv);
	iTS->SL_LaneModel.LaneL.B_i = (iTS->SL_LaneModel.bm - (iTS->SL_LaneModel.WCur>>1)) / S_HOCam * (S_euv);



    Upt_Coef_Fitting_LINE(iTS->SL_LaneModel.dU, iTS->SL_LaneModel.Um, Cxy);
    iTS->Reference_LaneModel.k  = 0.0;
    iTS->Reference_LaneModel.m  = Cxy[0] / S_eu;
    iTS->Reference_LaneModel.bm = Cxy[1] * iTS->SL_LaneModel.WCur;
    iTS->Reference_LaneModel.K = iTS->Reference_LaneModel.k * S_eu * S_ev * S_HOCam;
    iTS->Reference_LaneModel.M = iTS->Reference_LaneModel.m * S_eu;
    iTS->Reference_LaneModel.Bm = iTS->Reference_LaneModel.bm / S_HOCam * (S_euv);
}

short L_SingleLM_Estimation(ITS *iTS)
{
	if(!L_SingleLM_RoadModel(iTS))
    {
        return 0;
    }

	L_SingleLM_DecideModeOrder(iTS);
	L_SingleLM_BuildLaneModel(iTS);
	return 1;
}

short L_SingleLM_Searching(ITS *iTS)
{

	short row_;
	short TempErrorDistance = 0;

	L_SingleLM_SpecifyROI(SL_SCH_DEFAULT_ROI, &iTS->SL_LaneModel.LaneM, iTS); 		//default ROI

	for(row_ = L_SINGLE_LANE_SEARCH_BB; row_ < L_SINGLE_LANE_SEARCH_TB; row_ ++) 	//edge point finding
	{

        iTS->SL_LaneModel.Vrow = iTS->SL_LaneModel.evRodSlp - (row_ - iTS->F_H_C);	//road slope adjustment

        if(L_SingleLM_MarkSearch(row_,iTS))	//the mark is found: the main process row by row
        {
			TempErrorDistance = abs(iTS->SL_LaneModel.LaneM.LstCol - iTS->SL_LaneModel.LaneM.CurCol);
			if(TempErrorDistance >= iTS->MaximumErrorDistance)
				iTS->MaximumErrorDistance = TempErrorDistance;

            if(iTS->L_SL_PointGroup.RightCounter >= L_SINGLE_LANE_CORRECT_POINT_TH)
            {
                iTS->SL_LaneModel.LaneM.LstCol = iTS->SL_LaneModel.LaneM.CurCol;
                L_SingleLM_SpecifyROI(SL_SCH_SUB_ROI, &iTS->SL_LaneModel.LaneM, iTS);	//minimum ROI
                iTS->SL_LaneModel.CurFinalRow_ =  row_;
            }
            else
            {
                iTS->SL_LaneModel.LaneM.LstCol = iTS->SL_LaneModel.LaneM.CurCol;
                L_SingleLM_SpecifyROI(SL_SCH_MAIN_ROI, &iTS->SL_LaneModel.LaneM, iTS);	//medium ROI
            }
        }
        else //mark not found
        {
            if(iTS->L_SL_PointGroup.FindLaneCounter <= L_SINGLE_LANE_CHANGE_ROI_TH)
            {
                L_SingleLM_SpecifyROI(SL_SCH_DEFAULT_ROI, &iTS->SL_LaneModel.LaneM, iTS);	//default ROI
            }
            else
            {
                L_SingleLM_SpecifyROI(SL_SCH_MAIN_ROI, &iTS->SL_LaneModel.LaneM, iTS);		//medium ROI
            }

//            if(iTS->L_SL_PointGroup.RightCounter >= L_SINGLE_LANE_CHANGE_ROI_TH)
//            {
//                iTS->SL_LaneModel.LaneM.LstCol = iTS->SL_LaneModel.LaneM.CurCol;
//                L_SingleLM_SpecifyROI(SL_SCH_MAIN_ROI, &iTS->SL_LaneModel.LaneM, iTS);	//medium ROI
//            }
//            else
//            {
//                L_SingleLM_SpecifyROI(SL_SCH_DEFAULT_ROI, &iTS->SL_LaneModel.LaneM, iTS);		//default ROI
//            }
        }


        if(iTS->L_SL_PointGroup.RightCounter >= 15)
            break;
	}

	iTS->SL_LaneModel.LastFinalRow_ = iTS->SL_LaneModel.CurFinalRow_;

    if(iTS->L_SL_PointGroup.UpdateCounter > L_SINGLE_LANE_EDGE_POINT_TH)
	{
		if(L_SingleLM_Estimation(iTS))	//road model estimation and coeficient update
        {
            return 1;	//single lane detection successfully
        }
		else
        {
            return 0;
        }
	}
	else
    {
        iTS->single_lane_debug = 1;
        return 0;	//lane marking detection failure or not found
    }
}

short L_SingleLM_Tracking_PreProcess(short Final_row, ITS *iTS)
{
	short row_;

	iTS->SL_LaneModel.U[2] /= L_PW_SL_PW_MODEL_RESERVE;
	for(row_ = 0;row_ < 5;row_++)
	{
		iTS->SL_LaneModel.dU[row_] /= L_PW_SL_PW_MODEL_RESERVE;
		if(row_ < 2)
		{
			iTS->SL_LaneModel.U[row_] /= L_PW_SL_PW_MODEL_RESERVE;
			iTS->SL_LaneModel.V[row_] /= L_PW_SL_PW_MODEL_RESERVE;
		}
	}
	iTS->SL_LaneModel.Um[0] /= L_PW_SL_PW_MODEL_RESERVE;

	Final_row = L_SINGLE_LANE_TRACE_TB;

	return Final_row;
}

short L_SingleLM_Tracking(ITS *iTS)
{
	short Final_row = 0;
	short row_;

	Final_row = L_SingleLM_Tracking_PreProcess(Final_row, iTS);

	SingleLane_Tracking_SpecifyROI(SL_TCK_MAIN_ROI, &iTS->SL_LaneModel.LaneM, iTS);

	for(row_ = L_SINGLE_LANE_TRACE_BB; row_ < Final_row; row_ ++)	//edge point finding
	{
		iTS->SL_LaneModel.Vrow = iTS->SL_LaneModel.evRodSlp - (row_ - iTS->F_H_C);	//road slope adjustment

		if(L_SingleLM_MarkSearch(row_,iTS))	//the mark is found: the main process row by row
		{
			if(iTS->L_SL_PointGroup.RightCounter >= L_SINGLE_LANE_CORRECT_POINT_TH)
			{
				iTS->SL_LaneModel.LaneM.LstCol = iTS->SL_LaneModel.LaneM.CurCol;
				SingleLane_Tracking_SpecifyROI(SL_TCK_SUB_ROI, &iTS->SL_LaneModel.LaneM, iTS);	//minimum ROI
				//update lane model
				L_SingleLM_BuildLaneModel(iTS);
				iTS->SL_LaneModel.CurFinalRow_ = row_;
			}
			else
			{
				iTS->SL_LaneModel.LaneM.LstCol = iTS->SL_LaneModel.LaneM.CurCol;
				SingleLane_Tracking_SpecifyROI(SL_TCK_MAIN_ROI, &iTS->SL_LaneModel.LaneM, iTS);	//medium ROI
			}
		}
		else								//mark not found
			SingleLane_Tracking_SpecifyROI(SL_TCK_MAIN_ROI, &iTS->SL_LaneModel.LaneM, iTS); //default ROI
	}
	iTS->SL_LaneModel.LastFinalRow_ = iTS->SL_LaneModel.CurFinalRow_;

	if(iTS->L_SL_PointGroup.UpdateCounter > L_SINGLE_LANE_EDGE_POINT_TH)
	{
		if(L_SingleLM_Estimation(iTS))	//road model estimation and coeficient update
        {
            return 1;	//single lane detection successfully
        }
        else
        {
            return 0;
        }
	}
	else
	{
        iTS->single_lane_debug = 7;
        return 0;	//lane marking detection failure or not found
	}
}

short L_SingleLMProcess(ITS *iTS)
{
        //searching mode
	if (iTS->L_DetectMode == SL_SEARCH)
	{
		L_RstSingleLM_Variables(iTS);
		if (!L_SingleLM_Searching(iTS))
		{
			iTS->L_DetectMode = SL_SEARCH;
            iTS->SL_LaneModel.L_StbCtr = LimitCount(--iTS->SL_LaneModel.L_StbCtr, L_SINGLE_LANE_MAX_STABLE);

#if(L_SINGLE_LANE_SEARCH_DIRECTION == L_SINGLE_LANE_SEARCH_MERGE_DIRECTION)
			//switch searching direction
			if(iTS->SingleLaneDirectionSwitch == 0)	//right
				iTS->SingleLaneDirectionSwitch = 1;
			if(iTS->SingleLaneDirectionSwitch == 1)	//left
				iTS->SingleLaneDirectionSwitch = 0;
#endif

			return iTS->L_DetectionResultReturn = L_SINGLE_FAIL;
		}
        iTS->SL_LaneModel.L_StbCtr = LimitCount(++iTS->SL_LaneModel.L_StbCtr, L_SINGLE_LANE_MAX_STABLE);
		iTS->L_DetectMode = SL_SEARCH;
        if(iTS->SL_LaneModel.L_StbCtr >= 5)
            iTS->L_DetectMode = SL_TRACE;

		return iTS->L_DetectionResultReturn = L_SINGLE_SUCCESS;

	}

	//tracking mode
	if (iTS->L_DetectMode == SL_TRACE)
	{
		L_RstSingleLM_Tracking_Variables(iTS);
		if (!L_SingleLM_Tracking(iTS))
		{
            iTS->SL_LaneModel.L_StbCtr = LimitCount(--iTS->SL_LaneModel.L_StbCtr, L_SINGLE_LANE_MAX_STABLE);
			if(iTS->SL_LaneModel.L_StbCtr < 10)
			{

#if(L_SINGLE_LANE_SEARCH_DIRECTION == L_SINGLE_LANE_SEARCH_MERGE_DIRECTION)
				//switch searching direction
				if(iTS->L_BiasWarn < 0)	//right
					iTS->SingleLaneDirectionSwitch = 1;
				if(iTS->L_BiasWarn > 0)	//left
					iTS->SingleLaneDirectionSwitch = 0;
#endif

				if(iTS->SL_LaneModel.L_StbCtr < 1)
				{
					iTS->L_DetectMode = SL_SEARCH;
					iTS->SL_LaneModel.L_StbCtr = 0;
					return iTS->L_DetectionResultReturn = L_SINGLE_FAIL;
				}
			}
			return iTS->L_DetectionResultReturn = L_SINGLE_SUCCESS;
		}
		iTS->L_DetectMode = SL_TRACE;
        iTS->SL_LaneModel.L_StbCtr = LimitCount(iTS->SL_LaneModel.L_StbCtr + 4, L_SINGLE_LANE_MAX_STABLE);
		return iTS->L_DetectionResultReturn = L_SINGLE_SUCCESS;
	}
	//redundancy
	return iTS->L_DetectionResultReturn = L_SINGLE_SUCCESS;
}

#ifdef L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN
void L_DecisionSingleDirection(ITS *iTS)
{
	if(iTS->L_DetectMode == SL_TRACE || iTS->L_DetectMode == LTRACE)
	{
		if((iTS->L_BiasWarn > 0 && iTS->L_Last_BiasWarn == 0) || iTS->SL_LaneModel.L_SL_LorR == SL_Left)
		{
			iTS->SingleLaneDirectionSwitch = 1;
		}
		else if((iTS->L_BiasWarn < 0 && iTS->L_Last_BiasWarn == 0) || iTS->SL_LaneModel.L_SL_LorR == SL_Right)
		{
			iTS->SingleLaneDirectionSwitch = 0;
		}
		else //if(iTS->L_BiasWarn != 0 || iTS->L_BiasWarn == 0)
		{
			iTS->SingleLaneDirectionSwitch = iTS->SingleLaneDirectionSwitch;
		}
		iTS->L_Last_BiasWarn = iTS->L_BiasWarn;
	}
}
#endif //L_DECISION_SINGLE_SEARCH_DIRECTION_FOR_BIASWARN
#endif

