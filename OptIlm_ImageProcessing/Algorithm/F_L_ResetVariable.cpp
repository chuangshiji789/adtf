#include "InitialVariable.h"
#include "FunctionType.h"



void L_RstDualLM_Variables(ITS *iTS)
{
	iTS->L_BiasWarn = 0;
	iTS->L_last_bias_flag = 0;
	iTS->L_Bias = 0;

	iTS->L_StbCtr = 0;
	iTS->L_Ctr_LossBothLane =0;
	iTS->L_Ctr_LossSingleLane = 0;
	iTS->L_CurFinalRow_ = S_IMGCH;			//=====FOR DEFAULT OBSTACLE DETECTION=====
	iTS->L_LastFinalRow_ = S_IMGCH;			//=====FOR DEFAULT OBSTACLE DETECTION=====

	//實虛線判斷參數
	iTS->SolidlineCountL = INITIAL_FRAME_COUNT;
	iTS->SolidlineCountR = INITIAL_FRAME_COUNT;
	iTS->SolidlineL = 1;
	iTS->SolidlineR = 1;
	iTS->SolidlineTH_R = INITIAL_THRESHOLD;
	iTS->SolidlineTH_L = INITIAL_THRESHOLD;

    //Road Parameters
	iTS->L_WCur = L_RW_DefaultLW;					//	road width
	iTS->L_WAvg = L_RW_DefaultLW;					//=====FOR DEFAULT OBSTACLE DETECTION=====
    iTS->L_evRodSlp = EV_ROAD_SLOPE;							//	euslope = S_ev * slope

    iTS->VanishingPoint_V = (short)((double)S_IMGH - (double)S_IMGCH - iTS->L_evRodSlp);

    iTS->k = iTS->default_k;//L_MP_kDefault;							//	curvature
    iTS->m = iTS->default_m; //L_MP_mDefault;							//	orientation
    iTS->bm = iTS->default_b;//L_MP_bDefault;						//	bias
    iTS->br = iTS->default_b + (L_RW_ROILW >> 1);		//	bias
    iTS->bl = iTS->default_b - (L_RW_ROILW >> 1);			//	bias

	iTS->K = iTS->k * S_eu * S_ev * S_HOCam;			//	K = k * S_eu * S_ev * H
	iTS->M = iTS->m * S_eu;								//	M = m * S_eu
	iTS->Bm = iTS->bm / S_HOCam * (S_euv);				//	Bm = (bm / H) * (S_eu / S_ev)
	iTS->LaneR.B_i = iTS->br / S_HOCam * (S_euv);		//	Br = (br / H) * (S_eu / S_ev)
	iTS->LaneL.B_i = iTS->bl / S_HOCam * (S_euv);		//	Bl = (bl / H) * (S_eu / S_ev)
	memset(iTS->L_UVXY, 0, 11*sizeof(double));

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
	iTS->L_LaneModelType = CURVE;
#endif

#if(LANE_MODEL_SELECTION == LANE_MODEL_IS_LINE)
	iTS->L_LaneModelType = LINE;
#endif

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	iTS->SL_SupersedeFlag = REGULAR_PROCESS;
	iTS->SupersedeCounter = 0;
#endif

//Bias ratio
#ifdef BIAS_RATIO_CALCULATION
	iTS->L_Bias = 0;
	iTS->L_Last_Bias = 0;
	iTS->L_Last_BiasRatio = 0;
#endif

	//iTS->L_EdgeMeanValue = 60;

    iTS->Stop_Line.mode = SEARCH;
    iTS->Stop_Line.stable_counter = 0;
}

void O_RstDualLM_Variables(ITS *iTS)
{
	iTS->CARM.StableCtr=0;
	iTS->CARM.RealDistance = 10000;
	iTS->CARR.StableCtr=0;
	iTS->CARL.StableCtr=0;
}

void RstDualLM_Variables(ITS *iTS)
{
	L_RstDualLM_Variables(iTS);
	//障礙物
	O_RstDualLM_Variables(iTS);
}

void L_SetEviromonet(ITS *iTS)
{
	iTS->F_W = S_IMGW;				//	width of image		//MUST BE SET!!!
	iTS->F_H = S_IMGH;				//	height of image		//MUST BE SET!!!
    iTS->F_W_C = (S_IMGW >> 1) + CAMERA_HORIZONTAL_SHIFT;		//	half width of image	//MUST BE SET!!!
	iTS->F_H_C = S_IMGCH;				//  Vanish Point		//MUST BE SET!!!
	iTS->LaneR.Sd = L_RtSd;			//MUST BE SET!!!
	iTS->LaneL.Sd = L_LtSd;
	iTS->Um= &iTS->L_UVXY[0];
	iTS->U=  &iTS->L_UVXY[1];
	iTS->dU= &iTS->L_UVXY[4];
	iTS->V=  &iTS->L_UVXY[9];

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	iTS->SL_LaneModel.LaneR.Sd = L_RtSd;			//MUST BE SET!!!
	iTS->SL_LaneModel.LaneL.Sd = L_LtSd;
	iTS->SL_LaneModel.Um= &iTS->SL_LaneModel.L_UVXY[0];
	iTS->SL_LaneModel.U=  &iTS->SL_LaneModel.L_UVXY[1];
	iTS->SL_LaneModel.dU= &iTS->SL_LaneModel.L_UVXY[4];
	iTS->SL_LaneModel.V=  &iTS->SL_LaneModel.L_UVXY[9];
#endif
}

void W_SetEviromonet(ITS *iTS)
{
	iTS->W_Lane=LaneNoWarnning;
	iTS->W_Obstacle=SafeDistance;
	iTS->CARL.CarSign=-1;
	iTS->CARR.CarSign=1;
	iTS->CARM.CarSign=0;
}


#define V_TAIL_EDGE_DETECTION_TOP_BOUNDARY		4500
#define V_SEARCHING_BOUNDARY					4500
short ResetConstant(ITS *iTS)
{
#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS)
	iTS->L_DetectMode = SL_SEARCH;
#else
	iTS->L_DetectMode = LSEARCH;
#endif

	memset(iTS->O_InfoPlane, 0, S_IMGW * S_IMGH * sizeof(char));
	memset(iTS->L_ColProjection, 0, S_IMGW * S_IMGH * sizeof(char));
   	memset(iTS->O_SD_VerPrjArray, 0, S_IMGW * sizeof(short));
	memset(iTS->O_VE_VP_Array, 0, S_IMGW * sizeof(short));
	memset(iTS->O_HD_Array, 0, S_IMGH * sizeof(short));
   	memset(iTS->O_CL_Array, 0, S_IMGH * sizeof(short));

    iTS->O_evRodSlp = EV_ROAD_SLOPE;
	//for switch single lane to dual lane detection
	iTS->L_DetectionResultReturn = L_DUAL_SUCCESS;	//default value

    iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean = LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7;
    iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean = LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7;

	L_SetEviromonet(iTS);
	O_SetPixelArea(iTS);
	W_SetEviromonet(iTS);

	iTS->L_DelayWarningCounter = 0;

	RstDualLM_Variables(iTS);

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	L_RstSingleLM_Variables(iTS);
#endif

#if(SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	iTS->SL_SupersedeFlag = REGULAR_PROCESS;
	iTS->SupersedeCounter = 0;
#endif

	//for grayscale remapping
#ifdef GRAYSCALE_REMAPPING_ON_OFF
	iTS->L_Grayscale_Remapping.NewMapping = (short *) calloc(256, sizeof(short));
	iTS->L_Grayscale_Remapping.Illumination_Mapping = (short *) calloc(GRAYSCALE_SCALING_REGION, sizeof(short));
	iTS->L_Grayscale_Remapping.LastCorrectMarkMeanValue = 0;
	iTS->L_Grayscale_Remapping.MarkMeanFailCounter = 0;
	CreateIlluminationTable(iTS);
#endif

	//for calculation curvature
#ifdef CURVATURE_CALCULATION_ON_OFF
	iTS->CurvatureStructure.Avg_Curvature = 0;
#endif

#ifdef BIAS_RATIO_CALCULATION
	iTS->L_Bias = 0;
	iTS->L_Last_Bias = 0;
	iTS->L_Last_BiasRatio = 0;

#ifdef FILE_OUTPUT_FOR_BIAS
	fprintf(stdout,"	Bias	Bias_Ratio\n");
#endif
#endif

	iTS->O_CarLightTH = 230;
	iTS->O_SD_ShadowTh = 120;

	iTS->OSD_Color.Y = 105;
	iTS->OSD_Color.U = 68;
	iTS->OSD_Color.V = 52;

	iTS->day_or_night_count = 30;
	iTS->day_or_night_flag = 1;

    iTS->sobel_V_TH = 60;
    iTS->sobel_H_TH = 60;
    iTS->laplacian_TH = 60;

#ifdef SPEED_OF_A_MOTOR_VEHICLE_ON_OFF
	iTS->System_Parameter.Velocity = VELOCITY_OFF;
#else
	iTS->System_Parameter.Velocity = HIGH_VELOCITY;
#endif

#ifdef INCREASE_CONTRAST_ON_OFF
	IncreaseContrast(iTS);
#endif //INCREASE_CONTRAST_ON_OFF

#ifdef L_PROTECTION_MAIN_ON_OFF
	iTS->L_Protection_Main = PROTECTION_OFF;
	iTS->L_UsingPerception_Main = USING_PERCEPTION_OFF;
#endif

	ArrayPreprocessing(iTS);
	CalculateImageCarWidth(iTS);

	iTS->CAR = &iTS->CARM;
	iTS->CAR->CarInfo.Position.BB = 0;
	iTS->V_LAST_BB = 0;
	iTS->V_LAST_BB_use_flag = 0;
	iTS->V_tail_edge_one_time_flag = 0;

	iTS->V_tail_edge_detection_top_boundary = (short)(iTS->F_H_C - ((S_HOCam*S_ev / V_TAIL_EDGE_DETECTION_TOP_BOUNDARY) - 0));
	iTS->V_tail_edge_found_row = 0;

	iTS->V_serching_boundary = (short)(iTS->F_H_C - ((S_HOCam*S_ev / V_SEARCHING_BOUNDARY) - 0));

	iTS->V_distance_number[0] = O_GetRowDistance(VEHICLE_WARNING_BOUND_1, iTS);
	iTS->V_distance_number[1] = O_GetRowDistance(VEHICLE_WARNING_BOUND_2, iTS);
	iTS->V_distance_number[2] = O_GetRowDistance(VEHICLE_WARNING_BOUND_3, iTS);

	//O_GetDistance(0, iTS);



#ifdef PARAMETER_EXTRACTION
	F_ResetParameterExtraction(iTS);
#endif

	F_L_CalculationWheelPositionOnScreen(iTS);

	iTS->L_last_bias_flag = 0;

	return 1;
}
#ifdef PARAMETER_EXTRACTION
void F_ResetParameterExtraction(ITS *iTS)
{
	iTS->LDWS_detection_sensitivity = F_DETECTION_MIDDLE;
	iTS->LDWS_warning_sensitivity = F_WARNING_MIDDLE;
	iTS->FCW_detection_sensitivity = F_DETECTION_MIDDLE;

	iTS->L_draw_single_lane_mark = 10;
	iTS->L_draw_dual_lane_mark = 10;
	iTS->L_warning_vehicle_width = VEHICLE_WIDTH;
	iTS->V_draw_vehicle_position = O_FC_GoingStable;
}

void F_SetParameterExtraction(ITS *iTS)
{
	if(iTS->LDWS_detection_sensitivity == F_DETECTION_LOW)
	{
		iTS->L_draw_single_lane_mark = 19;
		iTS->L_draw_dual_lane_mark = 14;
	}
	else if(iTS->LDWS_detection_sensitivity == F_DETECTION_MIDDLE)
	{
		iTS->L_draw_single_lane_mark = 10;
		iTS->L_draw_dual_lane_mark = 10;
	}
	else if(iTS->LDWS_detection_sensitivity == F_DETECTION_HIGH)
	{
		iTS->L_draw_single_lane_mark = 3;
		iTS->L_draw_dual_lane_mark = 3;
	}

	if(iTS->LDWS_warning_sensitivity == F_WARNING_LOW)
	{
		iTS->L_warning_vehicle_width = VEHICLE_WIDTH - 50;
	}
	else if(iTS->LDWS_warning_sensitivity == F_WARNING_MIDDLE)
	{
		iTS->L_warning_vehicle_width = VEHICLE_WIDTH;
	}
	else if(iTS->LDWS_warning_sensitivity == F_WARNING_HIGH)
	{
		iTS->L_warning_vehicle_width = VEHICLE_WIDTH + 50;
	}

	if(iTS->FCW_detection_sensitivity == F_DETECTION_LOW)
	{
		iTS->V_draw_vehicle_position = O_FC_GoingStable + 4;
	}
	else if(iTS->FCW_detection_sensitivity == F_DETECTION_MIDDLE)
	{
		iTS->V_draw_vehicle_position = O_FC_GoingStable;
	}
	else if(iTS->FCW_detection_sensitivity == F_DETECTION_HIGH)
	{
		iTS->V_draw_vehicle_position = O_FC_GoingStable - 4;
	}


}
#endif


void SetITSBuffer(ITS *iTS)
{
    iTS->O_InfoPlane = (unsigned char *) calloc(S_IMGW*S_IMGH, sizeof(unsigned char));
    iTS->L_ColProjection = (unsigned char *) calloc(S_IMGW*S_IMGH, sizeof(unsigned char));
    iTS->O_MarkInfoPlane = (unsigned char *) calloc(S_IMGW*S_IMGH, sizeof(unsigned char));
    iTS->O_SD_VerPrjArray = (short *) calloc(S_IMGW, sizeof(short));
    iTS->O_VE_VP_Array = (short *) calloc(S_IMGW, sizeof(short));
    iTS->O_HD_Array = (short *) calloc(S_IMGH, sizeof(short));
    iTS->O_CL_Array = (short *) calloc(S_IMGH, sizeof(short));
    iTS->PixelArea = (unsigned short *) calloc(S_IMGH, sizeof(unsigned short));
    iTS->L_LaneMBound = (FORWARD_CROI *) calloc(S_IMGH, sizeof(FORWARD_CROI));
    iTS->O_LaneMBound = (FORWARD_CROI *) calloc(S_IMGH, sizeof(FORWARD_CROI));
    iTS->L_LaneLBound = (FORWARD_CROI *) calloc(S_IMGH, sizeof(FORWARD_CROI));
    iTS->L_LaneRBound = (FORWARD_CROI *) calloc(S_IMGH, sizeof(FORWARD_CROI));
    //iTS->Hor_mark = (HORIZONTAL_MARK *) calloc(O_Max_H_Mark_counter, sizeof(HORIZONTAL_MARK));
    iTS->O_P_InfoPlane = (unsigned char *) calloc(S_IMGW*S_IMGH, sizeof(unsigned char));
    iTS->Gxy_InfoPlane = (unsigned char *) calloc(S_IMGW*S_IMGH, sizeof(unsigned char));
    iTS->Axy_InfoPlane = (unsigned char *) calloc(S_IMGW*S_IMGH, sizeof(unsigned char));
    iTS->Hog_InfoPlane = (unsigned char *) calloc(S_IMGW*S_IMGH, sizeof(unsigned char));

}

void FreeMemory(ITS *iTS)
{
    if(iTS->Hog_InfoPlane != NULL) free(iTS->Hog_InfoPlane);
    if(iTS->Gxy_InfoPlane != NULL) free(iTS->Gxy_InfoPlane);
    if(iTS->Axy_InfoPlane != NULL) free(iTS->Axy_InfoPlane);
    if(iTS->O_P_InfoPlane != NULL) free(iTS->O_P_InfoPlane);
    //if(iTS->Hor_mark != NULL) free(iTS->Hor_mark);
    if(iTS->L_LaneRBound != NULL) free(iTS->L_LaneRBound);
    if(iTS->L_LaneLBound != NULL) free(iTS->L_LaneLBound);
    if(iTS->O_LaneMBound != NULL) free(iTS->O_LaneMBound);
    if(iTS->L_LaneMBound != NULL) free(iTS->L_LaneMBound);
    if(iTS->PixelArea != NULL) free(iTS->PixelArea);
    if(iTS->O_CL_Array != NULL) free(iTS->O_CL_Array);
    if(iTS->O_HD_Array != NULL) free(iTS->O_HD_Array);
    if(iTS->O_VE_VP_Array != NULL) free(iTS->O_VE_VP_Array);
    if(iTS->O_SD_VerPrjArray != NULL) free(iTS->O_SD_VerPrjArray);
    if(iTS->O_MarkInfoPlane != NULL) free(iTS->O_MarkInfoPlane);
    if(iTS->L_ColProjection != NULL) free(iTS->L_ColProjection);
    if(iTS->O_InfoPlane != NULL) free(iTS->O_InfoPlane);
}


#define IMAGE_CENTER_SHIFTING							CAMERA_HORIZONTAL_SHIFT	//單位 "pixel"
#define CORRECT_SHIFFT_VALUE							0						//單位 "cm"
#define WHEEL_POSITION_ON_WORLD_COORDINATE_SYSTEM		BIAS_FORWARD_EXTEND		//單位 "cm"
void F_L_CalculationWheelPositionOnScreen(ITS *iTS)
{
	//半車寬
	short half_vehicle_whidth = (short) (VEHICLE_WIDTH / 2);
    float temp = EV_ROAD_SLOPE - 22.8;//S_HOCam;

	temp = (temp / WHEEL_POSITION_ON_WORLD_COORDINATE_SYSTEM) * S_ev;
	if(temp < 0)
		temp = -temp;

	//延伸車寬，車輪所在列 (row number)
	//iTS->L_wheel_position_row = (short)(S_IMGCH - ((S_HOCam * S_ev / WHEEL_POSITION_ON_WORLD_COORDINATE_SYSTEM) - 0));
	iTS->L_wheel_position_row = (short)(S_IMGCH - (short)temp);

	//車中心 (column number)
	iTS->L_bias_vehicle_center_axis_position = S_IMGCW + IMAGE_CENTER_SHIFTING;

	//左車輪 (column number)
	iTS->L_bias_vehicle_left_wheel_position = (short) iTS->L_bias_vehicle_center_axis_position -
		(short) DWtoDI(half_vehicle_whidth, S_IMGCH - iTS->L_wheel_position_row);

	//右車輪 (column number)
	iTS->L_bias_vehicle_right_wheel_position = (short) iTS->L_bias_vehicle_center_axis_position +
		(short) DWtoDI(half_vehicle_whidth, S_IMGCH - iTS->L_wheel_position_row);

	//左車輪回復值 (column number + shift value)
	iTS->L_Bias_vehicle_left_wheel_position_correct_return_value = (short) iTS->L_bias_vehicle_center_axis_position -
		(short) DWtoDI(half_vehicle_whidth + CORRECT_SHIFFT_VALUE, S_IMGCH - iTS->L_wheel_position_row);

	//右車輪回復值 (column number + shift value)
	iTS->L_Bias_vehicle_right_wheel_position_correct_return_value = (short) iTS->L_bias_vehicle_center_axis_position +
		(short) DWtoDI(half_vehicle_whidth + CORRECT_SHIFFT_VALUE, S_IMGCH - iTS->L_wheel_position_row);
}

