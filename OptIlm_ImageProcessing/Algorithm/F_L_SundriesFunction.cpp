#include "InitialVariable.h"
#include "FunctionType.h"

#ifdef CAMERA_SETUP_FUNCTION_ON_OFF
void CameraSetUp(ITS *iTS)
{
#ifdef CAM_SETUP_USE_LANE
	short UB = L_IB_TB_DrawLane;		//Upper boundary
	short LB = O_IB_BB_SearchVehicle;	//Lower boundary
	float x1, x2, x3, x4, x5, x6;
	float y1, y2, y3, y4, y5, y6;
	float Slop_1, Slop_2;				//Line slop

	int AL, AR;
	int BL, BR;
	int CL, CR;
	int Skyline;

	while(iTS->L_LaneMBound[LB].Frt <= S_IMGLB)
		LB += 1;
	while(iTS->L_LaneMBound[LB].Scd >= S_IMGRB)
		LB += 1;

	x1 = iTS->L_LaneMBound[UB].Frt; //Top-Left
	x2 = iTS->L_LaneMBound[LB].Frt;	//Bottom-Left
	x3 = iTS->L_LaneMBound[UB].Scd; //Top-Right
	x4 = iTS->L_LaneMBound[LB].Scd; //Bottom-Right
	y1 = UB;
	y2 = LB;
	y3 = UB;
	y4 = LB;

	AL = (int)(y1 - y2);
	AR = (int)(y3 - y4);
	BL = (int)(x2 - x1);
	BR = (int)(x4 - x3);
	CL = (int)(x1 * y2 - x2 * y1);
	CR = (int)(x3 * y4 - x4 * y1);

	//Vanishing point calculation
	Skyline = AL * CR - AR * CL;
	if((AR * BL - AL * BR) == 0)
		Skyline = 0;
	else
		Skyline /= (AR * BL - AL * BR);

	y5 = (float)Skyline;
	y6 = (float)Skyline;

	if((x2 - x1) == 0)
		Slop_1 = 0;
	else
		Slop_1 = (y2 - y1) / (x2 - x1); //left line slop
	if((x4 - x3) == 0)
		Slop_2 = 0;
	else
		Slop_2 = (y4 - y3) / (x4 - x3); //right line slop

	//Line extension
	if(Slop_1 == 0)
		x5 = x2;
	else
        x5 = fabs(((y5 - y2) / Slop_1) + x2);
	if(Slop_2 == 0)
		x6 = x4;
	else
        x6 = fabs(((y6 - y4) / Slop_2) + x4);

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	if((iTS->L_StbCtr > 10 || iTS->SL_LaneModel.L_StbCtr > 10) && (y5 < S_IMGTB - 20) && (y6 < S_IMGTB - 20) && (y5 > S_IMGBB + 20) && (y6 > S_IMGBB + 20))
#else
	if(iTS->L_StbCtr > 10 && (x5 < S_IMGTB - 20) && (y5 < S_IMGTB - 20) && (y6 < S_IMGTB - 20) && (y5 > S_IMGBB + 20) && (y6 > S_IMGBB + 20))
#endif
	{
		iTS->Skyline_V = S_IMGH - Skyline;
		iTS->CenterPoint_U = (short)((x5 + x6) / 2);
		DrawRect((short)(S_IMGH - y1), (short)x1, 10, 255, iTS);
		DrawRect((short)(S_IMGH - y2), (short)x2, 10, 255, iTS);
		DrawRect((short)(S_IMGH - y3), (short)x3, 10, 255, iTS);
		DrawRect((short)(S_IMGH - y4), (short)x4, 10, 255, iTS);
	   	DrawRect((short)(S_IMGH - y5 - 10), (short)x5-10, 20, 255, iTS);
		DrawRect((short)(S_IMGH - y6 - 10), (short)x6-10, 20, 255, iTS);
		memset(&iTS->Showimage[iTS->F_W * iTS->Skyline_V], 255, iTS->F_W);
		OSD_Color_Setup(OCN_GREEN, iTS);
		ScalableNumber((short) Skyline, 4, S_IMGW >> 1, 170, iTS);
	}
#endif

#ifdef CAM_SETUP_USE_BALL
	// For new camera setup function
	DrawRect(197, 129, 10, 255, iTS);
	DrawRect(207, 129, 10, 255, iTS);
	DrawRect(217, 129, 10, 255, iTS);
	DrawRect(227, 129, 10, 255, iTS);

	DrawRect(197, 144, 10, 255, iTS);

	DrawRect(197, 160, 10, 255, iTS);
	DrawRect(207, 160, 10, 255, iTS);
	DrawRect(217, 160, 10, 255, iTS);
	DrawRect(227, 160, 10, 255, iTS);

	DrawRect(227, 144, 10, 255, iTS);

	DrawRect(212, 172, 10, 255, iTS);
	DrawRect(212, 182, 10, 255, iTS);
	DrawRect(212, 192, 10, 255, iTS);

	DrawRect(197, 205, 10, 255, iTS);
	DrawRect(207, 205, 10, 255, iTS);
	DrawRect(217, 205, 10, 255, iTS);
	DrawRect(227, 205, 10, 255, iTS);

	DrawRect(197, 220, 10, 255, iTS);

	DrawRect(197, 236, 10, 255, iTS);
	DrawRect(207, 236, 10, 255, iTS);
	DrawRect(217, 236, 10, 255, iTS);
	DrawRect(227, 236, 10, 255, iTS);

	DrawRect(227, 220, 10, 255, iTS);
#endif
}
#endif
void DrawRect(short RectStartRow, short RectStartCol, short DrawSize, unsigned char ColorNum, ITS *iTS)
{
	short row_;
	for (row_ = RectStartRow; row_ < RectStartRow + DrawSize; row_ ++)
		memset((iTS->Showimage + iTS->F_W * row_ + RectStartCol), ColorNum, DrawSize);
}


#ifdef GRAYSCALE_REMAPPING_ON_OFF
void CreateIlluminationTable(ITS *iTS)
{
	short OneScale = (short) ((double) 255 / GRAYSCALE_SCALING_REGION);
	short i;

	for(i = 0; i < GRAYSCALE_SCALING_REGION; i ++)
		iTS->L_Grayscale_Remapping.Illumination_Mapping[i] = i * OneScale;
}

void GrayScaleRemapping(ITS *iTS)
{
	int GrayMeanResult = 0;
	short BottomBound = 0;
	short TopBound = 0;
	short row_ = 0;

	//mean value
	GrayMeanResult = ((short) (iTS->L_Grayscale_Remapping.LastCorrectMarkMeanValue * GRAYSCALE_MEAN_BUFFER) + iTS->L_Grayscale_Remapping.LastRoadMeanValue) >> GRAYSCALE_DIVISOR;

	//boundaries limitation
	if(GrayMeanResult < GRAYSCALE_REMAPPING_MINIMUM)
		GrayMeanResult = GRAYSCALE_REMAPPING_MINIMUM;
	if(GrayMeanResult > GRAYSCALE_REMAPPING_MAXIMUM)
		GrayMeanResult = GRAYSCALE_REMAPPING_MAXIMUM;

	//record last correct value
	iTS->L_Grayscale_Remapping.LastCorrectMarkMeanValue = GrayMeanResult;

	//boundary definition
	TopBound = GrayMeanResult + GRAYSCALE_SCALING_HALF_REGION;
	BottomBound = TopBound - GRAYSCALE_SCALING_REGION;

	//elements shifting
	for(row_= GRAYSCALE_REMAPPING_MINIMUM; row_ <= 255; row_++)
	{
		if(row_ >=  TopBound)
			iTS->L_Grayscale_Remapping.NewMapping[row_] = 255;
		else if(row_ >=  BottomBound && row_ < TopBound)
		{
			iTS->L_Grayscale_Remapping.NewMapping[row_] = iTS->L_Grayscale_Remapping.Illumination_Mapping[row_ - BottomBound];
		}
	}
}
#endif

#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF
void LM_GrayScale_Adaptive_Thresholding_Initial(LM_GRAY_ADP_TH *TargetStructure)
{
	TargetStructure->MarkMean_Counter = 0;
	TargetStructure->RoadMean_Counter = 0;

	if(TargetStructure->RoadMean < 255 && TargetStructure->RoadMean != 0)
		TargetStructure->LastRoadMean = (unsigned short)TargetStructure->RoadMean;

    TargetStructure->MarkMean = 150;
    TargetStructure->RoadMean = 50;

	TargetStructure->Threshold = 0;
}

void LM_GrayScale_Adaptive_Thresholding_PreProcess(LM_GRAY_ADP_TH *TargetStructure, short LBound, short RBound, UBYTE *pSrc, ITS *iTS)
{
	short i;
	short AreaWidth = abs(RBound - LBound);

	if ((LBound - AreaWidth) <= S_IMGLB || (RBound + AreaWidth) >= S_IMGRB)
		return;

	for(i = LBound - AreaWidth; i < RBound + AreaWidth; i ++)
	{
		if(i >= LBound && i <= RBound)	//lane marking area
		{
			TargetStructure->MarkMean += pSrc[i];
			TargetStructure->MarkMean_Counter ++;

#ifdef DRAW_LANE_MARKING_AND_ROAD_SURFACE_AREA
            //iTS->Showimage[TargetStructure->ScanRow + i] = 255;
#endif
		}

		if(i <= LBound || i >= RBound)	//road surface area
		{
			TargetStructure->RoadMean += pSrc[i];
			TargetStructure->RoadMean_Counter ++;

#ifdef DRAW_LANE_MARKING_AND_ROAD_SURFACE_AREA
			iTS->Showimage[TargetStructure->ScanRow + i] = 0;
#endif

		}
	}
}

char LM_GrayScale_Adaptive_Thresholding_Calculation(LM_GRAY_ADP_TH *TargetStructure, ITS *iTS)
{
	short Temp_GrayTH = 0;
	short Temp_TH_Minimum_Day_or_Night = 0;

	if(TargetStructure->MarkMean_Counter == 0)
		TargetStructure->MarkMean = 0;
	else
		TargetStructure->MarkMean = (unsigned long) ((double) TargetStructure->MarkMean / TargetStructure->MarkMean_Counter);

	if(TargetStructure->RoadMean_Counter == 0)
		TargetStructure->RoadMean = 0;
	else
		TargetStructure->RoadMean = (unsigned long) ((double) TargetStructure->RoadMean / TargetStructure->RoadMean_Counter);

	if(iTS->L_Cur_Left_Or_Right == 0)
	{
        iTS->L_LeftLane_Difference_Value = (short)fabs((float)(TargetStructure->MarkMean - TargetStructure->RoadMean));

#ifdef SHOW_LANE_ROAD_DIFFERENT_VALUE
		OSD_Color_Setup(OCN_BLUE, iTS);
		ScalableNumber(iTS->L_LeftLane_Difference_Value, 4, (S_IMGW >> 1) - 110, 100, iTS);
#endif    //SHOW_LANE_ROAD_DIFFERENT_VALUE

	}
	else //iTS->L_Cur_Left_Or_Right == 1
	{
        iTS->L_RightLane_Difference_Value =(short)fabs((float)(TargetStructure->MarkMean - TargetStructure->RoadMean));

#ifdef SHOW_LANE_ROAD_DIFFERENT_VALUE
		OSD_Color_Setup(OCN_BLUE, iTS);
		ScalableNumber(iTS->L_RightLane_Difference_Value, 4, (S_IMGW >> 1) + 90, 100, iTS);
#endif    //SHOW_LANE_ROAD_DIFFERENT_VALUE

	}

	//limitation
	if(TargetStructure->MarkMean >= LM_GRAY_ADP_TH_LONG_MAXIMUM || TargetStructure->RoadMean >= LM_GRAY_ADP_TH_LONG_MAXIMUM ||
	   TargetStructure->MarkMean == 0                           || TargetStructure->RoadMean == 0)
	{

#ifdef GRAYSCALE_REMAPPING_ON_OFF
		iTS->L_Grayscale_Remapping.LastRoadMeanValue = GRAYSCALE_DEFAULT_MARK_MEAN;
#endif //GRAYSCALE_REMAPPING_ON_OFF

		if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_0)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_0;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_0;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_1)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_1;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_1;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_2)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_2;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_2;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_3)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_3;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_3;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_4)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_4;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_4;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_5)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_5;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_5;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_6)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_6;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_6;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_7;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_7;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_8)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_8;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_8;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_9)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_9;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_9;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_10)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_10;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_10;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_11)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_11;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_11;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_12;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_12;
		}
		else if(TargetStructure->LastRoadMean > LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_13;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_13;
		}

		return 0;
	}

#ifdef GRAYSCALE_REMAPPING_ON_OFF
	iTS->L_Grayscale_Remapping.LastRoadMeanValue = (short) TargetStructure->MarkMean;
#endif

	Temp_GrayTH = (short) (TargetStructure->MarkMean - TargetStructure->RoadMean);
	Temp_GrayTH -= (short) (Temp_GrayTH >> LM_GRAY_ADP_GRAY_TH_DIVISOR);

	if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_0)
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_0;
	else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_1)
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_1;
	else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_2)
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_2;
	else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_3)
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_3;
	else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_4)
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_4;
	else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_5)
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_5;
	else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_6)
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_6;
	else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7)
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_7;
	else
		Temp_TH_Minimum_Day_or_Night = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_8;

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	if(/*(iTS->L_StbCtr >= 10 || iTS->SL_LaneModel.L_StbCtr >= 10) && */Temp_GrayTH >= Temp_TH_Minimum_Day_or_Night &&
	   ((iTS->L_DetectMode == SL_TRACE && iTS->L_SL_PointGroup.UpdateCounter >= LM_GRAY_ADP_LANE_POINT_TH) ||
	    (iTS->L_DetectMode == LTRACE && iTS->LaneL.PixCtr >= LM_GRAY_ADP_LANE_POINT_TH && iTS->LaneR.PixCtr >= LM_GRAY_ADP_LANE_POINT_TH)))
#else
	if(iTS->L_StbCtr >= 10 && Temp_GrayTH >= Temp_TH_Minimum_Day_or_Night && iTS->LaneL.PixCtr >= LM_GRAY_ADP_LANE_POINT_TH && iTS->LaneR.PixCtr >= LM_GRAY_ADP_LANE_POINT_TH)
#endif

	{
		TargetStructure->GrayDiffernce_Value = Temp_GrayTH;

		TargetStructure->GrayThreshold =
			(short) ((TargetStructure->GrayThreshold * LM_GRAY_ADP_TH_BUFFER_SIZE + TargetStructure->GrayDiffernce_Value) >> LM_GRAY_ADP_GRAY_TH_MEAN_DIVISOR);

		TargetStructure->MeanThreshold =
			(short) (TargetStructure->GrayThreshold - (short) (TargetStructure->GrayThreshold >> LM_GRAY_ADP_MEAN_TH_DIVISOR));
	}
	else
	{
		TargetStructure->GrayDiffernce_Value = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_7;

		if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_0)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_0;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_0;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_1)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_1;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_1;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_2)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_2;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_2;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_3)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_3;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_3;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_4)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_4;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_4;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_5)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_5;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_5;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_6)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_6;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_6;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_7;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_7;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_8)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_8;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_8;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_9)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_9;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_9;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_10)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_10;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_10;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_11)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_11;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_11;
		}
		else if(TargetStructure->LastRoadMean <= LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_12;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_12;
		}
		else if(TargetStructure->LastRoadMean > LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_12)
		{
			TargetStructure->GrayThreshold = LM_GRAY_ADP_GRAY_TH_MINIMUM_LEVEL_13;
			TargetStructure->MeanThreshold  = LM_GRAY_ADP_MEAN_TH_MINIMUM_LEVEL_13;
		}
	}

	return 1;
}
#endif

#ifdef CURVATURE_CALCULATION_ON_OFF
void Radius_of_Curvature(int Y_AxisPosition, ITS *iTS)
{
	double _2k = 2.0 * iTS->k;
	double Square_k = iTS->k * iTS->k;
	double Numerator = 0;
	double TmpValue = 0;
	double Denominator = 0;

	Numerator = (double) fabs(_2k);
	TmpValue = (double) (Y_AxisPosition + (iTS->m / _2k));
	Denominator = (double) sqrt(1.0 + (4.0 * Square_k * (TmpValue * TmpValue)));
	Denominator = (double) Denominator * Denominator * Denominator;
	iTS->CurvatureStructure.RadiusOfCurvature = (double) 1.0 / ((double)Numerator / Denominator);

	if(iTS->CurvatureStructure.Avg_Curvature != 0)
	{
		iTS->CurvatureStructure.Avg_Curvature =
		(int) ((iTS->CurvatureStructure.Avg_Curvature * CURVATURE_CALCULATION_BUFFER_SIZE) + iTS->CurvatureStructure.RadiusOfCurvature)
		>> CURVATURE_CALCULATION_DIVISOR;
	}
	else
		iTS->CurvatureStructure.Avg_Curvature = (int) iTS->CurvatureStructure.RadiusOfCurvature;

	if(iTS->CurvatureStructure.Avg_Curvature <= 1000) //(1000cm)
		iTS->CurvatureStructure.Avg_Curvature = CURVATURE_CALCULATION_SHOW_NUMBER_LIMIT;
}
#endif

//實虛線偵測
void SolidOrDashedLine(ITS *iTS)
{

#ifdef SOLID_LINE_DYNAMIC_TH_ON_OFF
//	static int i = 0, count = 0, Index_R = 0, Index_L = 0;
//	static int Solid_dataR[16] = {INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA,
//								  INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA,
//								  INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA,
//								  INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA};
//	static int Dashed_dataR[16] = {INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA,
//								   INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA,
//								   INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA,
//								   INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA};
//	static int Solid_dataL[16] = {INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA,
//								  INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA,
//								  INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA,
//								  INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA, INITIAL_SOLID_DATA};
//	static int Dashed_dataL[16] = {INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA,
//								   INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA,
//								   INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA,
//								   INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA, INITIAL_DASHED_DATA};
//	static int SolidlineTH_DataR[4] = {INITIAL_THRESHOLD, INITIAL_THRESHOLD, INITIAL_THRESHOLD, INITIAL_THRESHOLD};
//	static int SolidlineTH_DataL[4] = {INITIAL_THRESHOLD, INITIAL_THRESHOLD, INITIAL_THRESHOLD, INITIAL_THRESHOLD};
//	int Solid_MeanL = 0, Dashed_MeanL = 0, Solid_MeanR = 0, Dashed_MeanR = 0;
//	int j;
#endif

	//車道線在非常穩定的狀況下才會啟動實虛線判斷流程
	if(iTS->L_StbCtr == 15){

#ifdef SOLID_LINE_DYNAMIC_TH_ON_OFF

//		if(iTS->SolidlineL == 1)
//		{
//			Solid_dataL[Index_L] = iTS->LaneL.PixCtr;

//			//虛線填入上一次的數值
//			if(Index_L == 0)
//			{
//				Dashed_dataL[Index_L] = Dashed_dataL[DATA_AVERAGE - 1];
//			}
//			else
//			{
//				Dashed_dataL[Index_L] = Dashed_dataL[Index_L-1];
//			}

//			Index_L++;
//			if(Index_L == DATA_AVERAGE)
//			{
//				Index_L = 0;
//			}
//		}
//		else
//		{
//			Dashed_dataL[Index_L] = iTS->LaneL.PixCtr;

//			//實線填入上一次的數值
//			if(Index_L==0)
//			{
//				Solid_dataL[Index_L] = Solid_dataL[DATA_AVERAGE - 1];
//			}
//			else
//			{
//				Solid_dataL[Index_L] = Solid_dataL[Index_L-1];
//			}

//			Index_L++;
//			if(Index_L == DATA_AVERAGE)
//			{
//				Index_L = 0;
//			}

//		}
//		if(iTS->SolidlineR == 1)
//		{
//			Solid_dataR[Index_R] = iTS->LaneR.PixCtr;

//			//虛線填入上一次的數值
//			if(Index_R == 0)
//			{
//				Dashed_dataR[Index_R] = Dashed_dataR[DATA_AVERAGE - 1];
//			}
//			else
//			{
//				Dashed_dataR[Index_R] = Dashed_dataR[Index_R-1];
//			}

//			Index_R++;
//			if(Index_R == DATA_AVERAGE)
//			{
//				Index_R = 0;
//			}

//		}
//		else{
//			Dashed_dataR[Index_R] = iTS->LaneR.PixCtr;

//			//實線填入上一次的數值
//			if(Index_R == 0)
//			{
//				Solid_dataR[Index_R] = Solid_dataR[DATA_AVERAGE - 1];
//			}
//			else
//			{
//				Solid_dataR[Index_R] = Solid_dataR[Index_R-1];
//			}

//			Index_R++;
//			if(Index_R == DATA_AVERAGE)
//			{
//				Index_R = 0;
//			}

//		}
//		if(i == DATA_AVERAGE){
//			for(j = 0; j < DATA_AVERAGE; j++)
//			{
//				Solid_MeanR += Solid_dataR[j];
//				Dashed_MeanR += Dashed_dataR[j];
//				Solid_MeanL += Solid_dataL[j];
//				Dashed_MeanL += Dashed_dataL[j];
//			}

//			Solid_MeanR = Solid_MeanR >> DATA_AVERAGE_SHIFT;
//			Dashed_MeanR = Dashed_MeanR >> DATA_AVERAGE_SHIFT;
//			Solid_MeanL = Solid_MeanL >> DATA_AVERAGE_SHIFT;
//			Dashed_MeanL = Dashed_MeanL >> DATA_AVERAGE_SHIFT;

//			SolidlineTH_DataR[count] = (Solid_MeanR + Dashed_MeanR) >> 1;
//			SolidlineTH_DataL[count] = (Solid_MeanL + Dashed_MeanL) >> 1;
//			count++;

//			if(count == THRESHOLD_DATA_AVERAGE)
//			{
//				count = 0;
//				for(j = 0; j < THRESHOLD_DATA_AVERAGE; j++)
//				{
//					iTS->SolidlineTH_R += SolidlineTH_DataR[j];
//					iTS->SolidlineTH_L += SolidlineTH_DataL[j];
//				}
//				iTS->SolidlineTH_R = iTS->SolidlineTH_R >> THRESHOLD_DATA_AVERAGE_SHIFT;
//				iTS->SolidlineTH_L = iTS->SolidlineTH_L >> THRESHOLD_DATA_AVERAGE_SHIFT;
//				if(iTS->SolidlineTH_L > SOLID_LINE_DYNAMIC_HIGHEST_LIMIT)
//				{
//					iTS->SolidlineTH_L = SOLID_LINE_DYNAMIC_HIGHEST_LIMIT;
//				}
//				else if(iTS->SolidlineTH_L < SOLID_LINE_DYNAMIC_LOWEST_LIMIT)
//				{
//					iTS->SolidlineTH_L = SOLID_LINE_DYNAMIC_LOWEST_LIMIT;
//				}

//				if(iTS->SolidlineTH_R > SOLID_LINE_DYNAMIC_HIGHEST_LIMIT)
//				{
//					iTS->SolidlineTH_R = SOLID_LINE_DYNAMIC_HIGHEST_LIMIT;
//				}
//				else if(iTS->SolidlineTH_R < SOLID_LINE_DYNAMIC_LOWEST_LIMIT)
//				{
//					iTS->SolidlineTH_R = SOLID_LINE_DYNAMIC_LOWEST_LIMIT;
//				}
//			}
// 			i = 0;
//		}
//		i++;
#endif

		//實線與虛線Counter
//		if(iTS->LaneL.PixCtr > iTS->SolidlineTH_L)
//		{
//			iTS->SolidlineCountL = LimitCount(++iTS->SolidlineCountL, FRAME_COUNT_LIMIT);
//		}
//		else if(iTS->LaneL.PixCtr < iTS->SolidlineTH_L)
//		{
//			iTS->SolidlineCountL = LimitCount(--iTS->SolidlineCountL, FRAME_COUNT_LIMIT);
//		}
//		if(iTS->LaneR.PixCtr > iTS->SolidlineTH_R)
//		{
//			iTS->SolidlineCountR = LimitCount(++iTS->SolidlineCountR, FRAME_COUNT_LIMIT);
//		}
//		else if(iTS->LaneR.PixCtr < iTS->SolidlineTH_R)
//		{
//			iTS->SolidlineCountR = LimitCount(--iTS->SolidlineCountR, FRAME_COUNT_LIMIT);
//		}


        if(iTS->LaneL.mark_width > LANE_MARK_WIDTH_TH)
        {
            iTS->SolidlineCountL = LimitCount(++iTS->SolidlineCountL, FRAME_COUNT_LIMIT);
        }
        else
        {
            iTS->SolidlineCountL = LimitCount(--iTS->SolidlineCountL, FRAME_COUNT_LIMIT);
        }
        if(iTS->LaneR.mark_width > LANE_MARK_WIDTH_TH)
        {
            iTS->SolidlineCountR = LimitCount(++iTS->SolidlineCountR, FRAME_COUNT_LIMIT);
        }
        else
        {
            iTS->SolidlineCountR = LimitCount(--iTS->SolidlineCountR, FRAME_COUNT_LIMIT);
        }

	}

	//決定實線或虛線
	if(iTS->SolidlineCountL > DASHED_CHANGE_SOLID_COUNT)
	{
		iTS->SolidlineL = 1;
	}
	else if(iTS->SolidlineCountL < SOLID_CHANGE_DASHED_COUNT)
	{
		if(iTS->SolidlineL == 1)
		{
			iTS->SolidlineTH_L = INITIAL_THRESHOLD;
			iTS->SolidlineL = 0;
		}
		iTS->SolidlineL = 0;
	}
	if(iTS->SolidlineCountR > DASHED_CHANGE_SOLID_COUNT)
	{
		iTS->SolidlineR = 1;
	}
	else if(iTS->SolidlineCountR < SOLID_CHANGE_DASHED_COUNT)
	{
		if(iTS->SolidlineR == 1)
		{
			iTS->SolidlineTH_R = INITIAL_THRESHOLD;
			iTS->SolidlineR = 0;
		}
		iTS->SolidlineR = 0;
	}

//    OSD_Color_Setup(OCN_RED, iTS);
//    ScalableNumber(int(iTS->LaneL.mark_width * 10), 4,iTS->F_W_C - 100, 125, iTS);
//    ScalableNumber(int(iTS->LaneR.mark_width * 10), 4,iTS->F_W_C + 100, 125, iTS);



    if(iTS->L_DetectMode == SL_TRACE)
    {
        if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
            iTS->SolidlineR = 1;
        else if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
            iTS->SolidlineL = 0;
    }


#ifdef SOLID_OR_DASHED_LINE_DEBUG
	OSD_Color_Setup(OCN_RED, iTS);
	ScalableNumber(iTS->LaneL.PixCtr, 4,iTS->F_W_C - 60, 125, iTS);
	ScalableNumber(iTS->LaneR.PixCtr, 4,iTS->F_W_C + 60, 125, iTS);
	ScalableNumber(iTS->SolidlineCountL, 4,iTS->F_W_C - 100, 125, iTS);
	ScalableNumber(iTS->SolidlineCountR, 4,iTS->F_W_C + 100, 125, iTS);
	ScalableNumber(iTS->SolidlineTH_L, 4,iTS->F_W_C - 60, 50, iTS);
	ScalableNumber(iTS->SolidlineTH_R, 4,iTS->F_W_C + 60, 50, iTS);
#endif

}

#ifdef SPEED_OF_A_MOTOR_VEHICLE_ON_OFF
void VelocityMode_Control(short VelocityIn, ITS *iTS)
{
	if(VelocityIn < LOW_VELOCITY_TH)
		iTS->System_Parameter.Velocity = VELOCITY_OFF;
	else if(VelocityIn >= LOW_VELOCITY_TH && VelocityIn < HIGH_VELOCITY_TH)
		iTS->System_Parameter.Velocity = LOW_VELOCITY;
	else
		iTS->System_Parameter.Velocity = HIGH_VELOCITY;
}
#endif

#ifdef GRAY_LEVEL_REGULATE
void GrayLevelRegulate(ITS *iTS)
{
	OSD_Color_Setup(OCN_YELLOW, iTS);
	GetGrayLevel((S_IMGH>>1) + 100,S_IMGCW - 100, 10,iTS);

	OSD_Color_Setup(OCN_YELLOW, iTS);
	GetGrayLevel((S_IMGH>>1) + 100,S_IMGCW      , 10,iTS);

	OSD_Color_Setup(OCN_YELLOW, iTS);
	GetGrayLevel((S_IMGH>>1) + 100,S_IMGCW + 100, 10,iTS);

	OSD_Color_Setup(OCN_YELLOW, iTS);
	GetGrayLevel((S_IMGH>>1) -  50,S_IMGCW - 100, 10,iTS);

	OSD_Color_Setup(OCN_YELLOW, iTS);
	GetGrayLevel((S_IMGH>>1) -  50,S_IMGCW      , 10,iTS);

	OSD_Color_Setup(OCN_YELLOW, iTS);
	GetGrayLevel((S_IMGH>>1) -  50,S_IMGCW + 100, 10,iTS);

}

void GetGrayLevel(short RectStartRow, short RectStartCol, short DrawSize, ITS *iTS)
{
	unsigned int Avg=0;
	short count = 0;
	short row_,col;
	int r=(iTS->F_W * RectStartRow + RectStartCol);
	for (row_ = RectStartRow; row_ < RectStartRow + DrawSize; row_ ++)
	{
		for(col = 0; col < DrawSize; col++)
		{
			Avg+=iTS->YImg[r + col];
			count++;
		}
		memset((iTS->Showimage + iTS->F_W * row_ + RectStartCol), iTS->OSD_Color.Y, DrawSize);
		memset(&iTS->ShowUImg[(r>>1)], iTS->OSD_Color.U, DrawSize>>1);
		memset(&iTS->ShowVImg[(r>>1)], iTS->OSD_Color.V, DrawSize>>1);
		r+=iTS->F_W;
	}
	Avg = Avg / count;
	OSD_Color_Setup(OCN_RED, iTS);
	ScalableNumber((short) Avg, 4, RectStartCol-20, RectStartRow-25, iTS);
}
#endif //GRAY_LEVEL_REGULATE

#ifdef INCREASE_CONTRAST_ON_OFF
void IncreaseContrast(ITS *iTS)
{
	int index = 0;

	for(index = 0; index < 256; index++)
	{
		iTS->increase_contrast_map[index] = index;
	}

	for(index = 90; index < 160; index++)
	{
  		iTS->increase_contrast_map[index] = (unsigned char)(((double)iTS->increase_contrast_map[index] - INCREASE_CONTRAST_PARAMETER) * INCREASE_CONTRAST_PARAMETER2);

		if(iTS->increase_contrast_map[index] <1)
			iTS->increase_contrast_map[index] = 0;
		if(iTS->increase_contrast_map[index] > 255)
			iTS->increase_contrast_map[index] = 255;
	}
}
#endif //INCREASE_CONTRAST_ON_OFF

