#include "InitialVariable.h"
#include "FunctionType.h"

#ifdef L_PROTECTION_MAIN_ON_OFF

#ifdef L_SHOW_PROTECTION_WARNING_SIGN
void L_ShowWarningForProtection(short Protection_Mode, ITS *iTS)
{
	switch(Protection_Mode)
	{
		case MIDDLE_BLOCK_OFF:
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(40, 168, 20, iTS);
			break;

		case MIDDLE_BLOCK_ON:
			OSD_Color_Setup(OCN_RED, iTS);
			DrawRect_Has_UV(40, 168, 20, iTS);
			break;

		case SIDE_BLOCK_OFF:
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(40, 191, 20, iTS);
			break;

		case SIDE_BLOCK_SINGLE_ON:
			OSD_Color_Setup(OCN_RED, iTS);
			DrawRect_Has_UV(40, 191, 20, iTS);
			break;

		case SMEAR_OFF:
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(40, 250, 20, iTS);
			break;

		case SMEAR_SKIP_ON:
			OSD_Color_Setup(OCN_YELLOW, iTS);
			DrawRect_Has_UV(40, 250, 20, iTS);
			break;

		case SMEAR_ON:
			OSD_Color_Setup(OCN_RED, iTS);
			DrawRect_Has_UV(40, 250, 20, iTS);
			break;

		case GROUND_REFLECTOR_ON:
			OSD_Color_Setup(OCN_RED, iTS);
			DrawRect_Has_UV(40, 296, 20, iTS);
			break;

		case GROUND_REFLECTOR_OFF:
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(40, 296, 20, iTS);
			break;

		case WATER_STREAK_ON:
			OSD_Color_Setup(OCN_RED, iTS);
			DrawRect_Has_UV(40, 273, 20, iTS);
			break;

		case WATER_STREAK_OFF:
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(40, 273, 20, iTS);
			break;

		case MARKING_OFF:
			OSD_Color_Setup(OCN_BLUE, iTS);
			DrawRect_Has_UV(40, 94, 20, iTS);
			break;

		case MARKING_ON:
			OSD_Color_Setup(OCN_RED, iTS);
			DrawRect_Has_UV(40, 94, 20, iTS);
			break;

		default:
			//不顯示
			break;
	}
}
#endif    //L_SHOW_PROTECTION_WARNING_SIGN

void L_Protection_Initial(ITS *iTS)
{
	iTS->L_LeftLane_Difference_Value = 0;
	iTS->L_RightLane_Difference_Value = 0;

#ifdef L_PROTECTION_MIDDLE_BLOCK_ON_OFF
	iTS->L_MiddleBlock_LaneBount_L = 0;
	iTS->L_MiddleBlock_LaneBount_R = 0;
#endif    //L_PROTECTION_MIDDLE_BLOCK_ON_OFF

#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
	iTS->L_TopBlock_Avg = 0;
#endif    //L_PROTECTION_SIDE_BLOCK_ON_OFF

#ifdef L_WATER_STREAK_DETECTION_ON_OFF
	iTS->L_Left_Lane_Pixel_Total = 0;
	iTS->L_Left_Lane_Pixel_Point = 0;
	iTS->L_Right_Lane_Pixel_Total = 0;
	iTS->L_Right_Lane_Pixel_Point = 0;
#endif    //#ifdef L_WATER_STREAK_DETECTION_ON_OFF
}

void L_DecisionForSystemProtection(ITS *iTS)
{
	//保護機制
	if(iTS->L_Protection_MiddleBlock == MIDDLE_BLOCK_ON || iTS->L_Protection_Smear == SMEAR_ON || iTS->L_Protection_GroundReflector == GROUND_REFLECTOR_ON ||
	   iTS->L_Protection_WaterStreak == WATER_STREAK_ON || iTS->L_UsingPerception_RoadMarking == MARKING_ON)
	{
		iTS->L_Protection_Main = PROTECTION_ON;
	}
	else
	{
		iTS->L_Protection_Main = PROTECTION_OFF;
	}
	//使用者觀感
	if(iTS->L_UsingPerception_PointShift == POINT_SHIFT_ON || iTS->L_UsingPerception_LaneTilt == LANE_TILT_ON)
	{
		iTS->L_UsingPerception_Main = USING_PERCEPTION_ON;
	}
	else
	{
		iTS->L_UsingPerception_Main = USING_PERCEPTION_OFF;
	}
}

#ifdef L_PROTECTION_SIDE_BLOCK_ON_OFF
void L_FindSideBlockMean(short StartRow, short L_StartCol, short R_StartCol, short RowSize, short ColSize, ITS *iTS)
{
	short index_1 = 0;
	short index_2 = 0;
	int block_pixel_position = 0;
	int block_2_3_center_point = L_StartCol + (short) ColSize / 3;
	int block_1_2_center_point = block_2_3_center_point + 1 + (short) ColSize / 3;
	int block_4_5_center_point = R_StartCol + (short) ColSize / 3;
	int block_5_6_center_point = block_4_5_center_point + 1 + (short) ColSize / 3;
	unsigned int block_average1 = 0;
	unsigned int block_average2 = 0;
	unsigned int block_average3 = 0;
	unsigned int block_average4 = 0;
	unsigned int block_average5 = 0;
	unsigned int block_average6 = 0;
	short pixel_count1 = 0;
	short pixel_count2 = 0;
	short pixel_count3 = 0;
	short pixel_count4 = 0;
	short pixel_count5 = 0;
	short pixel_count6 = 0;
	short block_max1 = 0;
	short block_max2 = 0;
	short block_max3 = 0;
	short block_max4 = 0;
	short block_max5 = 0;
	short block_max6 = 0;

	for(index_1 = StartRow; index_1 < (StartRow + RowSize); index_1++)
	{
		for(index_2 = L_StartCol; index_2 < (L_StartCol + ColSize); index_2++)
		{
			block_pixel_position = (index_1 * iTS->F_W + index_2);

#ifdef L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE
			if(index_1 == StartRow || index_1 == (StartRow + RowSize - 1) || index_2 == L_StartCol || index_2 == (L_StartCol + ColSize - 1))
			{
				iTS->Showimage[block_pixel_position] = 255;
			}
			if(index_2 == block_2_3_center_point || index_2 == block_1_2_center_point)
			{
				iTS->Showimage[block_pixel_position] = 255;
			}
#endif    //L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE

			if(index_2 > block_1_2_center_point)
			{
				block_average1 += iTS->YImg[block_pixel_position];
				pixel_count1++;
				if(iTS->YImg[block_pixel_position] >= L_PROTECTION_SIDE_BLOCK_MAX_THRESHOLD)
				{
					block_max1++;
				}
			}
			if((index_2 > block_2_3_center_point) && (index_2 < block_1_2_center_point))
			{
				block_average2 += iTS->YImg[block_pixel_position];
				pixel_count2++;
				if(iTS->YImg[block_pixel_position] >= L_PROTECTION_SIDE_BLOCK_MAX_THRESHOLD)
				{
					block_max2++;
				}
			}
			if(index_2 < block_2_3_center_point)
			{
				block_average3 += iTS->YImg[block_pixel_position];
				pixel_count3++;
				if(iTS->YImg[block_pixel_position] >= L_PROTECTION_SIDE_BLOCK_MAX_THRESHOLD)
				{
					block_max3++;
				}
			}
		}
		for(index_2 = R_StartCol; index_2 < (R_StartCol + ColSize); index_2++)
		{
			block_pixel_position = (index_1 * iTS->F_W + index_2);

#ifdef L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE
			if(index_1 == StartRow || index_1 == (StartRow + RowSize - 1) || index_2 == R_StartCol || index_2 == (R_StartCol + ColSize - 1))
			{
				iTS->Showimage[block_pixel_position] = 255;
			}
			if(index_2 == block_4_5_center_point || index_2 == block_5_6_center_point)
			{
				iTS->Showimage[block_pixel_position] = 255;
			}
#endif    //L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE

			if(index_2 < block_4_5_center_point)
			{
				block_average4 += iTS->YImg[block_pixel_position];
				pixel_count4++;
				if(iTS->YImg[block_pixel_position] >= L_PROTECTION_SIDE_BLOCK_MAX_THRESHOLD)
				{
					block_max4++;
				}
			}
			if((index_2 > block_4_5_center_point) && (index_2 < block_5_6_center_point))
			{
				block_average5 += iTS->YImg[block_pixel_position];
				pixel_count5++;
				if(iTS->YImg[block_pixel_position] >= L_PROTECTION_SIDE_BLOCK_MAX_THRESHOLD)
				{
					block_max5++;
				}
			}
			if(index_2 > block_5_6_center_point)
			{
				block_average6 += iTS->YImg[block_pixel_position];
				pixel_count6++;
				if(iTS->YImg[block_pixel_position] >= L_PROTECTION_SIDE_BLOCK_MAX_THRESHOLD)
				{
					block_max6++;
				}
			}
		}
	}
	block_average1 = block_average1 / pixel_count1;
	block_average2 = block_average2 / pixel_count2;
	block_average3 = block_average3 / pixel_count3;
	block_average4 = block_average4 / pixel_count4;
	block_average5 = block_average5 / pixel_count5;
	block_average6 = block_average6 / pixel_count6;
	block_max1 =(short) ((block_max1 * 100) / pixel_count1);
	block_max2 =(short) ((block_max2 * 100) / pixel_count2);
	block_max3 =(short) ((block_max3 * 100) / pixel_count3);
	block_max4 =(short) ((block_max4 * 100) / pixel_count4);
	block_max5 =(short) ((block_max5 * 100) / pixel_count5);
	block_max6 =(short) ((block_max6 * 100) / pixel_count6);

	L_DecisionTopBlockMean(iTS);

	if((block_average1 > L_PROTECTION_SIDE_BLOCK_AVERAGE_THRESHOLD && block_max1 > L_PROTECTION_SIDE_BLOCK_PERCENT_THRESHOLD))// ||
	   //(block_average2 > L_PROTECTION_SIDE_BLOCK_AVERAGE_THRESHOLD && block_max2 > L_PROTECTION_SIDE_BLOCK_PERCENT_THRESHOLD) ||
	   //(block_average3 > L_PROTECTION_SIDE_BLOCK_AVERAGE_THRESHOLD && block_max3 > L_PROTECTION_SIDE_BLOCK_PERCENT_THRESHOLD))
	{
		if(iTS->L_Left_SideBlock_Counter != 0)
		{
			iTS->L_Left_SideBlock_Counter = 20;
		}
		if((iTS->L_StbCtr > 10) &&
		   iTS->L_LeftLane_Difference_Value < L_PROTECTION_SIDE_BLOCK_DIFFERENCE_THRESHOLD && iTS->L_LeftLane_Difference_Value > 0 && iTS->L_TopBlock_Avg < L_PROTECTION_SIDE_BLOCK_TOPBLOCKMEAN_THRESHOLD)
		{
			iTS->L_Left_SideBlock_Counter = 20;
		}
	}
	else
	{
		iTS->L_Left_SideBlock_Counter = LimitCount(-- iTS->L_Left_SideBlock_Counter, 20);
	}
	if((block_average4 > L_PROTECTION_SIDE_BLOCK_AVERAGE_THRESHOLD && block_max4 > L_PROTECTION_SIDE_BLOCK_PERCENT_THRESHOLD))// ||
	   //(block_average5 > L_PROTECTION_SIDE_BLOCK_AVERAGE_THRESHOLD && block_max5 > L_PROTECTION_SIDE_BLOCK_PERCENT_THRESHOLD) ||
	   //(block_average6 > L_PROTECTION_SIDE_BLOCK_AVERAGE_THRESHOLD && block_max6 > L_PROTECTION_SIDE_BLOCK_PERCENT_THRESHOLD))
	{
		if(iTS->L_Right_SideBlock_Counter != 0)
		{
			iTS->L_Right_SideBlock_Counter = 20;
		}
		if((iTS->L_StbCtr > 10) &&
		   iTS->L_RightLane_Difference_Value < L_PROTECTION_SIDE_BLOCK_DIFFERENCE_THRESHOLD && iTS->L_RightLane_Difference_Value > 0 && iTS->L_TopBlock_Avg < L_PROTECTION_SIDE_BLOCK_TOPBLOCKMEAN_THRESHOLD)
		{
			iTS->L_Right_SideBlock_Counter = 20;
		}
	}
	else
	{
		iTS->L_Right_SideBlock_Counter = LimitCount(-- iTS->L_Right_SideBlock_Counter, 20);
	}


#ifdef L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE
	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber((short) block_average1, 2, block_1_2_center_point + 10, StartRow - 15, iTS);
	ScalableNumber((short) block_average2, 2, block_2_3_center_point + 10, StartRow - 15, iTS);
	ScalableNumber((short) block_average3, 2, block_2_3_center_point - 30, StartRow - 15, iTS);
	ScalableNumber((short) block_average4, 2, block_4_5_center_point - 30, StartRow - 15, iTS);
	ScalableNumber((short) block_average5, 2, block_4_5_center_point + 10, StartRow - 15, iTS);
	ScalableNumber((short) block_average6, 2, block_5_6_center_point + 10, StartRow - 15, iTS);
	ScalableNumber((short) block_max1, 2, block_1_2_center_point + 10, StartRow + 15, iTS);
	ScalableNumber((short) block_max2, 2, block_2_3_center_point + 10, StartRow + 15, iTS);
	ScalableNumber((short) block_max3, 2, block_2_3_center_point - 20, StartRow + 15, iTS);
	ScalableNumber((short) block_max4, 2, block_4_5_center_point - 20, StartRow + 15, iTS);
	ScalableNumber((short) block_max5, 2, block_4_5_center_point + 10, StartRow + 15, iTS);
	ScalableNumber((short) block_max6, 2, block_5_6_center_point + 10, StartRow + 15, iTS);
#endif    //L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE

}
void L_Protection_DecisionSide(ITS *iTS)
{
	if(iTS->L_Left_SideBlock_Counter == 20)
	{
		iTS->L_Protection_SideBlockAdaptiveTH = SIDE_BLOCK_SINGLE_ON;
		iTS->Protection_Sideblock_Left_Draw = 0;
		iTS->Protection_Sideblock_Right_Draw = 1;
	}
	else if(iTS->L_Right_SideBlock_Counter == 20)
	{
		iTS->L_Protection_SideBlockAdaptiveTH = SIDE_BLOCK_SINGLE_ON;
		iTS->Protection_Sideblock_Left_Draw = 1;
		iTS->Protection_Sideblock_Right_Draw = 0;
	}
	else if(iTS->L_Left_SideBlock_Counter == 0 && iTS->L_Right_SideBlock_Counter == 0)
	{
		iTS->L_Protection_SideBlockAdaptiveTH = SIDE_BLOCK_OFF;
		iTS->Protection_Sideblock_Left_Draw = 1;
		iTS->Protection_Sideblock_Right_Draw = 1;
	}
}
void L_DecisionSkyline(ITS *iTS)
{
	//LaneEdge 參數宣告
	double K = 0;
	double M = 0;
	double Bl = 0;
	double Br = 0;
	double evRodSlp = 0;

#ifdef L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE
	//Draw Lane Edge 參數宣告
	double Vrow = 0;
	short row_ = 0;
	unsigned char *CurRow;
#endif    //L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE

	iTS->Skyline_V = S_IMGH - S_IMGCH;

#ifdef L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE
	//Draw Skyline
	memset(&iTS->Showimage[iTS->F_W * iTS->Skyline_V], 255, iTS->F_W);
#endif    //L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE

	if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10)
	{
		K = iTS->K;
		M = iTS->M;
		Bl = iTS->LaneL.B_i;
		Br = iTS->LaneR.B_i;
		evRodSlp = iTS->L_evRodSlp;
	}
	else
	{
		K = 0;
		M = 0;
		Bl = - (double)(L_RW_DefaultLW * S_euv) / (2 * S_HOCam);
		Br = + (double)(L_RW_DefaultLW * S_euv) / (2 * S_HOCam);
		evRodSlp = 0;
	}

#ifdef L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE
	//Draw Lane Edge
	for(row_ = 0; row_ < S_IMGCH; row_++)
	{
		Vrow = evRodSlp - (row_ - iTS->F_H_C);
		iTS->L_SideBlock_L_Edge = LimitW(iTS->F_W_C + LaneModel(Vrow, K, M, Bl));
		iTS->L_SideBlock_R_Edge = LimitW(iTS->F_W_C + LaneModel(Vrow, K, M, Br));
		CurRow = &iTS->Showimage[((iTS->F_H - 1) - row_) * iTS->F_W];
		CurRow[iTS->L_SideBlock_L_Edge] = 255;
		CurRow[iTS->L_SideBlock_R_Edge] = 255;
	}
#endif    //L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE

#ifndef L_DUAL_LANE_EDGE_SETUP
	iTS->L_SideBlock_L_Edge = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (S_IMGH - iTS->Skyline_V - L_BLOCK_ROW_DISTANCE_VANISHPOINT - iTS->F_H_C), K, M, Bl));
	iTS->L_SideBlock_R_Edge = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (S_IMGH - iTS->Skyline_V - L_BLOCK_ROW_DISTANCE_VANISHPOINT - iTS->F_H_C), K, M, Br));

	if((iTS->L_SideBlock_L_Edge - 10 < L_PROTECTION_SIDE_BLOCK_COL_SIZE + L_BLOCK_COL_DISTANCE_LANE) ||
	   (S_IMGW - 10 < L_PROTECTION_SIDE_BLOCK_COL_SIZE + L_BLOCK_COL_DISTANCE_LANE + iTS->L_SideBlock_R_Edge) ||
	   (iTS->L_SideBlock_L_Edge >= iTS->L_SideBlock_R_Edge))
	{
		iTS->L_SideBlock_L_Edge = L_LEFT_EDGE;
		iTS->L_SideBlock_R_Edge = L_RIGHT_EDGE;
	}
	else
	{
		iTS->L_SideBlock_L_Edge = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (S_IMGH - iTS->Skyline_V - L_BLOCK_ROW_DISTANCE_VANISHPOINT - iTS->F_H_C), K, M, Bl));
		iTS->L_SideBlock_R_Edge = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (S_IMGH - iTS->Skyline_V - L_BLOCK_ROW_DISTANCE_VANISHPOINT - iTS->F_H_C), K, M, Br));
	}
#endif    //L_DUAL_LANE_EDGE_SETUP

#ifdef L_DUAL_LANE_EDGE_SETUP
		iTS->L_SideBlock_L_Edge = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (S_IMGH - iTS->Skyline_V - L_BLOCK_ROW_DISTANCE_VANISHPOINT - iTS->F_H_C), K, M, Bl));
		iTS->L_SideBlock_R_Edge = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (S_IMGH - iTS->Skyline_V - L_BLOCK_ROW_DISTANCE_VANISHPOINT - iTS->F_H_C), K, M, Br));
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber(iTS->L_SideBlock_L_Edge, 4, 100, iTS->Skyline_V + 100, iTS);
		ScalableNumber(iTS->L_SideBlock_R_Edge, 4, S_IMGW - 100, iTS->Skyline_V + 100, iTS);
#endif    //L_DUAL_LANE_EDGE_SETUP

}
void L_DecisionTopBlockMean(ITS *iTS)
{
	unsigned int average = 0;
	short index_1 = 0;
	short index_2 = 0;
	short pixel_count = 0;

	short block_height = 10;
	short start_row = (S_IMGH - S_IMGCH) - (block_height * 2);
	short start_col = S_IMGCW - 50;
	short block_width = iTS->F_W - (start_col << 1);

	int block_initial=(iTS->F_W * (start_row - 1) + start_col);
	static short L_LastTopBlock_Avg = 0;

	//計算區塊平均值
	for (index_1 = start_row; index_1 < start_row + block_height + 1; index_1++)
	{
		for(index_2 = 0; index_2 < block_width + 1; index_2++)
		{
			average += iTS->YImg[block_initial + index_2];
			pixel_count++;

#ifdef L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE
			//畫出計算區塊
			if(index_1 == start_row || index_1 == start_row + block_height || index_2 == 0 || index_2 == block_width)
			{
				iTS->Showimage[block_initial + index_2] = 255;
			}
#endif    //L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE

		}

		block_initial += iTS->F_W;
	}

	//避免除數為零
	if(pixel_count == 0 || average == 0)
	{
		iTS->L_TopBlock_Avg = 0;
		pixel_count = 0;
	}
	else
	{
		iTS->L_TopBlock_Avg = average / pixel_count;

		if(L_LastTopBlock_Avg == 0)
		{
			L_LastTopBlock_Avg = L_PROTECTION_SIDE_BLOCK_TOPBLOCKMEAN_THRESHOLD;
		}

		//區塊平均值smooth
		iTS->L_TopBlock_Avg = ((L_LastTopBlock_Avg * 7) + iTS->L_TopBlock_Avg) >> 3;

		if(iTS->L_TopBlock_Avg > 0 && iTS->L_TopBlock_Avg <= 255)
		{
			L_LastTopBlock_Avg = iTS->L_TopBlock_Avg;
		}
	}

#ifdef L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE
	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber((short)iTS->L_TopBlock_Avg, 4, start_col, start_row, iTS);
#endif    //L_SHOW_SIDE_BLOCK_EDGE_AND_VALUE

}
#endif    //L_PROTECTION_SIDE_BLOCK_ON_OFF

#ifdef L_PROTECTION_MIDDLE_BLOCK_ON_OFF
void L_SetLandBoundForProtection(unsigned char *Img,short row, ITS *iTS)
{
	double K = 0;
	double M = 0;
	double Bl = 0;
	double Br = 0;
	double evRodSlp = 0;

#ifdef L_SHOW_MIDDLE_BLOCK_EDGE
	double Vrow = 0;
	unsigned char *CurRow;
	short index = 0;
	short bound_L = 0;
	short bound_R = 0;
#endif    //L_SHOW_MIDDLE_BLOCK_EDGE

	if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr >= 15)
	{
		K = iTS->K;
		M = iTS->M;
		Bl = iTS->LaneL.B_i;
		Br = iTS->LaneR.B_i;
		evRodSlp = iTS->L_evRodSlp;
	}
	else
	{
		K = 0;
		M = 0;
		Bl = - (double)(L_RW_DefaultLW * S_euv) / (2 * S_HOCam);
		Br = + (double)(L_RW_DefaultLW * S_euv) / (2 * S_HOCam);
		evRodSlp = 0;
	}

	iTS->L_MiddleBlock_LaneBount_L = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (row - iTS->F_H_C), K, M, Bl));
	iTS->L_MiddleBlock_LaneBount_R = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (row - iTS->F_H_C), K, M, Br));

	//避免車道寬度過窄或過寬而影響M_subBlock3和M_subBlock4的選取範圍
	if((iTS->L_MiddleBlock_LaneBount_R - iTS->L_MiddleBlock_LaneBount_L) < (S_IMGCW >> 1) || (iTS->L_MiddleBlock_LaneBount_R - iTS->L_MiddleBlock_LaneBount_L) > S_IMGCW)
	{
		K = 0;
		M = 0;
		Bl = - (double)(L_RW_DefaultLW * S_euv) / (2 * S_HOCam);
		Br = + (double)(L_RW_DefaultLW * S_euv) / (2 * S_HOCam);
		evRodSlp = 0;

		iTS->L_MiddleBlock_LaneBount_L = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (row - iTS->F_H_C), K, M, Bl));
		iTS->L_MiddleBlock_LaneBount_R = LimitW(iTS->F_W_C + LaneModel(evRodSlp - (row - iTS->F_H_C), K, M, Br));
	}

#ifdef L_SHOW_MIDDLE_BLOCK_EDGE
	//顯示車道線邊界
	for(index = 0; index < S_IMGCH; index++)
	{
		Vrow = evRodSlp - (index - iTS->F_H_C);
		bound_L = LimitW(iTS->F_W_C + LaneModel(Vrow, K, M, Bl));
		bound_R = LimitW(iTS->F_W_C + LaneModel(Vrow, K, M, Br));
		CurRow = &Img[((iTS->F_H - 1) - index) * iTS->F_W];
		CurRow[bound_L] = 255;
		CurRow[bound_R] = 255;
	}
	memset(&iTS->Showimage[iTS->F_W * (iTS->F_H - iTS->F_H_C)], 255, iTS->F_W);
#endif    //L_SHOW_MIDDLE_BLOCK_EDGE

}

void L_Protection_MiddleBlock(ITS *iTS)
{
	unsigned int M_subBlock1_mean = 0;
	unsigned int M_subBlock2_mean = 0;
	unsigned int M_subBlock3_mean = 0;
	unsigned int M_subBlock4_mean = 0;

	short M_totalBlock_mean = 0;
	short index_1 = 0;
	short index_2 = 0;
	short index_3 = 0;

	short M_subBlock1_count = 0;
	short M_subBlock2_count = 0;
	short M_subBlock3_count = 0;
	short M_subBlock4_count = 0;
	char M_decision_count = 0;
	static char MiddleBlock_Count = 0;

	int M_subBlock1_initial = 0;
	int M_subBlock3_initial = 0;

	short StartRow12 = (short) (S_IMGH - (((S_IMGCH - S_IMGBB) * L_PROTECTION_MIDDLE_BLOCK_HEIGHT_RATIO) + S_IMGBB));
	short EndRow12 = (short) (S_IMGH - (((S_IMGCH - S_IMGBB) * L_PROTECTION_MIDDLE_BLOCK_HEIGHT_RATIO * 0.5) + S_IMGBB));
	short StartRow3 = (short) (StartRow12 - ((S_IMGCH - S_IMGBB) * 0.1));

	short subBlock12_LaneDistance = 0;
	short subBlock34_LaneDistance = 0;

	short RoadMean = 0;
	static short M_SecondThreshold = 0;
	static short M_FourthThreshold = 0;

	//計算LaneBountL與LaneBountR
	L_SetLandBoundForProtection(iTS->Showimage, (short) (S_IMGH - StartRow12), iTS);

	//區塊M_subBlock1和M_subBlock3起始點
    subBlock12_LaneDistance = (short)(fabs(iTS->L_MiddleBlock_LaneBount_R - iTS->L_MiddleBlock_LaneBount_L) * 0.2);
	M_subBlock1_initial = iTS->F_W * (StartRow12 - 1) + (iTS->L_MiddleBlock_LaneBount_L + subBlock12_LaneDistance);

    subBlock34_LaneDistance = (short)(fabs(iTS->L_MiddleBlock_LaneBount_R - iTS->L_MiddleBlock_LaneBount_L) * 0.4);
	M_subBlock3_initial = iTS->F_W * (StartRow3 - 1) + (iTS->L_MiddleBlock_LaneBount_L + subBlock34_LaneDistance);

	//計算M_subBlock1和M_subBlock2的平均值
	for (index_1 = StartRow12; index_1 < EndRow12; index_1++)
	{
		for(index_2 = 0; index_2 < 10; index_2++)
		{
			M_subBlock1_mean += iTS->YImg[M_subBlock1_initial + index_2];
			M_subBlock1_count++;
			M_subBlock2_mean += iTS->YImg[M_subBlock1_initial + (iTS->L_MiddleBlock_LaneBount_R - iTS->L_MiddleBlock_LaneBount_L - (subBlock12_LaneDistance << 1) - 10) + index_2];
			M_subBlock2_count++;

#ifdef L_SHOW_MIDDLE_BLOCK_EDGE
			//畫出M_subBlock1和M_subBlock2的計算區域
			if(index_1 == StartRow12 || index_1 == (EndRow12 - 1) || index_2 == 0 || index_2 == (10 - 1))
			{
				iTS->Showimage[M_subBlock1_initial + index_2] = 255;
				iTS->Showimage[M_subBlock1_initial + (iTS->L_MiddleBlock_LaneBount_R - iTS->L_MiddleBlock_LaneBount_L - (subBlock12_LaneDistance << 1) - 10) + index_2] = 255;
			}
#endif    //L_SHOW_MIDDLE_BLOCK_EDGE

		}
		M_subBlock1_initial += iTS->F_W;
	}

	//避免除數為零
	if(M_subBlock1_count == 0 || M_subBlock1_mean == 0)
	{
		M_subBlock1_mean = 0;
		M_subBlock1_count = 0;
	}
	else
	{
		M_subBlock1_mean = M_subBlock1_mean / M_subBlock1_count;
	}
	if(M_subBlock2_count == 0 || M_subBlock2_mean == 0)
	{
		M_subBlock2_mean = 0;
		M_subBlock2_count = 0;
	}
	else
	{
		M_subBlock2_mean = M_subBlock2_mean / M_subBlock2_count;
	}

	//計算M_subBlock3和M_subBlock4的平均值
	for (index_1 = StartRow3; index_1 < (StartRow3 + 10); index_1++)
	{
		for(index_2 = 0; index_2 < (iTS->L_MiddleBlock_LaneBount_R - iTS->L_MiddleBlock_LaneBount_L - (subBlock34_LaneDistance << 1)); index_2++)
		{
			M_subBlock3_mean += iTS->YImg[M_subBlock3_initial + index_2];
			M_subBlock3_count++;
			M_subBlock4_mean += iTS->YImg[M_subBlock3_initial + (iTS->F_W * ((StartRow12 - StartRow3) + (EndRow12 - StartRow12))) + index_2];
			M_subBlock4_count++;

#ifdef L_SHOW_MIDDLE_BLOCK_EDGE
			//畫出M_subBlock3和M_subBlock4的計算區域
			if(index_1 == StartRow3 || index_1 == ((StartRow3 + 10) - 1) || index_2 == 0 ||
			   index_2 == (iTS->L_MiddleBlock_LaneBount_R - iTS->L_MiddleBlock_LaneBount_L - (subBlock34_LaneDistance << 1) - 1))
			{
				iTS->Showimage[M_subBlock3_initial + index_2] = 255;
				iTS->Showimage[M_subBlock3_initial + (iTS->F_W * ((StartRow12 - StartRow3) + (EndRow12 - StartRow12))) + index_2] = 255;
			}
#endif    //L_SHOW_MIDDLE_BLOCK_EDGE

		}
		M_subBlock3_initial += iTS->F_W;
	}

	//避免除數為零
	if(M_subBlock3_count == 0 || M_subBlock3_mean == 0)
	{
		M_subBlock3_mean = 0;
		M_subBlock3_count = 0;
	}
	else
	{
		M_subBlock3_mean = M_subBlock3_mean / M_subBlock3_count;
	}

	if(M_subBlock4_count == 0 || M_subBlock4_mean == 0)
	{
		M_subBlock4_mean = 0;
		M_subBlock4_count = 0;
	}
	else
	{
		M_subBlock4_mean = M_subBlock4_mean / M_subBlock4_count;
	}

	//計算M_subBlock的總平均值
	//避免除數為零
	if((M_subBlock1_mean + M_subBlock2_mean + M_subBlock3_mean + M_subBlock4_mean) == 0)
	{
		M_totalBlock_mean = 0;
	}
	else
	{
		M_totalBlock_mean = (short)((M_subBlock1_mean + M_subBlock2_mean + M_subBlock3_mean + M_subBlock4_mean) >> 2);
	}

	RoadMean = (iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean + iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean) >> 1;

	if(iTS->L_LeftLane_Difference_Value == 0 || iTS->L_RightLane_Difference_Value == 0)
	{
		M_SecondThreshold = (short)((iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold + iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold) * L_PROTECTION_MIDDLE_BLOCK_SECOND_THRESHOLD);
		M_FourthThreshold = (short)((iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold + iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold) * L_PROTECTION_MIDDLE_BLOCK_FOURTH_THRESHOLD);
	}
	else
	{
		M_SecondThreshold = (short)((iTS->L_LeftLane_Difference_Value + iTS->L_RightLane_Difference_Value) * L_PROTECTION_MIDDLE_BLOCK_SECOND_THRESHOLD);
		M_FourthThreshold = (short)((iTS->L_LeftLane_Difference_Value + iTS->L_RightLane_Difference_Value) * L_PROTECTION_MIDDLE_BLOCK_FOURTH_THRESHOLD);
	}

	//判斷條件一 : M_totalBlock的總平均值
	if(M_totalBlock_mean > L_PROTECTION_MIDDLE_BLOCK_FIRST_THRESHOLD)
	{
		M_decision_count++;

#ifdef L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER
		OSD_Color_Setup(OCN_PURPLE, iTS);
		ScalableNumber(1, 2, iTS->F_W_C - 25, iTS->F_H - iTS->F_H_C, iTS);
#endif    //L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER

	}

	//判斷條件二 : RoadMean與M_subBlock差值總和
	if((abs((int)((unsigned int)RoadMean - M_subBlock1_mean)) + abs((int)((unsigned int)RoadMean - M_subBlock2_mean)) + abs((int)((unsigned int)RoadMean - M_subBlock3_mean)) + abs((int)((unsigned int)RoadMean - M_subBlock4_mean))) > M_SecondThreshold)
	{
		M_decision_count++;

#ifdef L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER
		OSD_Color_Setup(OCN_PURPLE, iTS);
		ScalableNumber(2, 2, iTS->F_W_C - 10, iTS->F_H - iTS->F_H_C, iTS);
#endif    //L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER

	}

	//判斷條件三 : M_subBlock的郁＃?
	if(M_subBlock1_mean > L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD)
	{
		index_3++;
	}
	if(M_subBlock2_mean > L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD)
	{
		index_3++;
	}
	if(M_subBlock3_mean > L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD)
	{
		index_3++;
	}
	if(M_subBlock4_mean > L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD)
	{
		index_3++;
	}

	if(index_3 >= 2)
	{
		M_decision_count++;

#ifdef L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER
		OSD_Color_Setup(OCN_PURPLE, iTS);
		ScalableNumber(3, 2, iTS->F_W_C + 5, iTS->F_H - iTS->F_H_C, iTS);
#endif    //L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER

	}

	//判斷條件四 : 平差值_MAX(M_subBlock) - MIN(M_subBlock)
	if(abs((int)(M_subBlock1_mean - M_subBlock2_mean)) > M_FourthThreshold || abs((int)(M_subBlock3_mean - M_subBlock4_mean)) > M_FourthThreshold ||
	   abs((int)(M_subBlock1_mean - M_subBlock3_mean)) > M_FourthThreshold || abs((int)(M_subBlock1_mean - M_subBlock4_mean)) > M_FourthThreshold ||
	   abs((int)(M_subBlock2_mean - M_subBlock3_mean)) > M_FourthThreshold || abs((int)(M_subBlock2_mean - M_subBlock4_mean)) > M_FourthThreshold)
	{
		M_decision_count++;

#ifdef L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER
		OSD_Color_Setup(OCN_PURPLE, iTS);
		ScalableNumber(4, 2, iTS->F_W_C + 20, iTS->F_H - iTS->F_H_C, iTS);
#endif    //L_SHOW_MIDDLE_BLOCK_DECISION_NUMBER

	}

	//符合以上四個判斷條件則啟動保護機制Protection_MiddleBlock
	if(M_decision_count >= 4)
	{
		MiddleBlock_Count = LimitCount(++ MiddleBlock_Count, 15);

		if(MiddleBlock_Count >= 15)
		{
			iTS->L_Protection_MiddleBlock = MIDDLE_BLOCK_ON;
		}
	}
	else //M_decision_count < 4
	{
		MiddleBlock_Count = LimitCount(-- MiddleBlock_Count, 15);

		if(MiddleBlock_Count < 5)
		{
			iTS->L_Protection_MiddleBlock = MIDDLE_BLOCK_OFF;
			MiddleBlock_Count = 0;
		}
	}

#ifdef L_SHOW_PROTECTION_WARNING_SIGN
	L_ShowWarningForProtection(iTS->L_Protection_MiddleBlock, iTS);
	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber((short) MiddleBlock_Count, 2, 175, 30, iTS);
#endif    //L_SHOW_PROTECTION_WARNING_SIGN

#ifdef L_SHOW_MIDDLE_BLOCK_EDGE
	//顯示M_subBlock1的平均值
	if(M_subBlock1_mean > L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short)M_subBlock1_mean, 4, (iTS->L_MiddleBlock_LaneBount_L + subBlock12_LaneDistance), StartRow12 - 25, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_ORANGE, iTS);
		ScalableNumber((short)M_subBlock1_mean, 2, (iTS->L_MiddleBlock_LaneBount_L + subBlock12_LaneDistance), StartRow12 - 15, iTS);
	}
	//顯示M_subBlock2的平均值
	if(M_subBlock2_mean > L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short)M_subBlock2_mean, 4, (iTS->L_MiddleBlock_LaneBount_R - subBlock12_LaneDistance), StartRow12 - 25, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_ORANGE, iTS);
		ScalableNumber((short)M_subBlock2_mean, 2, (iTS->L_MiddleBlock_LaneBount_R - subBlock12_LaneDistance), StartRow12 - 15, iTS);
	}
	//顯示M_subBlock3的平均值
	if(M_subBlock3_mean > L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short)M_subBlock3_mean, 4, (iTS->L_MiddleBlock_LaneBount_L + subBlock34_LaneDistance), StartRow3 - 25, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_ORANGE, iTS);
		ScalableNumber((short)M_subBlock3_mean, 2, (iTS->L_MiddleBlock_LaneBount_L + subBlock34_LaneDistance), StartRow3 - 15, iTS);
	}
	//顯示M_subBlock4的平均值
	if(M_subBlock4_mean > L_PROTECTION_MIDDLE_BLOCK_THIRD_THRESHOLD)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short)M_subBlock4_mean, 4, (iTS->L_MiddleBlock_LaneBount_L + subBlock34_LaneDistance), StartRow3 + 25, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_ORANGE, iTS);
		ScalableNumber((short)M_subBlock4_mean, 2, (iTS->L_MiddleBlock_LaneBount_L + subBlock34_LaneDistance), StartRow3 + 35, iTS);
	}
	//顯示M_totalBlock的平均值
	if(M_totalBlock_mean > L_PROTECTION_MIDDLE_BLOCK_FIRST_THRESHOLD)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short)M_totalBlock_mean, 4, 40, iTS->F_H - iTS->F_H_C  + 1, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_ORANGE, iTS);
		ScalableNumber((short)M_totalBlock_mean, 4, 40, iTS->F_H - iTS->F_H_C  + 1, iTS);
	}
#endif    //L_SHOW_MIDDLE_BLOCK_EDGE
}
#endif    //L_PROTECTION_MIDDLE_BLOCK_ON_OFF

#ifdef L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF
char L_CCD_FindSmear(ITS *iTS)
{
	short index_1 = 0;
	char index_2 = 0;
	char index_3 = 0;

	short GrayScale_threshold = 0;
	short row = iTS->F_H_C - 20;
	char row_bound = (row / SMEAR_ROW_NUMBER) - 1;
	unsigned short row_distance = 0;

	char smear_flag = 0;
	unsigned char smear_flag_count_1 = 0;
	unsigned char smear_flag_count_2 = 0;

	short smear_L_point = 0;
	short smear_R_point = 0;
	short smear_M_point = 0;
	short smear_MaxW = (short)(iTS->F_W * 0.1);
	char smear_ArrayIndex = 0;

	if(iTS->L_Protection_Smear == SMEAR_OFF)
	{
		memset(&iTS->L_Smear_LB[0], 0, sizeof(iTS->L_Smear_LB));
		memset(&iTS->L_Smear_RB[0], 0, sizeof(iTS->L_Smear_RB));
	}

	//判斷偵測漏光之範圍是否足夠
	if(row > (iTS->F_H - row))
	{
		row_bound = ((iTS->F_H - row) / SMEAR_ROW_NUMBER) - 1;
	}
	if(row < S_IMGBB || row_bound < 6 || (row - (SMEAR_ROW_NUMBER * row_bound)) < 0 || (row + (SMEAR_ROW_NUMBER * row_bound)) > iTS->F_H)
	{
		return 0;
	}

	//計算漏光之灰階門檻值
	if(iTS->L_LightLeak_GrayScaleThreshold != 0)
	{
		GrayScale_threshold = iTS->L_LightLeak_GrayScaleThreshold;

		if(GrayScale_threshold < SMEAR_SEARCH_GRAY_SCALE_LOWEST_LIMIT)
		{
			GrayScale_threshold = SMEAR_SEARCH_GRAY_SCALE_LOWEST_LIMIT;
		}
	}
	else
	{
		if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean == 0 || iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean == 0 ||
		   iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold == 0 || iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold == 0)
		{
			GrayScale_threshold = SMEAR_GRAY_SCALE_LOWEST_LIMIT;
		}
		else
		{
			GrayScale_threshold = (iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean + iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean +
	              	               iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold + iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold) >> 1;
		}
		if(GrayScale_threshold < SMEAR_GRAY_SCALE_LOWEST_LIMIT)
		{
			GrayScale_threshold = SMEAR_GRAY_SCALE_LOWEST_LIMIT;
		}
	}
#ifdef L_SMEAR_SHOW_VALUE_AND_LOCATION
	if(iTS->L_LightLeak_GrayScaleThreshold != 0)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short) GrayScale_threshold, 2, (S_IMGW >> 1) + 65, 85, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_BLUE, iTS);
		ScalableNumber((short) GrayScale_threshold, 2, (S_IMGW >> 1) + 65, 85, iTS);
	}
#endif    //L_SMEAR_SHOW_VALUE_AND_LOCATION

	//水平搜尋: Row
	for(index_1 = S_IMGLB; index_1 <= S_IMGRB; index_1++)
	{
		//smear_flag, 判斷灰階值是否達到漏光之門檻值
		if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1] > GrayScale_threshold)
		{
			row_distance = iTS->F_W * SMEAR_ROW_NUMBER;

			//index_2 = 1
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 + row_distance] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 - row_distance] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			//index_2 = 3
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 + (row_distance * 3)] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 - (row_distance * 3)] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			//index_2 = (row_bound >> 1)
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 + (row_distance * (row_bound >> 1))] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 - (row_distance * (row_bound >> 1))] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			//index_2 = (row_bound - 3)
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 + (row_distance * (row_bound - 3))] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 - (row_distance * (row_bound - 3))] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			//index_2 = (row_bound - 1)
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 + (row_distance * (row_bound - 1))] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}
			if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + index_1 - (row_distance * (row_bound - 1))] > GrayScale_threshold)
			{
				smear_flag_count_1++;
			}

			//初步判斷此Col位置是否疑似在漏d圍內
			if(smear_flag_count_1 >= 10)
			{
				smear_flag = 1;
			}
			else
			{
				smear_flag = 0;
			}

			//相關統計參數值初始化
			smear_flag_count_1 = 0;
		}
		else
		{
			smear_flag = 0;
		}

#ifdef L_SMEAR_SHOW_VALUE_AND_LOCATION
			if(smear_flag == 1)
			{
				iTS->Showimage[iTS->F_W * (iTS->F_H - row) + index_1] = 255;
			}
			else
			{
				iTS->Showimage[iTS->F_W * (iTS->F_H - row) + index_1] = 0;
			}
#endif    //L_SMEAR_SHOW_VALUE_AND_LOCATION

		if(smear_flag == 0)
		{
			//垂直搜尋: 判斷Col(smear_M_point)之垂直位置上的灰階值是否也達到漏光之門e值
			if(smear_L_point != 0 && smear_R_point != 0 && smear_M_point != 0)
			{
				for(index_2 = 1; index_2 <= row_bound; index_2++)
				{
					//避免重複計算
					if(index_2 == 1 || index_2 == 3 || index_2 == (row_bound >> 1) || index_2 == (row_bound - 3) || index_2 == (row_bound - 1))
					{
						index_2++;
					}

					row_distance = iTS->F_W * SMEAR_ROW_NUMBER * index_2;

					//smear_M_point
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + smear_M_point + row_distance] > GrayScale_threshold)
					{
						smear_flag_count_2++;
					}
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + smear_M_point - row_distance] > GrayScale_threshold)
					{
						smear_flag_count_2++;
					}
					//smear_L_point
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + (smear_L_point + 1) + row_distance] > GrayScale_threshold)
					{
						smear_flag_count_2++;
					}
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + (smear_L_point + 1) - row_distance] > GrayScale_threshold)
					{
						smear_flag_count_2++;
					}
					//smear_R_point
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + (smear_R_point - 1) + row_distance] > GrayScale_threshold)
					{
						smear_flag_count_2++;
					}
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + (smear_R_point - 1) - row_distance] > GrayScale_threshold)
					{
						smear_flag_count_2++;
					}

#ifdef L_SMEAR_SHOW_VALUE_AND_LOCATION
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + smear_M_point + row_distance] > GrayScale_threshold)
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_M_point + row_distance] = 255;
					}
					else
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_M_point + row_distance] = 0;
					}
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + smear_M_point - row_distance] > GrayScale_threshold)
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_M_point - row_distance] = 255;
					}
					else
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_M_point + row_distance] = 0;
					}
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + smear_L_point + row_distance] > GrayScale_threshold)
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_L_point + row_distance] = 255;
					}
					else
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_L_point + row_distance] = 0;
					}
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + smear_L_point - row_distance] > GrayScale_threshold)
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_L_point - row_distance] = 255;
					}
					else
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_L_point + row_distance] = 0;
					}
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + smear_R_point + row_distance] > GrayScale_threshold)
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_R_point + row_distance] = 255;
					}
					else
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_R_point + row_distance] = 0;
					}
					if(iTS->YImg[iTS->F_W * (iTS->F_H - row) + smear_R_point - row_distance] > GrayScale_threshold)
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_R_point - row_distance] = 255;
					}
					else
					{
						iTS->Showimage[iTS->F_W * (iTS->F_H - row) + smear_R_point + row_distance] = 0;
					}
#endif    //L_SMEAR_SHOW_VALUE_AND_LOCATION
				}

				//判斷漏光
				if(smear_flag_count_2 >= ((row_bound - 5) * 6 - 6))
				{
					//判斷漏光範圍是否大於0
					if(abs(smear_R_point - smear_L_point) > 0)
					{
						//判斷漏光範圍是否大於smear_MaxW
						if(abs(smear_R_point - smear_L_point) < smear_MaxW)
						{
							smear_ArrayIndex++;
							iTS->L_Smear_LB[smear_ArrayIndex - 1] = smear_L_point;
							iTS->L_Smear_RB[smear_ArrayIndex - 1] = smear_R_point;
						}
						else
						{
							smear_ArrayIndex++;
							iTS->L_Smear_LB[smear_ArrayIndex - 1] = smear_M_point - (smear_MaxW >> 1);
							iTS->L_Smear_RB[smear_ArrayIndex - 1] = smear_M_point + (smear_MaxW >> 1);
						}

						if(smear_ArrayIndex == 3)
						{
							//避免LightLeak_GrayScaleThreshold過低而造成誤偵測
							if(iTS->L_LightLeak_GrayScaleThreshold != 0)
							{
								iTS->L_LightLeak_GrayScaleThreshold = 0;
							}
							return 1;
						}
					}
				}
			}

			//相關統計參數值初始化
			smear_L_point = 0;
			smear_R_point = 0;
			smear_M_point = 0;
			smear_flag_count_2 = 0;
		}
		else //(smear_flag == 1)
		{
			//計算水平Row上疑似漏光之Col範圍Row(smear_L_point, smear_R_point)
			if(smear_L_point == 0 && smear_R_point == 0 && smear_M_point == 0)
			{
				smear_L_point = index_1;
				smear_R_point = index_1;
				smear_M_point = index_1;
			}
			else if((index_1 - smear_R_point) == 1)
			{
				smear_R_point = index_1;
				smear_M_point = (smear_L_point + smear_R_point) >> 1;
			}
			else
			{
				smear_flag = 0;
			}
		}
	}

#ifdef L_SMEAR_SHOW_VALUE_AND_LOCATION
	OSD_Color_Setup(OCN_BLUE, iTS);
	ScalableNumber((short) smear_ArrayIndex, 2, (S_IMGW >> 1) + 40, 85, iTS);
#endif    //L_SMEAR_SHOW_VALUE_AND_LOCATION

	if(smear_ArrayIndex == 0)
	{
		return 0;
	}
	else
	{
		for(index_3 = smear_ArrayIndex; index_3 < 3; index_3++)
		{
			iTS->L_Smear_LB[index_3] = 0;
			iTS->L_Smear_RB[index_3] = 0;
		}

		return 1;
	}
}

void L_Decision_CCD_Smear(ITS *iTS)
{
	static char decision_count = 0;
	short row = 0;
	short distance = (short)(S_IMGW * SMEAR_SWITCH_DISTANCE_RATE);

	if(L_CCD_FindSmear(iTS))
	{
		decision_count = LimitCount(++decision_count, (SMEAR_SWITCH_ON_COUNT + 5));

		if(decision_count >= SMEAR_SWITCH_ON_COUNT)
		{
			iTS->L_Protection_Smear = SMEAR_SKIP_ON;
		}
	}
	else
	{
		decision_count = LimitCount(--decision_count, (SMEAR_SWITCH_ON_COUNT + 5));

		if(decision_count == 0)
		{
			iTS->L_Protection_Smear = SMEAR_OFF;
		}

		if(decision_count < SMEAR_SWITCH_ON_COUNT)
		{
			iTS->L_LightLeak_GrayScaleThreshold = 0;
		}
	}

	if((iTS->L_Protection_Smear == SMEAR_SKIP_ON || iTS->L_Protection_Smear == SMEAR_ON) && iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > 10)
	{
		row = ((L_IB_TB_DrawLane - S_IMGBB) >> 1) + S_IMGBB;

		if(iTS->SL_LaneModel.L_SL_LorR == SL_Left)
		{
			if(iTS->L_Smear_LB[0] != 0 && iTS->L_Smear_RB[0] != 0 && iTS->L_LaneMBound[row].Frt > (iTS->L_Smear_LB[0] - distance) && iTS->L_LaneMBound[row].Frt < (iTS->L_Smear_LB[0] + distance))
			{
				iTS->L_Protection_Smear = SMEAR_ON;
			}
			if(iTS->L_Smear_LB[1] != 0 && iTS->L_Smear_RB[1] != 0 && iTS->L_LaneMBound[row].Frt > (iTS->L_Smear_LB[1] - distance) && iTS->L_LaneMBound[row].Frt < (iTS->L_Smear_LB[1] + distance))
			{
				iTS->L_Protection_Smear = SMEAR_ON;
			}
			if(iTS->L_Smear_LB[2] != 0 && iTS->L_Smear_RB[2] != 0 && iTS->L_LaneMBound[row].Frt > (iTS->L_Smear_LB[2] - distance) && iTS->L_LaneMBound[row].Frt < (iTS->L_Smear_LB[2] + distance))
			{
				iTS->L_Protection_Smear = SMEAR_ON;
			}
		}
		if(iTS->SL_LaneModel.L_SL_LorR == SL_Right)
		{
			if(iTS->L_Smear_LB[0] != 0 && iTS->L_Smear_RB[0] != 0 && iTS->L_LaneMBound[row].Scd > (iTS->L_Smear_RB[0] - distance) && iTS->L_LaneMBound[row].Scd < (iTS->L_Smear_RB[0] + distance))
			{
				iTS->L_Protection_Smear = SMEAR_ON;
			}
			if(iTS->L_Smear_LB[1] != 0 && iTS->L_Smear_RB[1] != 0 && iTS->L_LaneMBound[row].Scd > (iTS->L_Smear_RB[1] - distance) && iTS->L_LaneMBound[row].Scd < (iTS->L_Smear_RB[1] + distance))
			{
				iTS->L_Protection_Smear = SMEAR_ON;
			}
			if(iTS->L_Smear_LB[2] != 0 && iTS->L_Smear_RB[2] != 0 && iTS->L_LaneMBound[row].Scd > (iTS->L_Smear_RB[2] - distance) && iTS->L_LaneMBound[row].Scd < (iTS->L_Smear_RB[2] + distance))
			{
				iTS->L_Protection_Smear = SMEAR_ON;
			}
		}
	}

#ifdef L_SHOW_PROTECTION_WARNING_SIGN
	//顯示漏光警示訊號
	L_ShowWarningForProtection(iTS->L_Protection_Smear, iTS);
	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber((short) decision_count, 2, 253, 30, iTS);
#endif    //L_SHOW_PROTECTION_WARNING_SIGN
#ifdef L_SMEAR_SHOW_VALUE_AND_LOCATION
	//顯示漏光column位置與數值
	OSD_Color_Setup(OCN_RED, iTS);
	ScalableNumber((short) iTS->L_Smear_LB[0], 2, (S_IMGW >> 1) - 65, 85, iTS);
	ScalableNumber((short) iTS->L_Smear_RB[0], 2, (S_IMGW >> 1) - 65, 100, iTS);
	ScalableNumber((short) iTS->L_Smear_LB[1], 2, (S_IMGW >> 1) - 30, 85, iTS);
	ScalableNumber((short) iTS->L_Smear_RB[1], 2, (S_IMGW >> 1) - 30, 100, iTS);
	ScalableNumber((short) iTS->L_Smear_LB[2], 2, (S_IMGW >> 1) + 5, 85, iTS);
	ScalableNumber((short) iTS->L_Smear_RB[2], 2, (S_IMGW >> 1) + 5, 100, iTS);
	if(iTS->L_Smear_LB[0] != 0 && iTS->L_Smear_RB[0] != 0 && (iTS->L_Protection_Smear == SMEAR_SKIP_ON || iTS->L_Protection_Smear == SMEAR_ON))
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short) 0, 4, iTS->L_Smear_LB[0], 115, iTS);
	}
	if(iTS->L_Smear_LB[1] != 0 && iTS->L_Smear_RB[1] != 0 && (iTS->L_Protection_Smear == SMEAR_SKIP_ON || iTS->L_Protection_Smear == SMEAR_ON))
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short) 0, 4, iTS->L_Smear_LB[1], 115, iTS);
	}
	if(iTS->L_Smear_LB[2] != 0 && iTS->L_Smear_RB[2] != 0 && (iTS->L_Protection_Smear == SMEAR_SKIP_ON || iTS->L_Protection_Smear == SMEAR_ON))
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber((short) 0, 4, iTS->L_Smear_LB[2], 115, iTS);
	}
	//顯示切換成SMEAR_ON的有效距離
	if((iTS->L_Protection_Smear == SMEAR_SKIP_ON || iTS->L_Protection_Smear == SMEAR_ON) && iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > 10)
	{
		OSD_Color_Setup(OCN_GREEN, iTS);
		ScalableNumber((short) 0, 4, iTS->L_LaneMBound[row].Frt, 115, iTS);
		ScalableNumber((short) 0, 4, iTS->L_LaneMBound[row].Scd, 115, iTS);
		OSD_Color_Setup(OCN_BLUE, iTS);
		ScalableNumber((short) 0, 4, (iTS->L_Smear_LB[0] - distance), 115, iTS);
		ScalableNumber((short) 0, 4, (iTS->L_Smear_RB[0] + distance), 115, iTS);
	}
#endif    //L_SMEAR_SHOW_VALUE_AND_LOCATION
}
#endif    //L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF

#ifdef L_GROUND_REFLECTOR_PROTECTION_ON_OFF
void F_L_GroundReflector(ITS *iTS)
{
	short index_1 = 0;
	short start_row = (iTS->F_H - iTS->F_H_C) + L_VANISHING_POINT_DOWN_ROW;
	short start_row_bottom = iTS->F_H - start_row;
	short binary_row_start_col = 10;
	short binary_row_width = 320;
	short block_quantity = 0;
	short real_block_quantity = 0;
	short dual_lane_bias_flag = 0;

	//row_1跟以下row的距離
	short row_2 = L_ROW_1_2_DISTANCE;
	short row_3 = L_ROW_1_2_DISTANCE + L_ROW_2_3_DISTANCE;
	short row_4 = L_ROW_1_2_DISTANCE + L_ROW_2_3_DISTANCE + L_ROW_3_4_DISTANCE;
	short row_5 = L_ROW_1_2_DISTANCE + L_ROW_2_3_DISTANCE + L_ROW_3_4_DISTANCE + L_ROW_4_5_DISTANCE;

	short change_index_1 = 0;
	short first_col_1 = 0;
	short sec_col_1 = 0;
	short factor_col_1 = 0;
	short change_index_2 = 0;
	short first_col_2 = 0;
	short sec_col_2 = 0;
	short factor_col_2 = 0;
	short change_index_3 = 0;
	short first_col_3 = 0;
	short sec_col_3 = 0;
	short factor_col_3 = 0;
	short change_index_4 = 0;
	short first_col_4 = 0;
	short sec_col_4 = 0;
	short factor_col_4 = 0;

	short block_target_1 = 1;
	short block_target_2 = 2;
	short block_target_3 = 3;
	short block_target_4 = 4;
	short row_target_1 = 1;
	short row_target_2 = 2;
	short row_target_3 = 3;
	short row_target_4 = 4;

	int block_initial_1 = (iTS->F_W * (start_row - 1) + binary_row_start_col);
	int block_initial_2 = (iTS->F_W * (start_row + row_2 - 1) + binary_row_start_col);
	int block_initial_3 = (iTS->F_W * (start_row + row_3 - 1) + binary_row_start_col);
	int block_initial_4 = (iTS->F_W * (start_row + row_4 - 1) + binary_row_start_col);
	int block_initial_5 = (iTS->F_W * (start_row + row_5 - 1) + binary_row_start_col);

	//在程式初始時即沒有車道線可抓(diff_mean無法使用前一張的值)，
	//此時路面平均像素值為100，而路面車道線差值為0，因此將diff_mean初始化為155，
	//如此一來在此情況下的last_road_mean為255，保護機制不啟動
	static short diff_mean = 155;
	short last_road_mean = 0;
	short last_road_mean_half = 0;

	//二值化動態門檻值
	if(iTS->L_LeftLane_Difference_Value > iTS->L_RightLane_Difference_Value && (iTS->L_LeftLane_Difference_Value > 10 || iTS->L_RightLane_Difference_Value > 10))
		diff_mean = iTS->L_LeftLane_Difference_Value;
	else if(iTS->L_LeftLane_Difference_Value < iTS->L_RightLane_Difference_Value && (iTS->L_LeftLane_Difference_Value > 10 || iTS->L_RightLane_Difference_Value > 10))
		diff_mean = iTS->L_RightLane_Difference_Value;
	else
		diff_mean = diff_mean;
	if(iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean > iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean)
		last_road_mean = iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean;
	else
		last_road_mean = iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean;

	last_road_mean_half = last_road_mean + (diff_mean >> 1);
	last_road_mean += (diff_mean + L_RAISE_GRAYVALUE_TH);

	if(last_road_mean >= 255)
		last_road_mean = 255;
	else
		last_road_mean = last_road_mean;
	if(last_road_mean_half >= 255)
		last_road_mean_half = 255;
	else
		last_road_mean_half = last_road_mean_half;

	iTS->L_Check_High_Pixel_Quantity = 0;

#ifdef L_SHWO_SOBEL_EDGE
	F_L_Sobel(iTS);
#endif    //L_SHWO_SOBEL_EDGE

#ifdef L_SHOW_SOBEL_PROJECTION
	F_L_ReflectorEdge(iTS);
#endif    //L_SHOW_SOBEL_PROJECTION

	if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10 &&
	   iTS->L_LaneMBound[start_row_bottom].Frt < S_IMGCW && iTS->L_LaneMBound[start_row_bottom].Scd > S_IMGCW)
	{
		dual_lane_bias_flag = 1;
	}

 	F_L_DualToSingle(iTS);

	//(高速公路適用)連續1000frame都在trace模式下則...
	//F_L_DecisionStableOrUnstable(iTS);

#ifdef L_SHOW_GROUND_REFLECTOR_VALUE
	//顯示於右下方
	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber(last_road_mean, 3, (S_IMGW >> 1) + 30, 100, iTS);
	ScalableNumber(last_road_mean_half, 3, (S_IMGW >> 1) + 30, 120, iTS);
	//顯示於左上方
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber(dual_lane_bias_flag, 3, (S_IMGW >> 1) - 30, 70, iTS);
	//顯示於中上方
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber(iTS->L_Lane_Stable, 3, (S_IMGW >> 1) - 10, 70, iTS);
	//顯示於右上方
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber(iTS->L_Dual_to_Single_Flag, 3, (S_IMGW >> 1) + 10, 70, iTS);
	//顯示於正中間
	OSD_Color_Setup(OCN_YELLOW, iTS);
	ScalableNumber(iTS->L_SL_PointGroup.FindLaneCounter, 4, (S_IMGW >> 1) - 20, 100, iTS);

	//顯示於兩側
	/*if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10)
	{
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom].Frt, 3, 10, 60, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom - row_2].Frt, 3, 10, 80, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom - row_3].Frt, 3, 10, 100, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom - row_4].Frt, 3, 10, 120, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom - row_5].Frt, 3, 10, 140, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom].Scd, 3, 310, 60, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom - row_2].Scd, 3, 310, 80, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom - row_3].Scd, 3, 310, 100, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom - row_4].Scd, 3, 310, 120, iTS);
			ScalableNumber(iTS->L_LaneMBound[start_row_bottom - row_5].Scd, 3, 310, 140, iTS);
	}*/
#endif    //L_SHOW_GROUND_REFLECTOR_VALUE

	for(index_1 = 1; index_1 <= binary_row_width; index_1++)
	{

			//第一列row
			//條件1:第一列row上之像素值依門檻值二值化
			if(iTS->YImg[block_initial_1 + index_1] < last_road_mean)
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_1 + index_1] = 0;
#endif    //L_SHOW_BINARIZE_ROW

				if(first_col_1 != 0)
				{
					sec_col_1 = index_1;
					factor_col_1 = (short)first_col_1 + ((sec_col_1 - first_col_1) >> 1);

#ifdef L_SHOW_REFLECTOR_WIDTH
					OSD_Color_Setup(OCN_YELLOW, iTS);
					ScalableNumber(sec_col_1 - first_col_1, 2, first_col_1, start_row + 1, iTS);
#endif    //L_SHOW_REFLECTOR_WIDTH

					//條件2:第一列row上之反光部分限制其寬度
					if((sec_col_1 - first_col_1) <= L_BLOCK_WIDTH_LIMIT && (sec_col_1 - first_col_1) >= 4)
					{

						//條件3:雙車道(左右不超過中線)模式下兩側不偵測
						if((dual_lane_bias_flag == 1 && (factor_col_1 + binary_row_start_col) > iTS->L_LaneMBound[start_row_bottom].Frt &&
					   	   (factor_col_1 + binary_row_start_col) < iTS->L_LaneMBound[start_row_bottom].Scd) || dual_lane_bias_flag == 0)
						{

							//條件4:第一列row以下的row分別限制其寬度且針對相同的col位置做偵測
							if((iTS->YImg[block_initial_2 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_2, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
						   	   iTS->YImg[block_initial_3 + factor_col_1] <= last_road_mean)
							{
								block_quantity++;
								//符合以上4條件執行函式L_CountBlock,於函式中另有2個條件,若判斷為反光則傳回1
								if(F_L_CountBlock(start_row, factor_col_1 + binary_row_start_col, sec_col_1 - first_col_1, last_road_mean_half, block_target_1, row_target_1, iTS))
								{
									real_block_quantity++;
								}
							}
							else if((iTS->YImg[block_initial_2 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_2, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
							    	(iTS->YImg[block_initial_3 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
							    	iTS->YImg[block_initial_4 + factor_col_1] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row, factor_col_1 + binary_row_start_col, sec_col_1 - first_col_1, last_road_mean_half, block_target_2, row_target_1, iTS))
								{
									real_block_quantity++;
								}
							}
							else if((iTS->YImg[block_initial_2 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_2, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_3 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_4 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									iTS->YImg[block_initial_5 + factor_col_1] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row, factor_col_1 + binary_row_start_col, sec_col_1 - first_col_1, last_road_mean_half, block_target_3, row_target_1, iTS))
								{
									real_block_quantity++;
								}
							}
							else if((iTS->YImg[block_initial_2 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_2, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_3 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_4 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_5 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_5, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)))
							{
								block_quantity++;
								if(F_L_CountBlock(start_row, factor_col_1 + binary_row_start_col, sec_col_1 - first_col_1, last_road_mean_half, block_target_4, row_target_1, iTS))
								{
									real_block_quantity++;
								}
							}
						}
					}
				}
				change_index_1 = 0;
				first_col_1 = 0;
			}
			else
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_1 + index_1] = 255;
#endif    //L_SHOW_BINARIZE_ROW

				change_index_1++;
				if(change_index_1 == 1)
				{
					first_col_1 = index_1;
				}
				if(change_index_1 != 0 && index_1 == binary_row_width - 1)
				{
					sec_col_1 = index_1;
					factor_col_1 = (short)first_col_1 + ((sec_col_1 - first_col_1) >> 1);

#ifdef L_SHOW_REFLECTOR_WIDTH
					OSD_Color_Setup(OCN_YELLOW, iTS);
					ScalableNumber(sec_col_1 - first_col_1, 2, first_col_1, start_row + 1, iTS);
#endif    //L_SHOW_REFLECTOR_WIDTH

					if((sec_col_1 - first_col_1) <= L_BLOCK_WIDTH_LIMIT && (sec_col_1 - first_col_1) >= 4)
					{

						if((dual_lane_bias_flag == 1 && (factor_col_1 + binary_row_start_col) > iTS->L_LaneMBound[start_row_bottom].Frt &&
					   	   (factor_col_1 + binary_row_start_col) < iTS->L_LaneMBound[start_row_bottom].Scd) || dual_lane_bias_flag == 0)
						{

							if((iTS->YImg[block_initial_2 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_2, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
						   	   iTS->YImg[block_initial_3 + factor_col_1] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row, factor_col_1 + binary_row_start_col, sec_col_1 - first_col_1, last_road_mean_half, block_target_1, row_target_1, iTS))
								{
									real_block_quantity++;
								}
							}
							else if((iTS->YImg[block_initial_2 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_2, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
							    	(iTS->YImg[block_initial_3 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
							    	iTS->YImg[block_initial_4 + factor_col_1] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row, factor_col_1 + binary_row_start_col, sec_col_1 - first_col_1, last_road_mean_half, block_target_2, row_target_1, iTS))
								{
									real_block_quantity++;
								}
							}
							else if((iTS->YImg[block_initial_2 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_2, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_3 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_4 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									iTS->YImg[block_initial_5 + factor_col_1] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row, factor_col_1 + binary_row_start_col, sec_col_1 - first_col_1, last_road_mean_half, block_target_3, row_target_1, iTS))
								{
									real_block_quantity++;
								}
							}
							else if((iTS->YImg[block_initial_2 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_2, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_3 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_4 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_5 + factor_col_1] > last_road_mean && F_L_DecisionBondary(start_row + row_5, factor_col_1 + binary_row_start_col, last_road_mean, sec_col_1 - first_col_1, binary_row_start_col, binary_row_width, iTS)))
							{
								block_quantity++;
								if(F_L_CountBlock(start_row, factor_col_1 + binary_row_start_col, sec_col_1 - first_col_1, last_road_mean_half, block_target_4, row_target_1, iTS))
								{
									real_block_quantity++;
								}
							}
						}
					}
				}
			}
			//第二列row
			if(iTS->YImg[block_initial_2 + index_1] < last_road_mean)
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_2 + index_1] = 0;
#endif    //L_SHOW_BINARIZE_ROW

				if(first_col_2 != 0)
				{
					sec_col_2 = index_1;
					factor_col_2 = (short)first_col_2 + ((sec_col_2 - first_col_2) >> 1);

#ifdef L_SHOW_REFLECTOR_WIDTH
					OSD_Color_Setup(OCN_YELLOW, iTS);
					ScalableNumber(sec_col_2 - first_col_2, 2, first_col_2, start_row + row_2 + 1, iTS);
#endif    //L_SHOW_REFLECTOR_WIDTH

					if((sec_col_2 - first_col_2) <= L_BLOCK_WIDTH_LIMIT && (sec_col_2 - first_col_2) >= 4)
					{

						if((dual_lane_bias_flag == 1 && (factor_col_2 + binary_row_start_col) > iTS->L_LaneMBound[start_row_bottom - row_2].Frt &&
					   	   (factor_col_2 + binary_row_start_col) < iTS->L_LaneMBound[start_row_bottom - row_2].Scd) || dual_lane_bias_flag == 0)
						{

							if(iTS->YImg[block_initial_1 + factor_col_2] <= last_road_mean &&
						   	   (iTS->YImg[block_initial_3 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
						   	   iTS->YImg[block_initial_4 + factor_col_2] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_2, factor_col_2 + binary_row_start_col, sec_col_2 - first_col_2, last_road_mean_half, block_target_1, row_target_2, iTS))
								{
									real_block_quantity++;
								}
							}
							else if(iTS->YImg[block_initial_1 + factor_col_2] <= last_road_mean &&
						 			(iTS->YImg[block_initial_3 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
						  			(iTS->YImg[block_initial_4 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
						   			iTS->YImg[block_initial_5 + factor_col_2] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_2, factor_col_2 + binary_row_start_col, sec_col_2 - first_col_2, last_road_mean_half, block_target_2, row_target_2, iTS))
								{
									real_block_quantity++;
								}
							}
							else if(iTS->YImg[block_initial_1 + factor_col_2] <= last_road_mean &&
									(iTS->YImg[block_initial_3 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_4 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_5 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_5, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)))
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_2, factor_col_2 + binary_row_start_col, sec_col_2 - first_col_2, last_road_mean_half, block_target_3, row_target_2, iTS))
								{
									real_block_quantity++;
								}
							}
						}
					}
				}
				change_index_2 = 0;
				first_col_2 = 0;
			}
			else
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_2 + index_1] = 255;
#endif    //L_SHOW_BINARIZE_ROW

				change_index_2++;
				if(change_index_2 == 1)
				{
					first_col_2 = index_1;
				}
				if(change_index_2 != 0 && index_1 == binary_row_width - 1)
				{
					sec_col_2 = index_1;
					factor_col_2 = (short)first_col_2 + ((sec_col_2 - first_col_2) >> 1);

#ifdef L_SHOW_REFLECTOR_WIDTH
					OSD_Color_Setup(OCN_YELLOW, iTS);
					ScalableNumber(sec_col_2 - first_col_2, 2, first_col_2, start_row + row_2 + 1, iTS);
#endif    //L_SHOW_REFLECTOR_WIDTH

					if((sec_col_2 - first_col_2) <= L_BLOCK_WIDTH_LIMIT && (sec_col_3 - first_col_3) >= 4)
					{

						if((dual_lane_bias_flag == 1 && (factor_col_2 + binary_row_start_col) > iTS->L_LaneMBound[start_row_bottom - row_2].Frt &&
					   	   (factor_col_2 + binary_row_start_col) < iTS->L_LaneMBound[start_row_bottom - row_2].Scd) || dual_lane_bias_flag == 0)
						{

							if(iTS->YImg[block_initial_1 + factor_col_2] <= last_road_mean &&
						   	   (iTS->YImg[block_initial_3 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
						   	   iTS->YImg[block_initial_4 + factor_col_2] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_2, factor_col_2 + binary_row_start_col, sec_col_2 - first_col_2, last_road_mean_half, block_target_1, row_target_2, iTS))
								{
									real_block_quantity++;
								}
							}
							else if(iTS->YImg[block_initial_1 + factor_col_2] <= last_road_mean &&
						 			(iTS->YImg[block_initial_3 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
						  			(iTS->YImg[block_initial_4 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
						   			iTS->YImg[block_initial_5 + factor_col_2] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_2, factor_col_2 + binary_row_start_col, sec_col_2 - first_col_2, last_road_mean_half, block_target_2, row_target_2, iTS))
								{
									real_block_quantity++;
								}
							}
							else if(iTS->YImg[block_initial_1 + factor_col_2] <= last_road_mean &&
									(iTS->YImg[block_initial_3 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_3, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_4 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)) &&
									(iTS->YImg[block_initial_5 + factor_col_2] > last_road_mean && F_L_DecisionBondary(start_row + row_5, factor_col_2 + binary_row_start_col, last_road_mean, sec_col_2 - first_col_2, binary_row_start_col, binary_row_width, iTS)))
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_2, factor_col_2 + binary_row_start_col, sec_col_2 - first_col_2, last_road_mean_half, block_target_3, row_target_2, iTS))
								{
									real_block_quantity++;
								}
							}
						}
					}
				}
			}
			//第三列row
			if(iTS->YImg[block_initial_3 + index_1] < last_road_mean)
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_3 + index_1] = 0;
#endif    //L_SHOW_BINARIZE_ROW

				if(first_col_3 != 0)
				{
					sec_col_3 = index_1;
					factor_col_3 = (short)first_col_3 + ((sec_col_3 - first_col_3) >> 1);

#ifdef L_SHOW_REFLECTOR_WIDTH
					OSD_Color_Setup(OCN_YELLOW, iTS);
					ScalableNumber(sec_col_3 - first_col_3, 2, first_col_3, start_row + row_3 + 1, iTS);
#endif    //L_SHOW_REFLECTOR_WIDTH

					if((sec_col_3 - first_col_3) <= L_BLOCK_WIDTH_LIMIT && (sec_col_3 - first_col_3) >= 4)
					{

						if((dual_lane_bias_flag == 1 && (factor_col_3 + binary_row_start_col) > iTS->L_LaneMBound[start_row_bottom - row_3].Frt &&
					   	   (factor_col_3 + binary_row_start_col) < iTS->L_LaneMBound[start_row_bottom - row_3].Scd) || dual_lane_bias_flag == 0)
						{

							if(iTS->YImg[block_initial_1 + factor_col_3] <= last_road_mean &&
						   	   iTS->YImg[block_initial_2 + factor_col_3] <= last_road_mean &&
						   	   (iTS->YImg[block_initial_4 + factor_col_3] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_3 + binary_row_start_col, last_road_mean, sec_col_3 - first_col_3, binary_row_start_col, binary_row_width, iTS)) &&
						   	   iTS->YImg[block_initial_5 + factor_col_3] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_3, factor_col_3 + binary_row_start_col, sec_col_3 - first_col_3, last_road_mean_half, block_target_1, row_target_3, iTS))
								{
									real_block_quantity++;
								}
							}
							else if(iTS->YImg[block_initial_1 + factor_col_3] <= last_road_mean &&
							 		iTS->YImg[block_initial_2 + factor_col_3] <= last_road_mean &&
							    	(iTS->YImg[block_initial_4 + factor_col_3] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_3 + binary_row_start_col, last_road_mean, sec_col_3 - first_col_3, binary_row_start_col, binary_row_width, iTS)) &&
							    	(iTS->YImg[block_initial_5 + factor_col_3] > last_road_mean && F_L_DecisionBondary(start_row + row_5, factor_col_3 + binary_row_start_col, last_road_mean, sec_col_3 - first_col_3, binary_row_start_col, binary_row_width, iTS)))
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_3, factor_col_3 + binary_row_start_col, sec_col_3 - first_col_3, last_road_mean_half, block_target_2, row_target_3, iTS))
								{
									real_block_quantity++;
								}
							}
						}
					}
				}
				change_index_3 = 0;
				first_col_3 = 0;
			}
			else
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_3 + index_1] = 255;
#endif    //L_SHOW_BINARIZE_ROW

				change_index_3++;
				if(change_index_3 == 1)
				{
					first_col_3 = index_1;
				}
				if(change_index_3 != 0 && index_1 == binary_row_width - 1)
				{
					sec_col_3 = index_1;
					factor_col_3 = (short)first_col_3 + ((sec_col_3 - first_col_3) >> 1);

#ifdef L_SHOW_REFLECTOR_WIDTH
					OSD_Color_Setup(OCN_YELLOW, iTS);
					ScalableNumber(sec_col_3 - first_col_3, 2, first_col_3, start_row + row_3 + 1, iTS);
#endif    //L_SHOW_REFLECTOR_WIDTH

					if((sec_col_3 - first_col_3) <= L_BLOCK_WIDTH_LIMIT && (sec_col_3 - first_col_3) >= 4)
					{

						if((dual_lane_bias_flag == 1 && (factor_col_3 + binary_row_start_col) > iTS->L_LaneMBound[start_row_bottom - row_3].Frt &&
					   	   (factor_col_3 + binary_row_start_col) < iTS->L_LaneMBound[start_row_bottom - row_3].Scd) || dual_lane_bias_flag == 0)
						{

							if(iTS->YImg[block_initial_1 + factor_col_3] <= last_road_mean &&
						   	   iTS->YImg[block_initial_2 + factor_col_3] <= last_road_mean &&
						   	   (iTS->YImg[block_initial_4 + factor_col_3] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_3 + binary_row_start_col, last_road_mean, sec_col_3 - first_col_3, binary_row_start_col, binary_row_width, iTS)) &&
						   	   iTS->YImg[block_initial_5 + factor_col_3] <= last_road_mean)
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_3, factor_col_3 + binary_row_start_col, sec_col_3 - first_col_3, last_road_mean_half, block_target_1, row_target_3, iTS))
								{
									real_block_quantity++;
								}
							}
							else if(iTS->YImg[block_initial_1 + factor_col_3] <= last_road_mean &&
							 		iTS->YImg[block_initial_2 + factor_col_3] <= last_road_mean &&
							    	(iTS->YImg[block_initial_4 + factor_col_3] > last_road_mean && F_L_DecisionBondary(start_row + row_4, factor_col_3 + binary_row_start_col, last_road_mean, sec_col_3 - first_col_3, binary_row_start_col, binary_row_width, iTS)) &&
							    	(iTS->YImg[block_initial_5 + factor_col_3] > last_road_mean && F_L_DecisionBondary(start_row + row_5, factor_col_3 + binary_row_start_col, last_road_mean, sec_col_3 - first_col_3, binary_row_start_col, binary_row_width, iTS)))
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_3, factor_col_3 + binary_row_start_col, sec_col_3 - first_col_3, last_road_mean_half, block_target_2, row_target_3, iTS))
								{
									real_block_quantity++;
								}
							}
						}
					}
				}
			}
			//第四列row
			if(iTS->YImg[block_initial_4 + index_1] < last_road_mean)
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_4 + index_1] = 0;
#endif    //L_SHOW_BINARIZE_ROW

				if(first_col_4 != 0)
				{
					sec_col_4 = index_1;
					factor_col_4 = (short)first_col_4 + ((sec_col_4 - first_col_4) >> 1);

#ifdef L_SHOW_REFLECTOR_WIDTH
					OSD_Color_Setup(OCN_YELLOW, iTS);
					ScalableNumber(sec_col_4 - first_col_4, 2, first_col_4, start_row + row_4 + 1, iTS);
#endif    //L_SHOW_REFLECTOR_WIDTH

					if((sec_col_4 - first_col_4) <= L_BLOCK_WIDTH_LIMIT && (sec_col_4 - first_col_4) >= 4)
					{

						if((dual_lane_bias_flag == 1 && (factor_col_4 + binary_row_start_col) > iTS->L_LaneMBound[start_row_bottom - row_4].Frt &&
					   	   (factor_col_4 + binary_row_start_col) < iTS->L_LaneMBound[start_row_bottom - row_4].Scd) || dual_lane_bias_flag == 0)
						{

							if(iTS->YImg[block_initial_1 + factor_col_4] <= last_road_mean &&
						   	   iTS->YImg[block_initial_2 + factor_col_4] <= last_road_mean &&
						       iTS->YImg[block_initial_3 + factor_col_4] <= last_road_mean &&
						       (iTS->YImg[block_initial_5 + factor_col_4] > last_road_mean && F_L_DecisionBondary(start_row + row_5, factor_col_4 + binary_row_start_col, last_road_mean, sec_col_4 - first_col_4, binary_row_start_col, binary_row_width, iTS)))
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_4, factor_col_4 + binary_row_start_col, sec_col_4 - first_col_4, last_road_mean_half, block_target_1, row_target_4, iTS))
								{
									real_block_quantity++;
								}
							}
						}
					}
				}
				change_index_4 = 0;
				first_col_4 = 0;
			}
			else
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_4 + index_1] = 255;
#endif    //L_SHOW_BINARIZE_ROW

				change_index_4++;
				if(change_index_4 == 1)
				{
					first_col_4 = index_1;
				}
				if(change_index_4 != 0 && index_1 == binary_row_width - 1)
				{
					sec_col_4 = index_1;
					factor_col_4 = (short)first_col_4 + ((sec_col_4 - first_col_4) >> 1);

#ifdef L_SHOW_REFLECTOR_WIDTH
					OSD_Color_Setup(OCN_YELLOW, iTS);
					ScalableNumber(sec_col_4 - first_col_4, 2, first_col_4, start_row + row_4 + 1, iTS);
#endif    //L_SHOW_REFLECTOR_WIDTH

					if((sec_col_4 - first_col_4) <= L_BLOCK_WIDTH_LIMIT && (sec_col_4 - first_col_4) >= 4)
					{

						if((dual_lane_bias_flag == 1 && (factor_col_4 + binary_row_start_col) > iTS->L_LaneMBound[start_row_bottom - row_4].Frt &&
					   	   (factor_col_4 + binary_row_start_col) < iTS->L_LaneMBound[start_row_bottom - row_4].Scd) || dual_lane_bias_flag == 0)
						{

							if(iTS->YImg[block_initial_1 + factor_col_4] <= last_road_mean &&
						   	   iTS->YImg[block_initial_2 + factor_col_4] <= last_road_mean &&
						       iTS->YImg[block_initial_3 + factor_col_4] <= last_road_mean &&
						       (iTS->YImg[block_initial_5 + factor_col_4] > last_road_mean && F_L_DecisionBondary(start_row + row_5, factor_col_4 + binary_row_start_col, last_road_mean, sec_col_4 - first_col_4, binary_row_start_col, binary_row_width, iTS)))
							{
								block_quantity++;
								if(F_L_CountBlock(start_row + row_4, factor_col_4 + binary_row_start_col, sec_col_4 - first_col_4, last_road_mean_half, block_target_1, row_target_4, iTS))
								{
									real_block_quantity++;
								}
							}
						}
					}
				}
			}
			//第五列row
			if(iTS->YImg[block_initial_5 + index_1] < last_road_mean)
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_5 + index_1] = 0;
#endif    //L_SHOW_BINARIZE_ROW

			}
			else
			{

#ifdef L_SHOW_BINARIZE_ROW
				iTS->Showimage[block_initial_5 + index_1] = 255;
#endif    //L_SHOW_BINARIZE_ROW

			}
	}

#ifdef L_SHOW_GROUND_REFLECTOR_VALUE
	//顯示於左下方
	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber(block_quantity, 3, (S_IMGW >> 1) - 50, 100, iTS);
	ScalableNumber(real_block_quantity, 3, (S_IMGW >> 1) - 50, 120, iTS);
#endif    //L_SHOW_GROUND_REFLECTOR_VALUE

	/*if(block_quantity > 0)
	{

		if(iTS->L_Lane_Stable == 1)
		{
			iTS->GroundReflector_Ctr = iTS->GroundReflector_Ctr;
		}
		else if((iTS->L_Dual_to_Single_Flag == 1 && block_quantity == 1))
		{
			iTS->GroundReflector_Ctr = LimitCount(iTS->GroundReflector_Ctr += 3, 30);
		}
		else
		{
			iTS->GroundReflector_Ctr = LimitCount(iTS->GroundReflector_Ctr += 5, 30);
		}

	}
	else
	{
		iTS->GroundReflector_Ctr = LimitCount(-- iTS->GroundReflector_Ctr, 30);
	}*/

	if(iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > 10 && iTS->L_SL_PointGroup.FindLaneCounter > 100)
	{
		iTS->GroundReflector_Ctr = iTS->GroundReflector_Ctr;
	}
	else
	{
		if(real_block_quantity > 0 && block_quantity > 1)
		{
			iTS->GroundReflector_Ctr = (short)LimitCount((char)(iTS->GroundReflector_Ctr += 5), 30);
		}
		else if(real_block_quantity == 1)
		{
			if(iTS->L_Dual_to_Single_Flag == 1 || (iTS->L_Check_High_Pixel_Quantity == 1 && iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10))
			{
				iTS->GroundReflector_Ctr = iTS->GroundReflector_Ctr;
			}
			else
			{
				iTS->GroundReflector_Ctr = (short)LimitCount((char)(iTS->GroundReflector_Ctr += 3), 30);
			}
		}
		else
		{
			iTS->GroundReflector_Ctr = (short)LimitCount((char)(-- iTS->GroundReflector_Ctr), 30);
		}
	}
}
short F_L_DecisionBondary(short start_row, short start_col_middle, short last_road_mean, short col_width, short binary_row_start_col, short binary_row_width, ITS *iTS)
{
	short index_1 = 1;
	short index_2 = 1;
	short counter = 1;
	short flag_1 = 0;
	short flag_2 = 0;
	short max_width = 0;
	int block_initial = (iTS->F_W * (start_row - 1) + start_col_middle);

	while(flag_1 == 0 || flag_2 == 0)
	{

		if(flag_1 == 0)
		{

			if(start_col_middle + index_1 < binary_row_start_col + binary_row_width)
			{

				if(iTS->YImg[block_initial + index_1] > last_road_mean)
				{
					counter++;
					index_1++;
				}
				else
				{
					flag_1 = 1;
				}

			}
			else
			{
				flag_1 = 1;
			}

		}

		if(flag_2 == 0)
		{

			if(start_col_middle - index_2 > binary_row_start_col)
			{

				if(iTS->YImg[block_initial - index_2] > last_road_mean)
				{
					counter++;
					index_2++;
				}
				else
				{
					flag_2 = 1;
				}

			}
			else
			{
				flag_2 = 1;
			}

		}

	}

	if(counter > L_BLOCK_WIDTH_LIMIT)
	{
		return 0;
	}
	else
		//return 1;
	{

		if(counter > col_width)
		{
			max_width = counter;
		}
		else
		{
			max_width = col_width;
		}

		if(abs(counter - col_width) > ((max_width * 3) >> 2))
		{
			return 0;
		}
		else
		{
			return 1;
		}

	}

}
short F_L_CountBlock(short start_row, short start_col, short col_width, short last_road_mean_half, short block_target, short row_target, ITS *iTS)
{
	int block_initial = (iTS->F_W * (start_row - 1) + (start_col - (col_width >> 2)));
	short block_width = (col_width >> 1);
	short index_1 = 0;
	short index_2 = 0;
	short block_height_1 = 0;
	short block_height_2 = 0;
	short block_height_3 = 0;
	short block_height_4 = 0;
	unsigned int average_count = 0;
	short pixel_count = 0;
	short average = 0;

	if(row_target == 1)
	{
		block_height_1 = L_ROW_1_2_DISTANCE;
		block_height_2 = L_ROW_1_2_DISTANCE + L_ROW_2_3_DISTANCE;
		block_height_3 = L_ROW_1_2_DISTANCE + L_ROW_2_3_DISTANCE + L_ROW_3_4_DISTANCE;
		block_height_4 = L_ROW_1_2_DISTANCE + L_ROW_2_3_DISTANCE + L_ROW_3_4_DISTANCE + L_ROW_4_5_DISTANCE;
	}
	else if(row_target == 2)
	{
		block_height_1 = L_ROW_2_3_DISTANCE;
		block_height_2 = L_ROW_2_3_DISTANCE + L_ROW_3_4_DISTANCE;
		block_height_3 = L_ROW_2_3_DISTANCE + L_ROW_3_4_DISTANCE + L_ROW_4_5_DISTANCE;
	}
	else if(row_target == 3)
	{
		block_height_1 = L_ROW_3_4_DISTANCE;
		block_height_2 = L_ROW_3_4_DISTANCE + L_ROW_4_5_DISTANCE;
	}
	else if(row_target == 4)
	{
		block_height_1 = L_ROW_4_5_DISTANCE;
	}

	iTS->L_Edge_Quantity = 0;
	iTS->L_Pixel_Quantity_in_Block = 0;
	iTS->L_Over_TH_Pixel_Quantity = 0;

	switch(block_target)
	{
		case 1:

			for (index_1 = start_row; index_1 < start_row + block_height_1 + 1; index_1++)
			{

				for(index_2 = 0; index_2 < block_width + 1; index_2++)
				{

					if(index_1 == start_row || index_1 == start_row + block_height_1 || index_2 == 0 || index_2 == block_width)
					{

#ifdef L_SHOW_DETECTION_BLOCK
						iTS->Showimage[block_initial + index_2] = 0;
#endif    //L_SHOW_DETECTION_BLOCK

					}
					else
					{

						average_count += iTS->YImg[block_initial + index_2];
						pixel_count++;

						//計算區塊內超過像素門檻的比例
						if(iTS->YImg[block_initial + index_2] > (last_road_mean_half - L_MINUS_GRAYVALUE_TH_IN_BLOCK))
						{
							iTS->L_Over_TH_Pixel_Quantity++;
						}

						//計算區塊內經過Sobel後有被切出的比例
						if((iTS->L_ColProjection[block_initial + index_2] & 0x01) == 0x01)
						{
							iTS->L_Edge_Quantity++;
						}

					}

				}

				block_initial += iTS->F_W;
			}
			iTS->L_Pixel_Quantity_in_Block = (block_height_1 * block_width - block_height_1 - block_width);

			if(average_count == 0 || pixel_count == 0)
			{
				average = 0;
			}
			else
			{
				average = average_count / pixel_count;
			}

#ifdef L_SHOW_DETECTION_BLOCK
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber(iTS->L_Edge_Quantity, 2, start_col, start_row - 25, iTS);
			ScalableNumber(iTS->L_Pixel_Quantity_in_Block, 2, start_col, start_row - 40, iTS);
			ScalableNumber(iTS->L_Over_TH_Pixel_Quantity, 2, start_col, start_row - 55, iTS);
			ScalableNumber(average, 2, start_col, start_row - 70, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

			if(average > 230)
				iTS->L_Check_High_Pixel_Quantity++;

			//條件5:區塊內像素值大於門檻的比例超過一定門檻
			//條件6:區塊內被切出的垂直edge之比例超過一定門檻
			if((iTS->L_Over_TH_Pixel_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_PIXEL_PERCENT) &&
			   iTS->L_Edge_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_EDGE_PERCENT)) ||
			   average > 230)
			{

#ifdef L_SHOW_DETECTION_BLOCK
				ScalableNumber(1, 2, start_col, start_row - 10, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

				return 1;
			}
			else
			{

#ifdef L_SHOW_DETECTION_BLOCK
				ScalableNumber(0, 2, start_col, start_row - 10, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

				return 0;
			}

			//break;
		case 2:

			for (index_1 = start_row; index_1 < start_row + block_height_2 + 1; index_1++)
			{

				for(index_2 = 0; index_2 < block_width + 1; index_2++)
				{

					if(index_1 == start_row || index_1 == start_row + block_height_2 || index_2 == 0 || index_2 == block_width)
					{

#ifdef L_SHOW_DETECTION_BLOCK
						iTS->Showimage[block_initial + index_2] = 0;
#endif    //L_SHOW_DETECTION_BLOCK

					}
					else
					{

						average_count += iTS->YImg[block_initial + index_2];
						pixel_count++;

						if(iTS->YImg[block_initial + index_2] > (last_road_mean_half - L_MINUS_GRAYVALUE_TH_IN_BLOCK))
						{
							iTS->L_Over_TH_Pixel_Quantity++;
						}

						if((iTS->L_ColProjection[block_initial + index_2] & 0x01) == 0x01)
						{
							iTS->L_Edge_Quantity++;
						}

					}

				}

				block_initial += iTS->F_W;
			}
			iTS->L_Pixel_Quantity_in_Block = (block_height_2 * block_width - block_height_2 - block_width);

			if(average_count == 0 || pixel_count == 0)
			{
				average = 0;
			}
			else
			{
				average = average_count / pixel_count;
			}

#ifdef L_SHOW_DETECTION_BLOCK
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber(iTS->L_Edge_Quantity, 2, start_col, start_row - 25, iTS);
			ScalableNumber(iTS->L_Pixel_Quantity_in_Block, 2, start_col, start_row - 40, iTS);
			ScalableNumber(iTS->L_Over_TH_Pixel_Quantity, 2, start_col, start_row - 55, iTS);
			ScalableNumber(average, 2, start_col, start_row - 70, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

			if(average > 230)
				iTS->L_Check_High_Pixel_Quantity++;

			if((iTS->L_Over_TH_Pixel_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_PIXEL_PERCENT) &&
			   iTS->L_Edge_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_EDGE_PERCENT)) ||
			   average > 230)
			{

#ifdef L_SHOW_DETECTION_BLOCK
				ScalableNumber(1, 2, start_col, start_row - 10, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

				return 1;
			}
			else
			{

#ifdef L_SHOW_DETECTION_BLOCK
				ScalableNumber(0, 2, start_col, start_row - 10, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

				return 0;
			}

			//break;
		case 3:

			for (index_1 = start_row; index_1 < start_row + block_height_3 + 1; index_1++)
			{

				for(index_2 = 0; index_2 < block_width + 1; index_2++)
				{

					if(index_1 == start_row || index_1 == start_row + block_height_3 || index_2 == 0 || index_2 == block_width)
					{

#ifdef L_SHOW_DETECTION_BLOCK
						iTS->Showimage[block_initial + index_2] = 0;
#endif    //L_SHOW_DETECTION_BLOCK

					}
					else
					{

						average_count += iTS->YImg[block_initial + index_2];
						pixel_count++;

						if(iTS->YImg[block_initial + index_2] > (last_road_mean_half - L_MINUS_GRAYVALUE_TH_IN_BLOCK))
						{
							iTS->L_Over_TH_Pixel_Quantity++;
						}

						if((iTS->L_ColProjection[block_initial + index_2] & 0x01) == 0x01)
						{
							iTS->L_Edge_Quantity++;
						}

					}

				}

				block_initial += iTS->F_W;
			}
			iTS->L_Pixel_Quantity_in_Block = (block_height_3 * block_width - block_height_3 - block_width);

			if(average_count == 0 || pixel_count == 0)
			{
				average = 0;
			}
			else
			{
				average = average_count / pixel_count;
			}

#ifdef L_SHOW_DETECTION_BLOCK
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber(iTS->L_Edge_Quantity, 2, start_col, start_row - 25, iTS);
			ScalableNumber(iTS->L_Pixel_Quantity_in_Block, 2, start_col, start_row - 40, iTS);
			ScalableNumber(iTS->L_Over_TH_Pixel_Quantity, 2, start_col, start_row - 55, iTS);
			ScalableNumber(average, 2, start_col, start_row - 70, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

			if(average > 230)
				iTS->L_Check_High_Pixel_Quantity++;

			if((iTS->L_Over_TH_Pixel_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_PIXEL_PERCENT) &&
			   iTS->L_Edge_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_EDGE_PERCENT)) ||
			   average > 230)
			{

#ifdef L_SHOW_DETECTION_BLOCK
				ScalableNumber(1, 2, start_col, start_row - 10, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

				return 1;
			}
			else
			{

#ifdef L_SHOW_DETECTION_BLOCK
				ScalableNumber(0, 2, start_col, start_row - 10, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

				return 0;
			}

			//break;
		case 4:

			for (index_1 = start_row; index_1 < start_row + block_height_4 + 1; index_1++)
			{

				for(index_2 = 0; index_2 < block_width + 1; index_2++)
				{

					if(index_1 == start_row || index_1 == start_row + block_height_4 || index_2 == 0 || index_2 == block_width)
					{

#ifdef L_SHOW_DETECTION_BLOCK
						iTS->Showimage[block_initial + index_2] = 0;
#endif    //L_SHOW_DETECTION_BLOCK

					}
					else
					{

						average_count += iTS->YImg[block_initial + index_2];
						pixel_count++;

						if(iTS->YImg[block_initial + index_2] > (last_road_mean_half - L_MINUS_GRAYVALUE_TH_IN_BLOCK))
						{
							iTS->L_Over_TH_Pixel_Quantity++;
						}

						if((iTS->L_ColProjection[block_initial + index_2] & 0x01) == 0x01)
						{
							iTS->L_Edge_Quantity++;
						}

					}

				}

				block_initial += iTS->F_W;
			}
			iTS->L_Pixel_Quantity_in_Block = (block_height_4 * block_width - block_height_4 - block_width);

			if(average_count == 0 || pixel_count == 0)
			{
				average = 0;
			}
			else
			{
				average = average_count / pixel_count;
			}

#ifdef L_SHOW_DETECTION_BLOCK
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber(iTS->L_Edge_Quantity, 2, start_col, start_row - 25, iTS);
			ScalableNumber(iTS->L_Pixel_Quantity_in_Block, 2, start_col, start_row - 40, iTS);
			ScalableNumber(iTS->L_Over_TH_Pixel_Quantity, 2, start_col, start_row - 55, iTS);
			ScalableNumber(average, 2, start_col, start_row - 70, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

			if(average > 230)
				iTS->L_Check_High_Pixel_Quantity++;

			if((iTS->L_Over_TH_Pixel_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_PIXEL_PERCENT) &&
			   iTS->L_Edge_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_EDGE_PERCENT)) ||
			   average > 230)
			{

#ifdef L_SHOW_DETECTION_BLOCK
				ScalableNumber(1, 2, start_col, start_row - 10, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

				return 1;
			}
			else
			{

#ifdef L_SHOW_DETECTION_BLOCK
				ScalableNumber(0, 2, start_col, start_row - 10, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

				return 0;
			}

			//break;
		default:
			return 0;
	}
}
void F_L_DecisionStableOrUnstable(ITS *iTS)
{
	static short StableFrame_Ctr = 0;

	if((iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 0) || (iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > 0))
	{
		StableFrame_Ctr++;

		if(StableFrame_Ctr > L_STABLE_FRAME)
		{
			StableFrame_Ctr = L_STABLE_FRAME;
		}

	}
	else
	{
		StableFrame_Ctr = 0;
	}

	if(StableFrame_Ctr >= L_STABLE_FRAME)
	{
		iTS->L_Lane_Stable = 1;
	}
	else
	{
		iTS->L_Lane_Stable = 0;
	}

}
void F_L_DualToSingle(ITS *iTS)
{

	if(iTS->L_Last_Mode == LTRACE && iTS->L_DetectMode == SL_TRACE)
	{
		iTS->L_Dual_to_Single_Flag = 1;
	}
	else if(iTS->L_Dual_to_Single_Flag == 1 && iTS->L_DetectMode == SL_TRACE)
	{
		iTS->L_Dual_to_Single_Flag = 1;
	}
	else if(iTS->L_DetectMode != SL_TRACE)
	{
		iTS->L_Dual_to_Single_Flag = 0;
	}

	iTS->L_Last_Mode = iTS->L_DetectMode;
}
void F_L_DecisionGroundReflector(ITS *iTS)
{

	if(iTS->GroundReflector_Ctr == 30)
	{
		iTS->L_Protection_GroundReflector = GROUND_REFLECTOR_ON;
	}

	if(iTS->GroundReflector_Ctr == 0)
	{
		iTS->L_Protection_GroundReflector = GROUND_REFLECTOR_OFF;
	}

}

#ifdef L_SHWO_SOBEL_EDGE
void F_L_Sobel(ITS *iTS)
{
	short row = 0;
	short col = 0;
	int r = SinkDataIndexStart(S_IMGBB);
	unsigned char *L_Result, *L_Initial;

	for(row = S_IMGBB; row < S_IMGCH; row++)
	{
		r=SinkDataIndexNextRow(r);
		L_Result = &iTS->L_ColProjection[r];
		L_Initial = &iTS->Showimage[r];

		for(col = S_IMGLB; col < S_IMGRB; col++)
		{

			if((L_Result[col] & 0x01) == 0x01)
			{
				L_Initial[col] = 20;
			}
			else
			{
				L_Initial[col] = 0;
			}

		}

	}

}
#endif    //L_SHWO_SOBEL_EDGE

#ifdef L_SHOW_SOBEL_PROJECTION
void F_L_ReflectorEdge(ITS *iTS)
{
	short index_1 = 0;
	short row = 0;
	short col = 0;
	short start_row = iTS->F_H - S_IMGCH + 1;
	short height = S_IMGCH - S_IMGBB;
	short width = S_IMGRB - S_IMGLB;
	short VE_VP_Array[S_IMGRB - S_IMGLB + 1];
	int block_initial = (iTS->F_W * (start_row - 1) + S_IMGLB);

	for (index_1 = 0; index_1 < width; index_1++)
	{
		VE_VP_Array[index_1] = 0;
	}

	for(row = 0; row < height; row++)
	{

		for(col = 0; col < width; col++)
		{

			if((iTS->L_ColProjection[block_initial + col] & 0x02) == 0x02)
			{
				VE_VP_Array[col]++;
			}

		}

		block_initial += iTS->F_W;
	}

	F_L_ShowVEVP(iTS->Showimage, VE_VP_Array, iTS);
}
void F_L_ShowVEVP(unsigned char *Dst, short *VE_VP_Array, ITS *iTS)
{
	const unsigned char color = 0;
	short row = 0;
	short col = 0;
	short width = S_IMGRB - S_IMGLB;
	short height = (S_IMGCH - S_IMGBB) >> 1;

	for(col = 0; col < width; col++)
	{

		for(row = iTS->F_H - height; row < iTS->F_H; row++)
		{

			if((VE_VP_Array[col]) - (row-(iTS->F_H - height)) > 0)
			{
				Dst[(iTS->F_H-1-row) * iTS->F_W + S_IMGLB + col] = 255;
			}
			else
			{
				Dst[(iTS->F_H-1-row) * iTS->F_W + S_IMGLB + col] = color;
			}

		}

	}

}
#endif    //L_SHOW_SOBEL_PROJECTION

#endif    //L_GROUND_REFLECTOR_PROTECTION_ON_OFF

#ifdef L_WATER_STREAK_DETECTION_ON_OFF
void F_L_WaterStreakDetection(ITS *iTS)
{
	double K = 0;
	double M = 0;
	double Bl = 0;
	double Br = 0;
	double evRodSlp = 0;
	short left_line_col = 0;
	short right_line_col = 0;
	short max_lane_gray = 0;
	short cut_middle_width = 0;
	short left = 0;
	short right = 1;

	double Vrow = 0;
	short row_ = 0;

#ifdef L_SHOW_WATER_STREAK_VALUE
	unsigned char *CurRow;
#endif    //L_SHOW_WATER_STREAK_VALUE

	iTS->L_Block_Point = 0;
	iTS->L_Left_Block_Pixel_Total = 0;
	iTS->L_Right_Block_Pixel_Total = 0;
	iTS->L_Left_Block_Pixel_Average = 0;
	iTS->L_Right_Block_Pixel_Average = 0;
	iTS->L_Left_Block_Edge_Total = 0;
	iTS->L_Right_Block_Edge_Total = 0;
	iTS->L_Block_Position = (short)(S_IMGCH * 0.57);

	iTS->Skyline_V = S_IMGH - S_IMGCH;

#ifdef L_SHOW_WATER_STREAK_VALUE
	memset(&iTS->Showimage[iTS->F_W * iTS->Skyline_V], 255, iTS->F_W);
	memset(&iTS->Showimage[iTS->F_W * (S_IMGH - (S_IMGCH - 35))], 0, iTS->F_W);
	memset(&iTS->Showimage[iTS->F_W * (S_IMGH - (S_IMGCH - 95))], 0, iTS->F_W);
#endif    //L_SHOW_WATER_STREAK_VALUE

	/*if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10)
	{
		K = iTS->K;
		M = iTS->M;
		Bl = iTS->LaneL.B_i;
		Br = iTS->LaneR.B_i;
		evRodSlp = iTS->L_evRodSlp;
	}
	else
	{*/
		K = 0;
		M = 0;
		Bl = - (double)(L_RW_DefaultLW * S_euv) / (2 * S_HOCam);
		Br = + (double)(L_RW_DefaultLW * S_euv) / (2 * S_HOCam);
		evRodSlp = 0;
	//}

	if(iTS->L_Left_Lane_Pixel_Point == 0 || iTS->L_Right_Lane_Pixel_Point == 0)
	{
		max_lane_gray = 0;
	}
	else if((iTS->L_Left_Lane_Pixel_Total / iTS->L_Left_Lane_Pixel_Point) > (iTS->L_Right_Lane_Pixel_Total / iTS->L_Right_Lane_Pixel_Point))
		max_lane_gray = iTS->L_Left_Lane_Pixel_Total / iTS->L_Left_Lane_Pixel_Point;
	else
		max_lane_gray = iTS->L_Right_Lane_Pixel_Total / iTS->L_Right_Lane_Pixel_Point;

	for(row_ = 0; row_ < S_IMGCH; row_++)
	{
		Vrow = evRodSlp - (row_ - iTS->F_H_C);
		left_line_col = LimitW(iTS->F_W_C + LaneModel(Vrow, K, M, Bl));
		right_line_col = LimitW(iTS->F_W_C + LaneModel(Vrow, K, M, Br));

#ifdef L_SHOW_WATER_STREAK_VALUE
		CurRow = &iTS->Showimage[((iTS->F_H - 1) - row_) * iTS->F_W];
		CurRow[left_line_col] = 255;
		CurRow[right_line_col] = 255;
#endif    //L_SHOW_WATER_STREAK_VALUE

		if(row_ == iTS->L_Block_Position)
		{
			cut_middle_width = (abs(left_line_col - right_line_col)) / 3;
			F_L_WaterStreakDetectionBlock(S_IMGH - iTS->L_Block_Position, left_line_col, cut_middle_width, left, iTS);
			iTS->L_Block_Point = 0;
			F_L_WaterStreakDetectionBlock(S_IMGH - iTS->L_Block_Position, right_line_col - cut_middle_width, cut_middle_width, right, iTS);
		}
	}
	if(iTS->L_Left_Block_Pixel_Total == 0 || iTS->L_Block_Point == 0)
		iTS->L_Left_Block_Pixel_Average = 0;
	else
		iTS->L_Left_Block_Pixel_Average = iTS->L_Left_Block_Pixel_Total / iTS->L_Block_Point;
	if(iTS->L_Right_Block_Pixel_Total == 0 || iTS->L_Block_Point == 0)
		iTS->L_Right_Block_Pixel_Average = 0;
	else
		iTS->L_Right_Block_Pixel_Average = iTS->L_Right_Block_Pixel_Total / iTS->L_Block_Point;

#ifdef L_SHOW_WATER_STREAK_VALUE
	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber(iTS->L_Left_Block_Edge_Total, 3, 10, 10, iTS);
	ScalableNumber(iTS->L_Block_Point, 3, 10, 70, iTS);
	ScalableNumber(iTS->L_Right_Block_Edge_Total, 3, 50, 10, iTS);
	ScalableNumber(iTS->L_Block_Point, 3, 50, 70, iTS);
	ScalableNumber(iTS->L_Left_Block_Pixel_Average, 3, 90, 10, iTS);
	ScalableNumber(iTS->L_Right_Block_Pixel_Average, 3, 90, 70, iTS);
	ScalableNumber(iTS->L_Left_Lane_Pixel_Total / iTS->L_Left_Lane_Pixel_Point, 3, 130, 10, iTS);
	ScalableNumber(iTS->L_Right_Lane_Pixel_Total / iTS->L_Right_Lane_Pixel_Point, 3, 130, 70, iTS);
#endif    //L_SHOW_WATER_STREAK_VALUE

	if(iTS->L_DetectMode == LTRACE)
	{
		if((iTS->L_Left_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT) && (abs(max_lane_gray - iTS->L_Left_Block_Pixel_Average)) < L_PIXEL_DIFFERENT_TH && iTS->L_Right_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT_OTHER_SIDE)) ||
	   	   (iTS->L_Right_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT) && (abs(max_lane_gray - iTS->L_Right_Block_Pixel_Average)) < L_PIXEL_DIFFERENT_TH && iTS->L_Left_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT_OTHER_SIDE)))
		{
			if(iTS->L_Protection_WaterStreak == WATER_STREAK_OFF)
				iTS->L_Water_Streak_Ctr = (short)LimitCount((char)(iTS->L_Water_Streak_Ctr += 3), 30);
			else
				iTS->L_Water_Streak_Ctr = 30;
		}
		else
			iTS->L_Water_Streak_Ctr = (short)LimitCount((char)(-- iTS->L_Water_Streak_Ctr), 30);
	}
	else if(iTS->L_DetectMode == SL_TRACE)
	{
		if((iTS->L_BiasWarn > 0 && iTS->L_Right_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT) && (abs(max_lane_gray - iTS->L_Right_Block_Pixel_Average)) < L_PIXEL_DIFFERENT_TH) ||
		   (iTS->L_BiasWarn < 0 && iTS->L_Left_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT) && (abs(max_lane_gray - iTS->L_Left_Block_Pixel_Average)) < L_PIXEL_DIFFERENT_TH))
		{
				if(iTS->L_Protection_WaterStreak == WATER_STREAK_OFF)
					iTS->L_Water_Streak_Ctr = (short)LimitCount((char)(iTS->L_Water_Streak_Ctr += 3), 30);
				else
					iTS->L_Water_Streak_Ctr = 30;
		}
		else if(iTS->L_BiasWarn == 0)
		{
			if((iTS->L_Left_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT) && (abs(max_lane_gray - iTS->L_Left_Block_Pixel_Average)) < L_PIXEL_DIFFERENT_TH && iTS->L_Right_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT_OTHER_SIDE)) ||
	   	   	   (iTS->L_Right_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT) && (abs(max_lane_gray - iTS->L_Right_Block_Pixel_Average)) < L_PIXEL_DIFFERENT_TH && iTS->L_Left_Block_Edge_Total > (iTS->L_Block_Point * L_OVER_EDGE_TH_PERCENT_OTHER_SIDE)))
			{
				if(iTS->L_Protection_WaterStreak == WATER_STREAK_OFF)
					iTS->L_Water_Streak_Ctr = (short)LimitCount((char)(iTS->L_Water_Streak_Ctr += 3), 30);
				else
					iTS->L_Water_Streak_Ctr = 30;
			}
			else
				iTS->L_Water_Streak_Ctr = (short)LimitCount((char)(-- iTS->L_Water_Streak_Ctr), 30);
		}
		else
			iTS->L_Water_Streak_Ctr = (short)LimitCount((char)(-- iTS->L_Water_Streak_Ctr), 30);
	}
	else
		iTS->L_Water_Streak_Ctr = (short)LimitCount((char)(-- iTS->L_Water_Streak_Ctr), 30);

	if(iTS->L_Water_Streak_Ctr == 30)
		iTS->L_Protection_WaterStreak = WATER_STREAK_ON;
	else if(iTS->L_Water_Streak_Ctr == 0)
		iTS->L_Protection_WaterStreak = WATER_STREAK_OFF;
}
void F_L_WaterStreakDetectionBlock(short start_row, short start_col, short cut_middle_width, short left_or_right, ITS *iTS)
{
	short index_1 = 0;
	short index_2 = 0;

	short block_height = 3;
	short block_width = cut_middle_width;

	int block_initial=(iTS->F_W * (start_row - 1) + start_col);

	for (index_1 = start_row; index_1 < start_row + block_height + 1; index_1++)
	{
		for(index_2 = 0; index_2 < block_width + 1; index_2++)
		{
			iTS->L_Block_Point++;
			if(left_or_right == 0)
			{
				iTS->L_Left_Block_Pixel_Total += iTS->YImg[block_initial + index_2];
				if((iTS->L_ColProjection[block_initial + index_2] & 0x04) == 0x04)
					iTS->L_Left_Block_Edge_Total++;
			}
			else
			{
				iTS->L_Right_Block_Pixel_Total += iTS->YImg[block_initial + index_2];
				if((iTS->L_ColProjection[block_initial + index_2] & 0x04) == 0x04)
					iTS->L_Right_Block_Edge_Total++;
			}

#ifdef L_SHOW_WATER_STREAK_VALUE
			if(index_1 == start_row || index_1 == start_row + block_height || index_2 == 0 || index_2 == block_width)
			{
				iTS->Showimage[block_initial + index_2] = 255;
			}
#endif    //L_SHOW_WATER_STREAK_VALUE

		}
		block_initial += iTS->F_W;
	}
}
#endif    //L_WATER_STREAK_DETECTION_ON_OFF

#ifdef L_HORIZONTAL_MARKING_DETECTION_ON_OFF
void L_HorizontalMrakingDetection(ITS *iTS)
{
	short row = 0;
	short column = 0;
	char column_flag = 0;
	int r = 0;
	unsigned char *L_Result, *L_UpRow_Result, *L_GrayScale, *L_UpRow_GrayScale;

	short distance_V = (short)(iTS->F_H_C * L_MARKING_AREA_HEIGHT_RATE);
	short StartRow = iTS->F_H_C >> 1;
	short EndRow = StartRow + distance_V + distance_V;
	short StartColumn = iTS->F_W_C - (short)(iTS->F_W_C * L_MARKING_AREA_WIDTH_RATE);
	short EndColumn = iTS->F_W_C + (short)(iTS->F_W_C * L_MARKING_AREA_WIDTH_RATE);

	static short next_StartRow = 0;
	static char RoadMarking_decision_count = 0;
	static char RoadMarking_decision_flag = 0;
	static short TrackingMode_ContinuousCount = 0;

	short HorizontalEdge_L = 0;
	short HorizontalEdge_R = 0;
	short HorizontalEdge_RowCount = 0;
	short Marking[2] = {0, 0};
	short Marking_FlagCount = 0;
	short Marking_TotalCount = 0;
	short Marking_height = 0;

	short GrayScale_threshold = 0;
	short host_StartRow = StartRow;

	if((iTS->L_DetectMode == LTRACE && iTS->L_StbCtr >= 10) || (iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr >= 10))
	{
		GrayScale_threshold = ((iTS->LM_L_GrayScale_AdaptiveThresholding.LastRoadMean + iTS->LM_R_GrayScale_AdaptiveThresholding.LastRoadMean) >> 1) + L_MARKING_GRAY_THRESHOLD;

		if(TrackingMode_ContinuousCount < L_ADAPTIVE_THRESHOLD_FRAME_COUNT)
		{
			TrackingMode_ContinuousCount++;
		}
		else
		{
			TrackingMode_ContinuousCount = L_ADAPTIVE_THRESHOLD_FRAME_COUNT;
		}
	}
	else
	{
		GrayScale_threshold = LM_GRAY_ADP_LAST_ROAD_MEAN_TH_LEVEL_7 + L_MARKING_GRAY_THRESHOLD;
		TrackingMode_ContinuousCount = 0;
	}

	if(next_StartRow != 0 && next_StartRow >= S_IMGBB)
	{
		StartRow = next_StartRow;
	}

	r = SinkDataIndexStart(StartRow);

	for(row = StartRow; row <= EndRow; row++)
	{
		r = SinkDataIndexNextRow(r);
		L_Result = &iTS->L_ColProjection[r];
		L_UpRow_Result = &iTS->L_ColProjection[r - iTS->F_W];
		L_GrayScale = &iTS->YImg[r];
		L_UpRow_GrayScale = &iTS->YImg[r - iTS->F_W];

		HorizontalEdge_L = 0;
		HorizontalEdge_R = 0;
		HorizontalEdge_RowCount = 0;

		if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr >= 15 && iTS->L_Protection_Main == PROTECTION_OFF && iTS->L_UsingPerception_Main == USING_PERCEPTION_OFF)
		{
			StartColumn = iTS->L_LaneMBound[row].Frt - 10;
			EndColumn = iTS->L_LaneMBound[row].Scd + 10;
		}

		for(column = StartColumn; column <= EndColumn; column++)
		{
			if(((L_Result[column] & 0x08) == 0x08 && L_GrayScale[column] > GrayScale_threshold) ||
			   ((L_Result[column + 1] & 0x08) == 0x08 && L_GrayScale[column + 1] > GrayScale_threshold))
			{
				column_flag = 1;
				HorizontalEdge_RowCount++;

				if(Marking[0] != 0 && row <= (Marking[0] + L_MARKING_DENSITY_ROW_NUMBER))
				{
					Marking_FlagCount++;
				}

#ifdef L_MARKING_SHOW_VALUE_AND_EDGE
				iTS->Showimage[r + column] = 255;
#endif    //L_MARKING_SHOW_VALUE_AND_EDGE
			}
			else if(((L_UpRow_Result[column + 1] & 0x08) == 0x08 && L_UpRow_GrayScale[column + 1] > GrayScale_threshold) ||
			        ((L_UpRow_Result[column] & 0x08) == 0x08 && L_UpRow_GrayScale[column] > GrayScale_threshold) ||
			        ((L_UpRow_Result[column - 1] & 0x08) == 0x08 && L_UpRow_GrayScale[column - 1] > GrayScale_threshold))
			{
				column_flag = 1;
				HorizontalEdge_RowCount++;

				if(Marking[0] != 0 && row <= (Marking[0] + L_MARKING_DENSITY_ROW_NUMBER))
				{
					Marking_FlagCount++;
				}

#ifdef L_MARKING_SHOW_VALUE_AND_EDGE
				iTS->Showimage[r + column] = 128;
#endif    //L_MARKING_SHOW_VALUE_AND_EDGE
			}
			else
			{
				column_flag = 0;

#ifdef L_MARKING_SHOW_VALUE_AND_EDGE
				iTS->Showimage[r + column] = 0;
#endif    //L_MARKING_SHOW_VALUE_AND_EDGE
			}

			if(Marking[0] != 0 && row <= (Marking[0] + L_MARKING_DENSITY_ROW_NUMBER))
			{
				Marking_TotalCount++;
			}

			if(column == EndColumn)
			{
				if(HorizontalEdge_RowCount > ((EndColumn - StartColumn) * L_MARKING_ONE_ROW_EDGE_RATE))
				{
					if(Marking[0] == 0)
					{
						Marking[0] = row;
					}
					else if(Marking[0] != row)
					{
						Marking[1] = row;
					}
				}

				column_flag = 0;
			}

			if(	column_flag == 1)
			{
				if(HorizontalEdge_L == 0 || HorizontalEdge_R == 0)
				{
					HorizontalEdge_L = column;
					HorizontalEdge_R = column;
				}
				else
				{
					HorizontalEdge_R = column;
				}
			}
			else //column_flag == 0
			{
				if(HorizontalEdge_L != 0 && HorizontalEdge_R != 0)
				{
					if(abs(HorizontalEdge_R - HorizontalEdge_L) > ((EndColumn - StartColumn) * L_MARKING_ONE_ROW_EDGE_RATE))
					{
						if(Marking[0] == 0)
						{
							Marking[0] = row;
						}
						else if(Marking[0] != row)
						{
							Marking[1] = row;
						}
					}
				}

				HorizontalEdge_L = 0;
				HorizontalEdge_R = 0;
			} //end column_flag
		} //end column
	} //end row

	if(Marking[0] != 0 && Marking[0] < (host_StartRow + distance_V))
	{
		next_StartRow = Marking[0] - (short)(distance_V * 1.5);
	}
	else
	{
		next_StartRow = 0;
	}

	Marking_TotalCount = (short)(Marking_TotalCount * L_MARKING_MANY_ROW_EDGE_RATE);

	//decision
	if(Marking[0] == 0 && Marking[1] == 0)
	{
		RoadMarking_decision_flag = 0;
	}
	else
	{
		//Marking_height
		if(Marking[0] != 0 && Marking[1] != 0)
		{
			Marking_height = Marking[1] - Marking[0];
		}
		else
		{
			Marking_height = 0;
		}

		//RoadMarking_decision_flag
		if(Marking[0] > host_StartRow && Marking_height > L_MARKING_HEIGHT_THRESHOLD && Marking_height <= distance_V)
		{
			RoadMarking_decision_flag = 1;
		}

		if(RoadMarking_decision_flag != 1 && Marking_height > 0 && Marking_FlagCount >= Marking_TotalCount)
		{
			RoadMarking_decision_flag = 2;
		}
		else if(RoadMarking_decision_flag == 2 && Marking_FlagCount < Marking_TotalCount)
		{
			RoadMarking_decision_flag = 0;
		}
	}

	//RoadMarking_decision_count
	if(RoadMarking_decision_flag == 2 && Marking[0] <= host_StartRow && Marking_FlagCount >= Marking_TotalCount)
	{
		RoadMarking_decision_flag = 0;
		RoadMarking_decision_count = L_MARKING_COUNT_THRESHOLD;
	}
	else if(RoadMarking_decision_flag != 1 && Marking_height > distance_V && Marking[1] > host_StartRow)
	{
		RoadMarking_decision_count = LimitCount((RoadMarking_decision_count + 10), L_MARKING_COUNT_THRESHOLD);
	}
	else if(iTS->L_UsingPerception_RoadMarking == MARKING_ON && Marking[0] > distance_V && Marking_FlagCount >= Marking_TotalCount)
	{
		RoadMarking_decision_count = LimitCount(++RoadMarking_decision_count, L_MARKING_COUNT_THRESHOLD);
	}
	else
	{
		RoadMarking_decision_count = LimitCount(--RoadMarking_decision_count, L_MARKING_COUNT_THRESHOLD);
	}

	//iTS->L_UsingPerception_RoadMarking
	if(RoadMarking_decision_count == 0 || TrackingMode_ContinuousCount >= L_ADAPTIVE_THRESHOLD_FRAME_COUNT)
	{
		iTS->L_UsingPerception_RoadMarking = MARKING_OFF;
	}
	else if(RoadMarking_decision_count >= 5)
	{
		iTS->L_UsingPerception_RoadMarking = MARKING_ON;
	}

#ifdef L_MARKING_SHOW_VALUE_AND_EDGE
	if(Marking[0] != 0)
	{
		memset(&iTS->Showimage[iTS->F_W * (iTS->F_H - Marking[0])], 255, iTS->F_W);

		if(Marking[1] != 0)
		{
			memset(&iTS->Showimage[iTS->F_W * (iTS->F_H - Marking[1])], 255, iTS->F_W);
		}
	}

	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber(RoadMarking_decision_flag, 4, (S_IMGW >> 1), 65, iTS);
	ScalableNumber(Marking_height, 4, (S_IMGW >> 1) + 18, 65, iTS);

	OSD_Color_Setup(OCN_RED, iTS);
	ScalableNumber((short) Marking[0], 2, (S_IMGW >> 1) + 47, 64, iTS);
	ScalableNumber((short) Marking[1], 2, (S_IMGW >> 1) + 47, 78, iTS);
	ScalableNumber((short) Marking_TotalCount, 4, (S_IMGW >> 1) + 70, 65, iTS);
	ScalableNumber((short) Marking_FlagCount, 4, (S_IMGW >> 1) + 115, 65, iTS);
#endif    //L_MARKING_SHOW_VALUE_AND_EDGE

#ifdef L_SHOW_PROTECTION_WARNING_SIGN
	//顯示警示訊號
	if((iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 10) || (iTS->L_DetectMode == SL_TRACE && iTS->SL_LaneModel.L_StbCtr > 10))
	{
		L_ShowWarningForProtection(iTS->L_UsingPerception_RoadMarking, iTS);
	}
	else
	{
		L_ShowWarningForProtection(iTS->L_UsingPerception_RoadMarking, iTS);
		OSD_Color_Setup(OCN_WHITE, iTS);
		DrawRect_Has_UV(40, 94, 5, iTS);

	}

	OSD_Color_Setup(OCN_ORANGE, iTS);
	ScalableNumber((short) RoadMarking_decision_count, 2, 102, 30, iTS);

	if(TrackingMode_ContinuousCount >= L_ADAPTIVE_THRESHOLD_FRAME_COUNT)
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber(1, 2, 95, 30, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_RED, iTS);
		ScalableNumber(0, 2, 95, 30, iTS);
	}
#endif    //L_SHOW_PROTECTION_WARNING_SIGN
}
#endif    //L_HORIZONTAL_MARKING_DETECTION_ON_OFF

#endif    //L_PROTECTION_MAIN_ON_OFF


