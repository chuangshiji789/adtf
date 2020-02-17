#include "InitialVariable.h"
#include "FunctionType.h"

#ifdef L_PROTECTION_MAIN_ON_OFF

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
	ScalableNumber(real_block_quantity, 3, (S_IMGW >> 1) - 50, 130, iTS);
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

	if(real_block_quantity > 0 && block_quantity > 1)
	{
		iTS->GroundReflector_Ctr = LimitCount(iTS->GroundReflector_Ctr += 5, 30);
	}
	else if(real_block_quantity == 1)
	{
		if(iTS->L_Dual_to_Single_Flag == 1)
		{
			iTS->GroundReflector_Ctr = iTS->GroundReflector_Ctr;
		}
		else
		{
			iTS->GroundReflector_Ctr = LimitCount(iTS->GroundReflector_Ctr += 3, 30);
		}
	}
	else
	{
		iTS->GroundReflector_Ctr = LimitCount(-- iTS->GroundReflector_Ctr, 30);
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

#ifdef L_SHOW_DETECTION_BLOCK
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber(iTS->L_Edge_Quantity, 2, start_col, start_row - 25, iTS);
			ScalableNumber(iTS->L_Pixel_Quantity_in_Block, 2, start_col, start_row - 40, iTS);
			ScalableNumber(iTS->L_Over_TH_Pixel_Quantity, 2, start_col, start_row - 55, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

			//條件5:區塊內像素值大於門檻的比例超過一定門檻
			//條件6:區塊內被切出的垂直edge之比例超過一定門檻
			if(iTS->L_Over_TH_Pixel_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_PIXEL_PERCENT)
			   && iTS->L_Edge_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_EDGE_PERCENT))
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

#ifdef L_SHOW_DETECTION_BLOCK
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber(iTS->L_Edge_Quantity, 2, start_col, start_row - 25, iTS);
			ScalableNumber(iTS->L_Pixel_Quantity_in_Block, 2, start_col, start_row - 40, iTS);
			ScalableNumber(iTS->L_Over_TH_Pixel_Quantity, 2, start_col, start_row - 55, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

			if(iTS->L_Over_TH_Pixel_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_PIXEL_PERCENT)
			   && iTS->L_Edge_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_EDGE_PERCENT))
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

#ifdef L_SHOW_DETECTION_BLOCK
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber(iTS->L_Edge_Quantity, 2, start_col, start_row - 25, iTS);
			ScalableNumber(iTS->L_Pixel_Quantity_in_Block, 2, start_col, start_row - 40, iTS);
			ScalableNumber(iTS->L_Over_TH_Pixel_Quantity, 2, start_col, start_row - 55, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

			if(iTS->L_Over_TH_Pixel_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_PIXEL_PERCENT)
			   && iTS->L_Edge_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_EDGE_PERCENT))
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

#ifdef L_SHOW_DETECTION_BLOCK
			OSD_Color_Setup(OCN_ORANGE, iTS);
			ScalableNumber(iTS->L_Edge_Quantity, 2, start_col, start_row - 25, iTS);
			ScalableNumber(iTS->L_Pixel_Quantity_in_Block, 2, start_col, start_row - 40, iTS);
			ScalableNumber(iTS->L_Over_TH_Pixel_Quantity, 2, start_col, start_row - 55, iTS);
#endif    //L_SHOW_DETECTION_BLOCK

			if(iTS->L_Over_TH_Pixel_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_PIXEL_PERCENT)
			   && iTS->L_Edge_Quantity > (iTS->L_Pixel_Quantity_in_Block * L_OVER_TH_EDGE_PERCENT))
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

			if((iTS->L_ColProjection[block_initial + col] & VEINFO) == VEINFO)
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

#endif    //L_PROTECTION_MAIN_ON_OFF
