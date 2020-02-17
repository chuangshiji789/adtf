#include "InitialVariable.h"
#include "FunctionType.h"


short F_V_FindContinuousProjection(short not_successive_threshold, unsigned char *image_buffer, short start, short end, short row, ITS *iTS)
{
	short col = 0;

	char  horizontal_flag = 0;
	short horizontal_temp = 0;
	short horizontal_max_length = 0;
	short horizontal_successive_count = 0;
	short horizontal_not_successive_count = 0;
	short horizontal_projection_start = 0;
	short horizontal_projection_end = 0;

	char  shadow_flag = 0;
	short shadow_temp = 0;
	short shadow_max_length = 0;
	short shadow_successive_count = 0;
	short shadow_not_successive_count = 0;
	short shadow_projection_start = 0;
	short shadow_projection_end = 0;

	short return_value = 0;

	iTS->shadow_projection = 0;

	//根據列數差異清除紀錄的Corner
	if(iTS->find_corner_flag == 0)
	{
		F_V_CleanOldCorner(row, iTS);
	}

	for(col = start; col <= end; col++)
	{

		//找出Corner
		if(iTS->find_corner_flag == 0)
		{
			if(col >= iTS->O_LaneMBound[row].Frt - 20 && col <= iTS->O_LaneMBound[row].Scd + 20 &&
				row > O_IB_BB_SearchVehicle + 5 && row < S_IMGCH - 10)
				F_V_FindCorner(image_buffer, row, col,iTS);
		}

		//Sobel水平連續投影
		if((image_buffer[col] & HEVEINFO) == HEINFO)
		{
			horizontal_successive_count++;
			if(horizontal_flag == 0)
			{
				horizontal_temp = col;
				horizontal_flag = 1;
			}
			else
			{
				horizontal_successive_count += horizontal_not_successive_count;
			}
			horizontal_not_successive_count = 0;
		}
		else
		{
			horizontal_not_successive_count++;
			if(horizontal_not_successive_count > not_successive_threshold)
			{
				if(horizontal_successive_count >= horizontal_max_length && horizontal_successive_count >= iTS->min_image_car_width[row])
				{
					horizontal_max_length = horizontal_successive_count;

					//紀錄連續水平投影量位置
					horizontal_projection_start = horizontal_temp;
					horizontal_projection_end   = col - horizontal_not_successive_count + 1;

					//OSD_Color_Setup(OCN_PURPLE, iTS);	//畫出連續水平投影量位置
					//DrawColorLine(row, horizontal_projection_start, horizontal_projection_end, iTS);
				}
				horizontal_flag = 0;
				horizontal_temp = 0;
				horizontal_successive_count = 0;
				horizontal_not_successive_count = 0;
			}
		}

		//陰影連續投影
		if((image_buffer[col] & RSDINFO) == RSDINFO)
		{
			shadow_successive_count++;
			if(shadow_flag == 0)
			{
				shadow_temp = col;
				shadow_flag = 1;
			}
			else
			{
				shadow_successive_count += shadow_not_successive_count;
			}
			shadow_not_successive_count = 0;
		}
		else
		{
			shadow_not_successive_count++;
			if(shadow_not_successive_count > not_successive_threshold)
			{
				if(shadow_successive_count >= shadow_max_length && shadow_successive_count >= iTS->min_image_car_width[row])
				{
					shadow_max_length = shadow_successive_count;

					//紀錄連續陰影投影量位置
					shadow_projection_start = shadow_temp;
					shadow_projection_end = col - shadow_not_successive_count + 1;

					//OSD_Color_Setup(OCN_ORANGE, iTS);	//畫出連續陰影投影量位置
					//DrawColorLine(row, shadow_projection_start, shadow_projection_end , iTS);
				}
				shadow_flag = 0;
				shadow_temp = 0;

				shadow_successive_count = 0;
				shadow_not_successive_count = 0;
			}
		}

	}

	if(horizontal_successive_count >= horizontal_max_length && horizontal_successive_count >= iTS->min_image_car_width[row])
	{
		horizontal_max_length = horizontal_successive_count;

		//紀錄連續水平投影量位置
		horizontal_projection_start = horizontal_temp;
		horizontal_projection_end   = col - horizontal_not_successive_count + 1;

		//OSD_Color_Setup(OCN_PURPLE, iTS);	//畫出連續水平投影量位置
		//DrawColorLine(row, horizontal_projection_start, horizontal_projection_end, iTS);
	}

	if(shadow_successive_count >= shadow_max_length && shadow_successive_count >= iTS->min_image_car_width[row])
	{
		shadow_max_length = shadow_successive_count;

		//紀錄連續陰影投影量位置
		shadow_projection_start = shadow_temp;
		shadow_projection_end = col - shadow_not_successive_count + 1;

		//OSD_Color_Setup(OCN_ORANGE, iTS);	//畫出連續陰影投影量位置
		//DrawColorLine(row, shadow_projection_start, shadow_projection_end , iTS);
	}

	//找出成對Corner
	if(iTS->find_corner_flag == 0)
	{
		if(iTS->left_corner_count != 0 && iTS->right_corner_count != 0)
		{
			if(F_V_CornerPairDetector(row, iTS) == 1)
			{
				iTS->have_corner_row = row;
				iTS->find_corner_flag = 1;
				return_value = 100;
			}
		}
	}

	if((shadow_max_length >= iTS->min_image_car_width[row]) &&
		(V_DetemineContinuousProjectionPosition(row, shadow_projection_start, shadow_projection_end, iTS) == 1))
	{
		iTS->shadow_projection = 1;
		return_value = 100;
	}

	if(iTS->day_or_night_flag == 1)
	{
		if((iTS->shadow_projection == 1) && (horizontal_max_length >= iTS->min_image_car_width[row]) &&
			(V_DetemineContinuousProjectionPosition(row, horizontal_projection_start, horizontal_projection_end, iTS) == 1))
		{
			return_value = 100;
		}
	}
	else
	{
		if((horizontal_max_length >= iTS->min_image_car_width[row]) &&
			(V_DetemineContinuousProjectionPosition(row, horizontal_projection_start, horizontal_projection_end, iTS) == 1))
		{
			return_value = 100;
		}
	}
	return return_value;
}


short O_FindMax(short *Array,short start,short end,short *MaxIndex)
{
	short i;
	short MaxValue=0;
	for(i = start; i <= end; i++)
	{
		if(Array[i] > MaxValue)
		{
			MaxValue = Array[i];
			*MaxIndex = i;
		}
	}
	return MaxValue;
}

short O_NormalizeArray(short *Array,short start,short end,short MaxValue)
{
	short i;
	int temp;
	if(MaxValue==0)
		return 0;
	for(i=start; i<=end; i++)
	{
		if(Array[i]==0)
			continue;

		temp=Array[i]*100;
		Array[i]=temp/MaxValue;
		if(Array[i]>100)
			Array[i]=100;
		if(Array[i]<0)
			Array[i]=0;
	}
	return 1;
}

short O_GetMean(short *Array,short start,short end)
{
	short i;
	int Temp=0;
	if(end+1-start <= 1) return 0;

	for(i=start; i<=end; i++)
		Temp+=Array[i];
	Temp/=(end-start+1);
	return Temp;
}

short O_FindSingleGroup(short Th, short Endurance, short MinWidth, short start, short end, short *InArr, FORWARD_CROI *Result)
{
	short i,j,je,js;
	short Gs,Ge;
	short Gnum = 0;
	short Flag = 0;
	for(i = start; i <= end; i++)
	{
		if(InArr[i] < Th)
			continue;

		Gs = Ge = i;
		do
		{
			Flag=0;
			if(i+1>end)
				break;
			else
			{
				i++;
				js=i;
			}
			if(i+Endurance > end)
				je=end;
			else
				je=(i+Endurance);
			for(j=js; j<=je; j++)
			{
				if(InArr[j]>=Th)
				{
					Flag=1;
					i=j;
					Ge=j;
				}
			}
		}
		while(Flag==1&&i<=end);

		if(Ge-Gs<MinWidth)
			continue;
		else
		{
			Result[Gnum].Frt=(Ge+Gs)>>1;
			Result[Gnum].Scd=Ge-Gs+1;
			Gnum++;
			if(Gnum>=N_MaxSingleGroup)
				return N_MaxSingleGroup;
		}

	}
	return Gnum;
}

short O_FindPairGroup(short MinW,short MaxW,FORWARD_CROI *In,short InGrpNum,FORWARD_CROI *Out)
{   //O_FindPairGroup(MinW,MaxW,O_VE_VP_SingleGroupArray,O_VE_VP_SingleGroupNum,iTS->O_VE_VP_PairGroupArray);
	short PGrpNum=0;
	short i,j,w;
	if(InGrpNum<=1)
		return 0;
	for(i=0; i<InGrpNum; i++)
	{
		for(j=i+1; j<InGrpNum; j++)
		{
			w=In[j].Frt-In[i].Frt+1;
			if(w>=MinW && w<=MaxW)
			{
				Out[PGrpNum].Frt=In[i].Frt;
				Out[PGrpNum].Scd=In[j].Frt;
				PGrpNum++;
				if(PGrpNum>=N_MaxPairGroup)
					return 0;
			}
		}
	}
	return PGrpNum;
}

void ArrayPreprocessing(ITS *iTS)
{
	short index = 0;

	for(index = 0; index < S_IMGH; index++)
	{
		iTS->row_width_culmulative[index] = index * iTS->F_W;
	}
}

void CalculateImageCarWidth(ITS *iTS)
{
	short index = 0;
	//short max_width = S_IMGW - (S_IMGLB * 2);

	for(index = 0; index < S_IMGCH; index++)
	{
		iTS->min_image_car_width[index] = O_DWtoDI(O_RW_MinCarWidth, index, iTS);
		if(iTS->min_image_car_width[index] <= 10)
			iTS->min_image_car_width[index] = 10;

		iTS->max_image_car_width[index] = S_IMGW; //O_DWtoDI(O_RW_MaxCarWidth, index, iTS);
		/*if(iTS->max_image_car_width[index] <= 20)
			iTS->max_image_car_width[index] = 20;
		if(iTS->max_image_car_width[index] >= max_width)
			iTS->max_image_car_width[index] = max_width;*/
	}
}

//20111025 Jeremy Lee Added This Function for Class 9 of Vehicle Detection
//#define DRAW_VARIABLES_OF_PROJECTION_EXTRACTION
short F_V_ProjectionFeatrueExtraction(short left_boundary, short right_boundary, ITS *iTS)
{
	short index = 0; //行索引
	short calculation_width = right_boundary - left_boundary; //陣列的計算長度

	short not_zero_counter = 0; //非零直計數
	short no_zero_threshold = calculation_width - (calculation_width >> 2); //非零值門檻

	unsigned int average = 0; //平均高度
	short short_average = 0;
	short apperance_time_of_average = 0; //平均高度以上出現次數
	short average_apperance_threshold = (calculation_width >> 2) - (calculation_width >> 4); //平均高度以上出現次數門檻

	short max_value = 0; //最大值

	for(index = left_boundary; index <= right_boundary; index ++)
	{
		if(iTS->O_VE_VP_Array[index] > 0) //非零值計數
		{
			not_zero_counter ++;
			average += iTS->O_VE_VP_Array[index];
		}

		if(iTS->O_VE_VP_Array[index] > max_value) //最大值保留
			max_value = iTS->O_VE_VP_Array[index];
	}

	if(average == 0 || not_zero_counter == 0 || max_value != 100) //離開條件：加總為零 || 非零值計數為零 || 最大值未超過100
		return 0;

	short_average = (short) (average / not_zero_counter); //平均值計算

	for(index = left_boundary; index <= right_boundary; index ++) //平均值以上出現次數計數
	{
		if(iTS->O_VE_VP_Array[index] >= short_average)
			apperance_time_of_average ++;
	}

	if(not_zero_counter < no_zero_threshold) //離開條件：非零列大於非零門檻
		return 0;

	if(apperance_time_of_average < average_apperance_threshold) //離開條件：平均值以上出現次數小於門檻
		return 0;

#ifdef DRAW_VARIABLES_OF_PROJECTION_EXTRACTION
	OSD_Color_Setup(OCN_RED, iTS);
	ScalableNumber((short) not_zero_counter, 2, 20, S_IMGH - 50, iTS);
	ScalableNumber((short) left_boundary, 2, 70, S_IMGH - 50, iTS);
	ScalableNumber((short) right_boundary, 2, 120, S_IMGH - 50, iTS);
	ScalableNumber((short) average, 2, 170, S_IMGH - 50, iTS);
	ScalableNumber((short) max_value, 2, 220, S_IMGH - 50, iTS);
	ScalableNumber((short) apperance_time_of_average, 2, 270, S_IMGH - 50, iTS);
	ScalableNumber((short) iTS->first_search_row, 2, 20, S_IMGH - 30, iTS);
	ScalableNumber((short) right_boundary - left_boundary, 2, 70, S_IMGH - 30, iTS);
	ScalableNumber((short) no_zero_threshold, 2, 120, S_IMGH - 30, iTS);
	ScalableNumber((short) average_apperance_threshold, 2, 170, S_IMGH - 30, iTS);
#endif

	return 1;
}
//20111025 Jeremy Lee Added This Function

short F_V_FindBoundaryUsedExistedVerticalEdgeProjection(short left_boundary, short right_boundary, ITS *iTS)
{
	short calculation_width = right_boundary - left_boundary;
	short start_column = (left_boundary + right_boundary) >> 1;
	short column = 0;
	short left_column = 0;
	short right_column = 0;
	short find_left = 0;
	short find_right = 0;
	short left_maximum = 0;
	short right_maximum = 0;

	for(column = 0; column <= calculation_width; column ++)
	{
		left_column = start_column - column;
		if(iTS->O_VE_VP_Array[left_column] > left_maximum)
		{
			find_left = left_column;
			left_maximum = iTS->O_VE_VP_Array[left_column];
		}

		right_column = start_column + column;
		if(iTS->O_VE_VP_Array[right_column] > right_maximum)
		{
			find_right = right_column;
			right_maximum = iTS->O_VE_VP_Array[right_column];
		}
	}
	if(find_left != 0 && find_right != 0)
	{
		iTS->CAR->CarInfo.Position.LB = find_left;
		iTS->CAR->CarInfo.Position.RB = find_right;
		return 1;
	}
	else
	{
		return 0;
	}
}
