#include "InitialVariable.h"
#include "FunctionType.h"

//Corner資訊初值化
void F_V_CornerReset(ITS * iTS)
{
	short index = 0;

	for(index = 0; index < 10; index++)
	{
		iTS->left_corner[index][0] = 0;
		iTS->left_corner[index][1] = 0;
		iTS->right_corner[index][0] = 0;
		iTS->right_corner[index][1] = 0;

	}
	iTS->left_corner_count = 0;
	iTS->right_corner_count = 0;
	iTS->pair_corner_count = 0;
	iTS->left_corner_mean = 0;
	iTS->right_corner_mean = 0;

}

//依據列數差異門檻值清除舊的Corner資訊
void F_V_CleanOldCorner(short row, ITS * iTS)
{
	short index = 0;
	short left_count = 0;
	short left_clean_flag = 0;
	short right_count = 0;
	short right_clean_flag = 0;

	if(iTS->left_corner_count != 0)
	{
		for(index = 0; index < iTS->left_corner_count; index++)
		{
			if(iTS->left_corner[index][1] != 0 && abs(row - iTS->left_corner[index][1]) >= CORNER_DIFFERENCE_ROW_THRESHOLD)
			{
				iTS->left_corner[index][0] = 0;
				iTS->left_corner[index][1] = 0;
				left_clean_flag = 1;
			}
			else if(left_clean_flag == 1)
			{
				iTS->left_corner[left_count][0] = iTS->left_corner[index][0];
				iTS->left_corner[left_count][1] = iTS->left_corner[index][1];
				iTS->left_corner[index][0] = 0;
				iTS->left_corner[index][1] = 0;
				left_count++;
			}
		}
		if(left_clean_flag == 0)
		{
			left_count = iTS->left_corner_count;
		}
	}

	if(iTS->right_corner_count != 0)
	{
		for(index = 0; index < iTS->right_corner_count; index++)
		{
			if(iTS->right_corner[index][1] != 0 && abs(row - iTS->right_corner[index][1]) >= CORNER_DIFFERENCE_ROW_THRESHOLD)
			{
				iTS->right_corner[index][0] = 0;
				iTS->right_corner[index][1] = 0;
				right_clean_flag = 1;
			}
			else if(right_clean_flag == 1)
			{
				iTS->right_corner[right_count][0] = iTS->right_corner[index][0];
				iTS->right_corner[right_count][1] = iTS->right_corner[index][1];
				iTS->right_corner[index][0] = 0;
				iTS->right_corner[index][1] = 0;
				right_count++;
			}
		}
		if(right_clean_flag == 0)
		{
			right_count = iTS->right_corner_count;
		}
	}
	iTS->left_corner_count = left_count;
	iTS->right_corner_count = right_count;
	iTS->pair_corner_count = 0;
	iTS->left_corner_mean = 0;
	iTS->right_corner_mean = 0;
}

//尋找偵測範圍內的Corner
void F_V_FindCorner(unsigned char *image_buffer, short row, short column, ITS * iTS)
{
	//根據影像中的距離使用不同的遮罩大小
	if(row <= iTS->V_distance_number[0])
	{
		//距離近使用大的Corner遮罩
		F_V_BigCornerMask(image_buffer, row, column, iTS);
	}
	else
	{
		//距離遠使用小的Corner遮罩
		F_V_MiniCornerMask(image_buffer, row, column, iTS);
	}
}

//Corner成對配對
short F_V_CornerPairDetector(short row, ITS * iTS)
{
	short index = 0;
	short index2 = 0;

	for(index = 0; index < iTS->left_corner_count; index++)
	{
		for(index2 = 0; index2 < iTS->right_corner_count; index2++)
		{
			//Corner└ ┘配對
			if((iTS->right_corner[index2][0] > iTS->left_corner[index][0]) &&
			   ((iTS->right_corner[index2][0]  - iTS->left_corner[index][0] + 1) >= iTS->min_image_car_width[row]) &&
			   ((iTS->right_corner[index2][0]  - iTS->left_corner[index][0] + 1) <= iTS->max_image_car_width[row]))
			{

#ifdef O_DRAW_ALL_PAIR_CORNER_DEBUG
				OSD_Color_Setup(OCN_BLUE, iTS);    //配對1為藍色
				DrawRectangle(row, row + 10, iTS->left_corner[index][0], iTS->left_corner[index][0], iTS);
				DrawRectangle(row, row, iTS->left_corner[index][0], iTS->left_corner[index][0] + 5, iTS);
				DrawRectangle(row, row + 10, iTS->right_corner[index2][0], iTS->right_corner[index2][0], iTS);
				DrawRectangle(row, row, iTS->right_corner[index2][0] - 5, iTS->right_corner[index2][0], iTS);
#endif

				iTS->left_corner_mean += iTS->left_corner[index][0];
				iTS->right_corner_mean += iTS->right_corner[index2][0];
				iTS->pair_corner_count++;
			}
		}
	}

#ifdef O_DRAW_ALL_PAIR_CORNER_DEBUG
		OSD_Color_Setup(OCN_BLUE, iTS);
		ScalableNumber((short) iTS->pair_corner_count, 4, (S_IMGW >> 1), 70, iTS);
#endif

	//平均配對成功的Corner位置
	if(iTS->pair_corner_count  != 0)
	{
		iTS->left_corner_mean /= iTS->pair_corner_count;
		iTS->right_corner_mean /= iTS->pair_corner_count;

#ifdef O_DRAW_PAIR_CORNER_DEBUG
		//畫出成對Corner debug
		OSD_Color_Setup(OCN_YELLOW, iTS);    //配對橘色
		DrawRectangle(row, row + 5, iTS->left_corner_mean, iTS->left_corner_mean, iTS);
		DrawRectangle(row, row, iTS->left_corner_mean, iTS->left_corner_mean + 5, iTS);
		DrawRectangle(row, row + 5, iTS->right_corner_mean, iTS->right_corner_mean, iTS);
		DrawRectangle(row, row, iTS->right_corner_mean - 5, iTS->right_corner_mean, iTS);
#endif    //O_DRAW_PAIR_CORNER_DEBUG

		return 1;
	}
	return 0;
}

//大的Corner遮罩
void F_V_BigCornerMask(unsigned char *image_buffer, short row, short column, ITS * iTS)
{
	//根據Sobel edge尋找Corner
	if((image_buffer[column] & HEVEINFO))
	{
		if((image_buffer[column - iTS->row_width_culmulative[1]] & HEVEINFO) == VEINFO && (image_buffer[column - iTS->row_width_culmulative[2]] & HEVEINFO) == VEINFO&&
			(image_buffer[column - iTS->row_width_culmulative[3]] & HEVEINFO) == VEINFO && (image_buffer[column - iTS->row_width_culmulative[4]] & HEVEINFO) == VEINFO &&
			(image_buffer[column - iTS->row_width_culmulative[5]] & HEVEINFO) == VEINFO && (image_buffer[column - iTS->row_width_culmulative[6]] & HEVEINFO) == VEINFO &&
			(image_buffer[column - iTS->row_width_culmulative[7]] & HEVEINFO) == VEINFO && !(image_buffer[column + iTS->row_width_culmulative[1]] & HEVEINFO) &&
			!(image_buffer[column + iTS->row_width_culmulative[2]] & HEVEINFO) && !(image_buffer[column + iTS->row_width_culmulative[3]] & HEVEINFO) &&
			!(image_buffer[column + iTS->row_width_culmulative[4]] & HEVEINFO) && !(image_buffer[column + iTS->row_width_culmulative[5]] & HEVEINFO))
		{

			//尋找└ corner
			if(!(image_buffer[column - 1] & HEVEINFO) && !(image_buffer[column - 2] & HEVEINFO) &&
				!(image_buffer[column - 3] & HEVEINFO) && (image_buffer[column + 1] & HEVEINFO) == HEINFO &&
				(image_buffer[column + 2] & HEVEINFO) == HEINFO && (image_buffer[column + 3] & HEVEINFO) == HEINFO &&
				(image_buffer[column + 4] & HEVEINFO) == HEINFO && (image_buffer[column + 5] & HEVEINFO) == HEINFO)
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_ORANGE, iTS);    //畫出Corner
				DrawRectangle(row, row + 10, column, column, iTS);
				DrawRectangle(row, row, column, column + 5, iTS);
#endif

				iTS->left_corner[iTS->left_corner_count][0] = column;
				iTS->left_corner[iTS->left_corner_count][1] = row;
				iTS->left_corner_count++;
				if(iTS->left_corner_count == 10)
					iTS->left_corner_count = 9;
			}

			//尋找└ corner 2
			if(!(image_buffer[column - 1] & HEVEINFO) && !(image_buffer[column - 2] & HEVEINFO) &&
				!(image_buffer[column - 3] & HEVEINFO))
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_ORANGE, iTS);    //畫出Corner
				DrawRectangle(row, row + 10, column, column, iTS);
#endif

				iTS->left_corner[iTS->left_corner_count][0] = column;
				iTS->left_corner[iTS->left_corner_count][1] = row;
				iTS->left_corner_count++;
				if(iTS->left_corner_count == 10)
					iTS->left_corner_count = 9;
			}

			//尋找 ┘corner
			if(!(image_buffer[column + 1] & HEVEINFO) && !(image_buffer[column + 2] & HEVEINFO) &&
				!(image_buffer[column + 3] & HEVEINFO)&& (image_buffer[column - 1] & HEVEINFO) == HEINFO &&
				(image_buffer[column - 2] & HEVEINFO) == HEINFO && (image_buffer[column - 3] & HEVEINFO) == HEINFO &&
				(image_buffer[column - 4] & HEVEINFO) == HEINFO && (image_buffer[column - 5] & HEVEINFO) == HEINFO)
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_ORANGE, iTS);    //畫出Corner
				DrawRectangle(row, row + 10, column, column, iTS);
				DrawRectangle(row, row, column-5, column, iTS);
#endif

				iTS->right_corner[iTS->right_corner_count][0] = column;
				iTS->right_corner[iTS->right_corner_count][1] = row;
				iTS->right_corner_count++;
				if(iTS->right_corner_count == 10)
					iTS->right_corner_count = 9;
			}

			//尋找 ┘corner2
			if(!(image_buffer[column + 1] & HEVEINFO) && !(image_buffer[column + 2] & HEVEINFO) &&
				!(image_buffer[column + 3] & HEVEINFO))
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_ORANGE, iTS);    //畫出Corner
				DrawRectangle(row, row + 10, column, column, iTS);
#endif

				iTS->right_corner[iTS->right_corner_count][0] = column;
				iTS->right_corner[iTS->right_corner_count][1] = row;
				iTS->right_corner_count++;
				if(iTS->right_corner_count == 10)
					iTS->right_corner_count = 9;
			}
		}
	}

	//根據Shadow尋找Corner
	if((image_buffer[column] & RSDINFO))
	{
		if((image_buffer[column - iTS->row_width_culmulative[1]] & RSDINFO) && (image_buffer[column - iTS->row_width_culmulative[2]] & RSDINFO) &&
			(image_buffer[column - iTS->row_width_culmulative[3]] & RSDINFO) && (image_buffer[column - iTS->row_width_culmulative[4]] & RSDINFO) &&
			(image_buffer[column - iTS->row_width_culmulative[5]] & RSDINFO) && (image_buffer[column - iTS->row_width_culmulative[6]] & RSDINFO) &&
			(image_buffer[column - iTS->row_width_culmulative[7]] & RSDINFO) && (image_buffer[column - iTS->row_width_culmulative[8]] & RSDINFO) &&
			(image_buffer[column - iTS->row_width_culmulative[9]] & RSDINFO) && (image_buffer[column - iTS->row_width_culmulative[10]] & RSDINFO) &&
			!(image_buffer[column + iTS->row_width_culmulative[1]] & RSDINFO) && !(image_buffer[column + iTS->row_width_culmulative[2]] & RSDINFO) &&
			!(image_buffer[column + iTS->row_width_culmulative[3]] & RSDINFO) && !(image_buffer[column + iTS->row_width_culmulative[4]] & RSDINFO) &&
			!(image_buffer[column + iTS->row_width_culmulative[5]] & RSDINFO))
		{

			//尋找└ corner
			if(!(image_buffer[column - 1] & RSDINFO) && !(image_buffer[column - 2] & RSDINFO) &&
				!(image_buffer[column - 3] & RSDINFO) && !(image_buffer[column - 4] & RSDINFO) &&
				!(image_buffer[column - 5] & RSDINFO) && (image_buffer[column + 1] & RSDINFO) &&
				(image_buffer[column + 2] & RSDINFO) && (image_buffer[column + 3] & RSDINFO) &&
				(image_buffer[column + 4] & RSDINFO) && (image_buffer[column + 5] & RSDINFO))
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_GREEN, iTS);    //畫出Corner
				DrawRectangle(row, row + 10, column, column, iTS);
				DrawRectangle(row, row, column, column + 5, iTS);
#endif

				iTS->left_corner[iTS->left_corner_count][0] = column;
				iTS->left_corner[iTS->left_corner_count][1] = row;
				iTS->left_corner_count++;
				if(iTS->left_corner_count == 10)
					iTS->left_corner_count = 9;
			}

			//尋找 ┘corner
			if(!(image_buffer[column + 1] & RSDINFO) && !(image_buffer[column + 2] & RSDINFO) &&
				!(image_buffer[column + 3] & RSDINFO) && !(image_buffer[column + 4] & RSDINFO) &&
				!(image_buffer[column + 5] & RSDINFO) && (image_buffer[column - 1] & RSDINFO) &&
				(image_buffer[column - 2] & RSDINFO) && (image_buffer[column - 3] & RSDINFO) &&
				(image_buffer[column - 4] & RSDINFO) && (image_buffer[column - 5] & RSDINFO))
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_GREEN, iTS);    //畫出Corner
				DrawRectangle(row, row + 10, column, column, iTS);
				DrawRectangle(row, row, column-5, column, iTS);
#endif

				iTS->right_corner[iTS->right_corner_count][0] = column;
				iTS->right_corner[iTS->right_corner_count][1] = row;
				iTS->right_corner_count++;
				if(iTS->right_corner_count == 10)
					iTS->right_corner_count = 9;
			}
		}
	}
}

//小的Corner遮罩
void F_V_MiniCornerMask(unsigned char *image_buffer, short row, short column, ITS * iTS)
{
	//根據Sobel edge尋找Corner
	if((image_buffer[column] & HEVEINFO))
	{
		if((image_buffer[column - iTS->row_width_culmulative[1]] & HEVEINFO) == VEINFO && (image_buffer[column - iTS->row_width_culmulative[2]] & HEVEINFO) == VEINFO &&
			(image_buffer[column - iTS->row_width_culmulative[3]] & HEVEINFO) == VEINFO && (image_buffer[column - iTS->row_width_culmulative[4]] & HEVEINFO) == VEINFO &&
			(image_buffer[column - iTS->row_width_culmulative[5]] & HEVEINFO) == VEINFO && !(image_buffer[column + iTS->row_width_culmulative[1]] & HEVEINFO) &&
			!(image_buffer[column + iTS->row_width_culmulative[2]] & HEVEINFO) && !(image_buffer[column + iTS->row_width_culmulative[3]] & HEVEINFO))
		{

			//尋找└ corner
			if(!(image_buffer[column - 1] & HEVEINFO) && !(image_buffer[column - 2] & HEVEINFO) &&
				!(image_buffer[column - 3] & HEVEINFO) && (image_buffer[column + 1] & HEVEINFO) == HEINFO &&
				(image_buffer[column + 2] & HEVEINFO) == HEINFO && (image_buffer[column + 3] & HEVEINFO) == HEINFO)
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_ORANGE, iTS);    //畫出Corner
				DrawRectangle(row, row + 5, column, column, iTS);
				DrawRectangle(row, row, column, column + 5, iTS);
#endif

				iTS->left_corner[iTS->left_corner_count][0] = column;
				iTS->left_corner[iTS->left_corner_count][1] = row;
				iTS->left_corner_count++;
				if(iTS->left_corner_count == 10)
					iTS->left_corner_count = 9;
			}
			//尋找└ corner 2
			if(!(image_buffer[column - 1] & HEVEINFO) && !(image_buffer[column - 2] & HEVEINFO) &&
				!(image_buffer[column - 3] & HEVEINFO))
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_ORANGE, iTS);    //畫出Corner
				DrawRectangle(row, row + 5, column, column, iTS);
#endif

				iTS->left_corner[iTS->left_corner_count][0] = column;
				iTS->left_corner[iTS->left_corner_count][1] = row;
				iTS->left_corner_count++;
				if(iTS->left_corner_count == 10)
					iTS->left_corner_count = 9;
			}

			//尋找 ┘corner
			if(!(image_buffer[column + 1] & HEVEINFO) && !(image_buffer[column + 2] & HEVEINFO) &&
				!(image_buffer[column + 3] & HEVEINFO)&& (image_buffer[column - 1] & HEVEINFO) == HEINFO &&
				(image_buffer[column - 2] & HEVEINFO) == HEINFO && (image_buffer[column - 3] & HEVEINFO) == HEINFO)
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_ORANGE, iTS);    //畫出Corner
				DrawRectangle(row, row + 5, column, column, iTS);
				DrawRectangle(row, row, column-5, column, iTS);
#endif

				iTS->right_corner[iTS->right_corner_count][0] = column;
				iTS->right_corner[iTS->right_corner_count][1] = row;
				iTS->right_corner_count++;
				if(iTS->right_corner_count == 10)
					iTS->right_corner_count = 9;
			}

			//尋找 ┘corner2
			if(!(image_buffer[column + 1] & HEVEINFO) && !(image_buffer[column + 2] & HEVEINFO) &&
				!(image_buffer[column + 3] & HEVEINFO))
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_ORANGE, iTS);    //畫出Corner
				DrawRectangle(row, row + 5, column, column, iTS);
#endif

				iTS->right_corner[iTS->right_corner_count][0] = column;
				iTS->right_corner[iTS->right_corner_count][1] = row;
				iTS->right_corner_count++;
				if(iTS->right_corner_count == 10)
					iTS->right_corner_count = 9;
			}
		}
	}

	//根據Shadow尋找Corner
	if((image_buffer[column] & RSDINFO))
	{
		if((image_buffer[column - iTS->row_width_culmulative[1]] & RSDINFO) && (image_buffer[column - iTS->row_width_culmulative[2]] & RSDINFO) &&
			(image_buffer[column - iTS->row_width_culmulative[3]] & RSDINFO) && (image_buffer[column - iTS->row_width_culmulative[4]] & RSDINFO) &&
			(image_buffer[column - iTS->row_width_culmulative[5]] & RSDINFO) && !(image_buffer[column + iTS->row_width_culmulative[1]] & RSDINFO) &&
			!(image_buffer[column + iTS->row_width_culmulative[2]] & RSDINFO) && !(image_buffer[column + iTS->row_width_culmulative[3]] & RSDINFO))
		{
			//尋找└ corner
			if(!(image_buffer[column - 1] & RSDINFO) && !(image_buffer[column - 2] & RSDINFO) &&
				!(image_buffer[column - 3] & RSDINFO) && (image_buffer[column + 1] & RSDINFO) &&
				(image_buffer[column + 2] & RSDINFO) && (image_buffer[column + 3] & RSDINFO) &&
				(image_buffer[column + 4] & RSDINFO) && (image_buffer[column + 5] & RSDINFO))
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_GREEN, iTS);    //畫出Corner
				DrawRectangle(row, row + 5, column, column, iTS);
				DrawRectangle(row, row, column, column + 5, iTS);
#endif

				iTS->left_corner[iTS->left_corner_count][0] = column;
				iTS->left_corner[iTS->left_corner_count][1] = row;
				iTS->left_corner_count++;
				if(iTS->left_corner_count == 10)
					iTS->left_corner_count = 9;
			}

			//尋找 ┘corner
			if(!(image_buffer[column + 1] & RSDINFO) && !(image_buffer[column + 2] & RSDINFO) &&
				!(image_buffer[column + 3] & RSDINFO)&& (image_buffer[column - 1] & RSDINFO) &&
				(image_buffer[column - 2] & RSDINFO) && (image_buffer[column - 3] & RSDINFO) &&
				(image_buffer[column - 4] & RSDINFO) && (image_buffer[column - 5] & RSDINFO))
			{

#ifdef O_DRAW_CORNER_DEBUG
				OSD_Color_Setup(OCN_GREEN, iTS);    //畫出Corner
				DrawRectangle(row, row + 5, column, column, iTS);
				DrawRectangle(row, row, column-5, column, iTS);
#endif

				iTS->right_corner[iTS->right_corner_count][0] = column;
				iTS->right_corner[iTS->right_corner_count][1] = row;
				iTS->right_corner_count++;
				if(iTS->right_corner_count == 10)
					iTS->right_corner_count = 9;
			}
		}
	}
}

