#include "InitialVariable.h"
#include "FunctionType.h"

//#define DRAW_CORNER_DEBUG
//#define DRAW_CALCULATION_AREA
//#define DRAW_SEARCH_START_ROW
short O_SetPossiblePosition(short BB, short TB, short Irow, ITS *iTS)
{
	short row_ = 0;
	char last_count = 0; //20111005 Jeremy Lee Modify
	short LB = 0;
	short RB = 0;
	unsigned char *CrRow;
	short not_successive_threshold = 5;
	char first_row_flag = 0;
	int r;

	//在可能的位置找到連續的水平邊緣：前處理
	LB=0;//First50=0;
	RB=0;//First100=0;

	iTS->first_search_row = 0;
	iTS->find_corner_flag = 0;
	iTS->have_corner_row = 0;

	r=SinkDataIndexStart(BB);
	memset(iTS->O_HD_Array, 0, S_IMGH * sizeof(short));


	//找到可能是車的陰影或車燈 Searching Mode, Make sure Mode使用
	if(Irow==0)
	{
		LB = S_IMGLB;
		RB = S_IMGRB;

		//三值化
		O_CreateRtgRoadLightShadowInfo(iTS->O_CarLightTH, iTS->O_SD_ShadowTh, S_IMGBB, S_IMGCH ,LB, RB, iTS);
	}

	//Tracking Mode
	else
	{
		LB = LimitW(iTS->CAR->CarInfo.Position.LB - (iTS->CAR->CarInfo.Position.BW>>1));
		RB = LimitW(iTS->CAR->CarInfo.Position.RB + (iTS->CAR->CarInfo.Position.BW>>1));

		P_O_CreateRtgRoadLightShadowInfo(Irow,iTS->O_CarLightTH,iTS->O_SD_ShadowTh,BB,TB,LB,RB,iTS);
	}

#ifdef DRAW_CALCULATION_AREA
	//20111005 Jeremy Lee Modify
	if(Irow==0)
	{
		OSD_Color_Setup(OCN_BLUE, iTS);    //畫出上列描述式運算範圍
		DrawRectangle(S_IMGBB, S_IMGCH, LB, RB, iTS);
	}
	else
	{
		OSD_Color_Setup(OCN_BLUE, iTS);    //畫出上列描述式運算範圍
		DrawRectangle(BB, TB, LB, RB, iTS);
	}

	//20111005 Jeremy Lee Modify
	OSD_Color_Setup(OCN_RED, iTS);    //畫下列描述式運算範圍
	DrawRectangle(BB, TB, LB, RB, iTS);
#endif

	for(row_= BB; row_ <= TB; row_++)
	{
		//下一列列數
		r=SinkDataIndexNextRow(r);
		//下一列起始
		CrRow =&iTS->O_InfoPlane[r];

		//找出連續水平edge和陰影投影量的最大寬度
		iTS->O_HD_Array[row_] = F_V_FindContinuousProjection(not_successive_threshold, CrRow, LB, RB, row_, iTS);


		//在RowByRow時從有找到投影量的第一列開始搜尋
		if(iTS->O_HD_Array[row_] == 100 && first_row_flag == 0)
		{
			iTS->first_search_row = row_;

			first_row_flag = 1;

#ifdef DRAW_SEARCH_START_ROW
			memset(&iTS->Showimage[(S_IMGH - row_) * S_IMGW], 255, S_IMGW);
			memset(&iTS->Showimage[(S_IMGH - iTS->first_search_row) * S_IMGW], 0, S_IMGW);
#endif

		}

		//Tracking Mode 時使用
		if(Irow != 0 && iTS->shadow_projection == 1  && last_count == 1)
		{
			iTS->O_HD_Array[row_] = 100;

			if(iTS->first_search_row == 0)
			{
				iTS->first_search_row = row_;

#ifdef DRAW_SEARCH_START_ROW
				memset(&iTS->Showimage[(S_IMGH - row_ - 1) * S_IMGW], 128, S_IMGW);
				memset(&iTS->Showimage[(S_IMGH - row_ - 2) * S_IMGW], 128, S_IMGW);
#endif
			}

		}
		last_count = iTS->shadow_projection;
	}

	if(iTS->first_search_row != 0)
	{
		return iTS->first_search_row;
	}

	return -1;
}

short V_DetemineContinuousProjectionPosition(short row_, short start, short end, ITS *iTS)
{
	short width = 0;

	//算出最大連續投影量起始點與終點寬度的1/4
	width = (end - start + 1)>>2;

	//最大連續投影量起始點與結束點都在左側車道
	if(start < iTS->O_LaneBound[row_].Frt && end < iTS->O_LaneBound[row_].Frt)
	{
		return 0;
	}

	//最大連續投影量起始點與結束點都在右側車道
	if(start > iTS->O_LaneBound[row_].Scd && end > iTS->O_LaneBound[row_].Scd)
	{
		return 0;
	}

	//最大連續投影量起始點與結束點超出中間車道
	if(start < iTS->O_LaneBound[row_].Frt && end > iTS->O_LaneBound[row_].Scd)
	{
		return 0;
	}

	//最大連續投影量起始點與結束點寬度有1/4在中間車道
	if((end - iTS->O_LaneBound[row_].Frt + 1) < width)
	{
		return 0;
	}

	//最大連續投影量起始點與結束點寬度有1/4在中間車道
	if((iTS->O_LaneBound[row_].Scd - start + 1) < width)
	{
		return 0;
	}

	return 1;
}

#ifdef O_SHOW_HORIZONTAL_PROJECTION_ON_OFF
short O_ShowHEHP(unsigned char *Dst,ITS *iTS)
{
	short row_;
	for(row_=0; row_<S_IMGCH; row_++)
	{
		memset(&Dst[(iTS->F_H-row_)*iTS->F_W + (S_IMGW - 100) ],255, 100);
		memset(&Dst[(iTS->F_H-row_)*iTS->F_W + (S_IMGW - 100) ],0, iTS->O_HD_Array[row_]);
	}
	return 1;
}
#endif


