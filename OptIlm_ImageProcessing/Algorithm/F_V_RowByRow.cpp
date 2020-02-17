#include "InitialVariable.h"
#include "FunctionType.h"

#define MODE_UP        1
#define MODE_DOWN     -1
#define DOUBLE_SIGNED  0

short O_SetDefaultObsH(short row_,ITS *iTS)
{
	short H;
	H = O_DWtoDI(O_RH_DefaultCarHeigth,row_,iTS);
 	if(H<0)
		H =0;
	return H;
}

short O_GetImgLaneWidth(short row_,ITS *iTS)
{
	// Using the function must set the O_WAvg and O_LaneBound!!!!!
	short w=0;
	if(iTS->O_LaneBound[row_].Scd>=S_IMGRB || iTS->O_LaneBound[row_].Frt<=S_IMGLB)
		w = O_DWtoDI((short)(iTS->O_WAvg),row_,iTS);
	else
		w = iTS->O_LaneBound[row_].Scd - iTS->O_LaneBound[row_].Frt+1;
	if(w<=0)
		w=1;
	return w;
}

short O_FeatureExtraction_WholeSurface(short row_,short RefIW,ITS *iTS)
{
		// Set 0 Set detect region//
		short LB,RB,TB;
		short p100H = O_SetDefaultObsH(row_,iTS);
		TB = O_GetImgLaneWidth(row_,iTS);
		LB = iTS->O_LaneBound[row_].Frt;
		RB = iTS->O_LaneBound[row_].Scd;

		//根據距離設定左右邊搜尋範圍慎廷論文4.2.2節
		if(O_GetDistance(row_,iTS) < O_RD_SetMore25W)
		{
			if(iTS->CAR->CarSign != 1)
				LB = LimitW(LB - (TB >> 2));
			if(iTS->CAR->CarSign != -1)
				RB = LimitW(RB + (TB >> 2));
		}
		else if(O_GetDistance(row_,iTS) < O_RD_SetMore12W)
		{
			if(iTS->CAR->CarSign != 1)
				LB = LimitW(LB - (TB >> 3));
			if(iTS->CAR->CarSign != -1)
				RB=LimitW(RB + (TB >> 3));
		}
		else
		{
			if(iTS->CAR->CarSign != 1)
				LB = LimitW(LB - (TB >> 3));
			if(iTS->CAR->CarSign != -1)
				RB=LimitW(RB + (TB >> 3));
		}

		if(iTS->have_corner_row == row_)
		{
			LB = iTS->left_corner_mean - ((iTS->right_corner_mean - iTS->left_corner_mean) >> 1);
			RB = iTS->right_corner_mean + ((iTS->right_corner_mean - iTS->left_corner_mean) >> 1);
		}


		//Tracking Mode
#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
		if(RefIW!=0)
#else
		if(RefIW!=0)
#endif

		{
			if(LB > iTS->CAR->CarInfo.Position.LB)
				LB = iTS->CAR->CarInfo.Position.LB;
			else
				LB = LB;
			if(RB < iTS->CAR->CarInfo.Position.RB)
				RB = iTS->CAR->CarInfo.Position.RB;
			else
				RB = RB;
		}

		//Vertical Edge Pair Group
		TB = LimitH(row_+p100H);

		//畫出可能是候選物件的位置
#ifdef O_DRAW_VEHICLE_CANDIDATE
		if(iTS->have_corner_row == row_)
			OSD_Color_Setup(OCN_YELLOW, iTS);
		else if(iTS->O_CL_Array[row_] == 100)
			OSD_Color_Setup(OCN_BLUE, iTS);
		else
			OSD_Color_Setup(OCN_RED, iTS);
		DrawRectangle(row_, TB, LB , RB, iTS);
#endif


		iTS->O_VE_VP_PairGroupNum = 0;
		if(0 == O_VE_VP_SetArray(row_,TB,LB,RB,iTS))
			return 0;

		if(RefIW!=0)
			O_ModifyBoundary_TROI(iTS);
		O_VE_VP_FindPairGroupFlow(row_,LB,RB,RefIW,iTS);

		//Road Shadow Road Light
		TB = LimitH(row_+p100H);

		P_O_CreateRtgRoadLightShadowInfo(row_,iTS->O_CarLightTH,iTS->O_SD_ShadowTh,row_,TB,LB,RB,iTS);

		TB = O_GetRow_(O_GetDistance(row_,iTS)+O_RL_RegionToCheckBSD,iTS);
		O_CheckBridgeShadow(row_,TB,iTS);
		TB = LimitH(row_+(p100H>>1));
		O_ProjectRoadShadow(row_,TB,LB,RB,iTS);

		return 1;
}

short O_FeatureExtraction_InBlock(short row_,ITS *iTS)
{
		// Set 0 Set detect region//
		short iPG=0;
		short TB;
		short LB,RB;

#ifdef O_SHOW_SHADOW_VERTICAL_PROJECTION_ON_OFF
		short max = 0;
		short weight = 0;
#endif    //O_SHOW_SHADOW_VERTICAL_PROJECTION_ON_OFF

		if(iTS->O_VE_VP_PairGroupNum > 2)
		{
			return 0;
		}
		for(iPG=0; iPG<iTS->O_VE_VP_PairGroupNum; iPG++)
		{
			LB=iTS->O_VE_VP_PairGroupArray[iPG].Frt;
			RB=iTS->O_VE_VP_PairGroupArray[iPG].Scd;

			if(iTS->have_corner_row == row_)
			{
				LB = iTS->left_corner_mean ;
				RB = iTS->right_corner_mean;
			}

			TB=LimitH(row_+(((RB-LB+1)*3)>>2));//可以改為依據寬度調整高低

			iTS->O_FZ_PVEInfo[iPG][CL_]=(char)O_CheckCarLightInPairVE(row_,TB,LB,RB,iTS);		//車燈特徵數量
			iTS->O_FZ_PVEInfo[iPG][VE_]=(char)O_CheckVEDensityInPairVE(row_,TB,LB,RB,iTS);		//垂直邊緣密度
			TB=LimitH(row_+((RB-LB+1)>>2));
			iTS->O_FZ_PVEInfo[iPG][BB_]=(char)O_CheckAllDarkLightInPairVE(row_,TB,LB,RB,iTS);	//區塊陰影密度
			iTS->O_FZ_PVEInfo[iPG][HE_]=(char)O_CheckHEDensityInPairVE(row_,TB,LB,RB,iTS); 		//水平邊緣密度
			iTS->O_FZ_PVEInfo[iPG][SD_]=(char)O_CheckShadowInPariVE(row_,LB,RB,iTS);			// 陰影分布程度
			iTS->O_FZ_PVEInfo[iPG][IR_]=(char)O_CheckInTrackingROI(LB,RB,iTS);					//追蹤重疊程度

#ifdef O_SHOW_SHADOW_VERTICAL_PROJECTION_ON_OFF
			max = O_FindMax(iTS->O_SD_VerPrjArray,0,352,&max);
			weight=O_NormalizeArray(iTS->O_SD_VerPrjArray,0,352,max);
#endif    //O_SHOW_SHADOW_VERTICAL_PROJECTION_ON_OFF

		}
	return 1;
}


//#define DRAW_SEARCHIN_BOUNDARY_IN_ROW_BY_ROW
short O_Search_RowByRow(short BB,short TB,ITS *iTS)
{
	short row_ = 0;
	short FinalResult = 0;

	//如果有Corner直接先對Corner的候選物件做車輛偵測流程，若有偵測到下面的就不用做了。
	if(iTS->have_corner_row != 0)
	{
		iTS->O_HD_Array[row_] = 50;//debug use

		if(O_FeatureExtraction_WholeSurface(iTS->have_corner_row, 0, iTS) == 0)
			return 0;

		O_FeatureExtraction_WholeSurface(iTS->have_corner_row, 0, iTS);
		if(!O_FeatureExtraction_InBlock(iTS->have_corner_row, iTS))
			return 0;
		FinalResult = O_FuzzySystem_row(iTS->have_corner_row, iTS);
		//OSD_Color_Setup(OCN_PURPLE, iTS);
		//ScalableNumber(101, 4, 50, 50, iTS);
		if(FinalResult == 1 || FinalResult == 2 || FinalResult == 3 || FinalResult == 4)
		{
			return 1;
		}
	}

	if(iTS->first_search_row != 0)
	{
		BB = LimitSearchingBoundary(iTS->first_search_row, iTS);

#ifdef DRAW_SEARCHIN_BOUNDARY_IN_ROW_BY_ROW
		memset(&iTS->Showimage[(S_IMGH - BB) * S_IMGW], 128, S_IMGW);
		memset(&iTS->Showimage[(S_IMGH - TB) * S_IMGW], 0, S_IMGW);
#endif

		//根據車燈與水平投影量做車輛偵測
		for(row_ = BB; row_ <= TB; row_++)
		{
			FinalResult = 0;

			if(iTS->O_HD_Array[row_] < 100 && iTS->O_CL_Array[LimitVP(row_)] < 100)
				continue;

			if(P_AddEffortAndCheck(iTS->O_VE_VP_PairGroupNum, iTS) == 1)
				return 0;

			if(O_FeatureExtraction_WholeSurface(row_, 0, iTS) == 0)
				continue;

			if(!O_FeatureExtraction_InBlock(row_, iTS))
				return 0;
			FinalResult = O_FuzzySystem_row(row_, iTS);
			if(FinalResult == 1 || FinalResult == 2 || FinalResult == 3 || FinalResult == 4)
			{
				FinalResult = 1;
				break;
			}
			else
			{
				continue;
			}
		}
	}
	return FinalResult;
}

short O_InRowByRow(short row_,ITS *iTS)
{
	if(iTS->O_HD_Array[LimitVP(row_)] < 100 && iTS->O_CL_Array[LimitVP(row_)] < 100 && row_ != iTS->have_corner_row)
		return 0;

	if(P_AddEffortAndCheck(iTS->O_VE_VP_PairGroupNum,iTS)==1)
		return 0;

	if(	iTS->CAR->RealDistance < O_RD_CanEnhanceBoundry	&& iTS->CAR->StableCtr > O_FC_GoingStable)
	{
		if(O_FeatureExtraction_WholeSurface(row_,iTS->CAR->CarInfo.Position.BW,iTS) == 0)
			return 0;
	}
	else
	{
		if(O_FeatureExtraction_WholeSurface(row_,0,iTS) == 0)
			return 0;
	}
	if(!O_FeatureExtraction_InBlock(row_,iTS))
		return 0;
	return O_FuzzySystem_row(row_,iTS);
}

//#define DRAW_SEARCH_ROW
//#define SHOW_NUMBER_FOR_SEARCHING_ROW
short ForSearch(short Mode,short row_,short d,short Margin,ITS *iTS)
{
	//Mode = 1 up, Mode = -1 Down, Mode = 0 Doboule signed!
	short i=0;
	short HR;

#ifdef SHOW_NUMBER_FOR_SEARCHING_ROW
	OSD_Color_Setup(OCN_PURPLE, iTS);
	ScalableNumber((short) Margin, 4, S_IMGCW - 20, 150, iTS);
	OSD_Color_Setup(OCN_RED, iTS);
	ScalableNumber((short) row_, 4, S_IMGCW - 20, 180, iTS);
#endif

//20111017 Jeremy Lee Modify
	while(row_ < iTS->V_serching_boundary && row_ > S_IMGBB && i<= Margin)
	{
//20111017 Jeremy Lee Modify
#ifdef DRAW_SEARCH_ROW
		if(Mode==1)
			memset(&iTS->Showimage[(S_IMGH - row_) * S_IMGW], 255, S_IMGW);
		else if(Mode==-1)
			memset(&iTS->Showimage[(S_IMGH - row_) * S_IMGW], 128, S_IMGW);
		else
			memset(&iTS->Showimage[(S_IMGH - row_) * S_IMGW], 0, S_IMGW);
#endif
//20111017 Jeremy Lee Modify

		HR=0;
		if(row_>0)
			HR=O_InRowByRow(row_,iTS);

		if(HR==1 || HR==2)
			HR=V_VehicleCompare(&iTS->CAR->CarInfo,&iTS->V_FZCar,O_ID_Area_TrackingRegion,O_ID_VFill_TrackingRegion,O_ID_HFill_TrackingRegion);
		else
			HR = 0;

		if(HR == 1)
		{
			return 1;
		}
		i++;
		if(Mode==1)
			row_++;
		else if(Mode==-1)
			row_--;
	}

	return 0;
}

short O_Tracking_RowByRow(short row_,short Margin,ITS *iTS)
{
	//Mode = 1 up, Mode = -1 Down, Mode = 0 Doboule signed!

	if((iTS->CAR->CarInfo.Ctype == Day || iTS->CAR->CarInfo.Ctype == BigBlack || iTS->CAR->CarInfo.Ctype == CarCorner) && iTS->CAR->StableCtr >= L_FC_GoingStable)
		if(1==ForSearch(DOUBLE_SIGNED, row_, -1, 1, iTS))
			return 1;

	if(1==ForSearch(MODE_UP, row_, 0,Margin >> 1, iTS))
		return 1;
	if(1==ForSearch(MODE_DOWN, row_, 0, Margin >> 1, iTS))
		return 1;

	return 0;
}



