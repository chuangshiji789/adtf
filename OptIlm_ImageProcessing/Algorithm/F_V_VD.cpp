#include "InitialVariable.h"
#include "FunctionType.h"

//#define DRAW_TEST_AREA
#define JUMP_LANE_MARK		30
short O_TestVEDensity(short Srow_,short Erow_,ITS *iTS)
{
	short LB,RB;
	short EDC=0;
	short row_,col;
	int	count=0;
	int	total=0;
	int r=0;
	unsigned char *CrRow;
	int Dcount=0;

	//20111007 Samuel and Jeremy Modify
	unsigned int edge_counter = 0;
	unsigned int tail_light_counter = 0;
	unsigned int edge_appear = 0;
	unsigned int tail_appear = 0;
	//20111007 Samuel and Jeremy Modify


	Srow_ = LimitVP(Srow_);
	Erow_ = LimitVP(Erow_);

	if(iTS->O_LaneBound[Srow_].Frt < iTS->O_LaneBound[Erow_].Frt)
		LB = iTS->O_LaneBound[Srow_].Frt;
	else
		LB = iTS->O_LaneBound[Erow_].Frt;
	if(iTS->O_LaneBound[Srow_].Scd > iTS->O_LaneBound[Erow_].Scd)
		RB = iTS->O_LaneBound[Srow_].Scd;
	else
		RB = iTS->O_LaneBound[Erow_].Scd;

	if(RB - LB < 5)
		return 0;
	memset(&iTS->O_VE_VP_Array[0], 0, S_IMGW * sizeof(short));

	//20111007 Samuel and Jeremy Modify
#ifdef DRAW_TEST_AREA
	memset(&iTS->Showimage[(S_IMGH - iTS->V_tail_edge_detection_top_boundary) * S_IMGW], 128, S_IMGW * sizeof(short));
	if(iTS->V_tail_edge_one_time_flag == 0 && Srow_ < iTS->V_tail_edge_detection_top_boundary)
	{
		EDC = O_DWtoDI(JUMP_LANE_MARK,Srow_,iTS);
		LB = LimitW(iTS->O_LaneBound[Srow_].Frt - EDC);
		RB = LimitW(iTS->O_LaneBound[Srow_].Scd + EDC);
		memset(&iTS->Showimage[(S_IMGH - Srow_) * S_IMGW + LB], 255, (RB - LB) * sizeof(char));
	}
#endif
	//20111007 Samuel and Jeremy Modify

	r = SinkDataIndexStart(Srow_);
	for(row_ = Srow_; row_ <= ((Srow_ + Erow_) >> 1); row_++)
	{
		r = SinkDataIndexNextRow(r);
		CrRow =& iTS->O_InfoPlane[r];

		EDC = O_DWtoDI(JUMP_LANE_MARK,row_,iTS);

		LB = LimitW(iTS->O_LaneBound[row_].Frt - EDC);
		RB = LimitW(iTS->O_LaneBound[row_].Scd + EDC);

		//20111007 Samuel and Jeremy Modify
#ifdef DRAW_TEST_AREA
		if(iTS->V_tail_edge_one_time_flag == 0 && Srow_ < iTS->V_tail_edge_detection_top_boundary)
		{
			iTS->Showimage[(S_IMGH - row_) * S_IMGW + LB] = 255;
			iTS->Showimage[(S_IMGH - row_) * S_IMGW + RB] = 255;
		}
#endif
		//20111007 Samuel and Jeremy Modify

		for(col = LB; col <= RB; col++)
		{
			//20111007 Samuel and Jeremy Modify
			if(iTS->V_tail_edge_one_time_flag == 0 && Srow_ < iTS->V_tail_edge_detection_top_boundary)
			{
				if((CrRow[col] & HEVEINFO) == HEVEINFO)
					edge_counter ++;

				if((CrRow[col] & RLTINFO) == RLTINFO)
					tail_light_counter ++;
			}
			//20111007 Samuel and Jeremy Modify

			total++;
			if(	(CrRow[col]&HEVEINFO) == VEINFO )
			{
				iTS->O_VE_VP_Array[col]+=1;
				count++;
			}
			if(	(CrRow[col]&HEVEINFO) != 0x00 )
			{
				Dcount++;
			}

		}
	}

	//20111007 Samuel and Jeremy Modify
	if(iTS->V_tail_edge_one_time_flag == 0 && Srow_ < iTS->V_tail_edge_detection_top_boundary)
	{
		edge_appear = (unsigned int) (100 * ((double) edge_counter / (double) total));
		tail_appear = (unsigned int) (100 * ((double) tail_light_counter / (double) total));

#ifdef DRAW_TEST_AREA
		OSD_Color_Setup(OCN_PURPLE, iTS);
		ScalableNumber((short) edge_appear, 4, S_IMGCW - 30 , 200, iTS);
		ScalableNumber((short) tail_appear, 4, S_IMGCW + 20, 200, iTS);
#endif
		iTS->V_tail_edge_one_time_flag = 1;

		if((edge_appear > 5 && edge_appear < 20) && (tail_appear > 30 && tail_appear < 50))
		{
			iTS->V_tail_edge_found_row = Srow_;

#ifdef DRAW_TEST_AREA
			OSD_Color_Setup(OCN_GREEN, iTS);
			DrawRect_Has_UV(200, S_IMGCW - 70, 20, iTS);
#endif

		}
		else
			iTS->V_tail_edge_found_row = 0;
	}
	//20111007 Samuel and Jeremy Modify

	if(iTS->O_LaneBound[Srow_].Frt < iTS->O_LaneBound[Erow_].Frt)
		LB = iTS->O_LaneBound[Srow_].Frt;
	else
		LB = iTS->O_LaneBound[Erow_].Frt;
	if(iTS->O_LaneBound[Srow_].Scd > iTS->O_LaneBound[Erow_].Scd)
		RB = iTS->O_LaneBound[Srow_].Scd;
	else
		RB = iTS->O_LaneBound[Erow_].Scd;

	EDC = (((Erow_ - Srow_ + 1) >> 2) + 1);
	for(col = LB; col <= RB; col++)
	{
		iTS->O_VE_VP_Array[col] += iTS->O_VE_VP_Array[col+1];
		if(iTS->O_VE_VP_Array[col] >= EDC)
			return 1;
	}
	if(total <= (O_IA_FewTotal >> 1))
		return 0;
	Dcount = Dcount * 100 / total;
	if(Dcount >= O_ID_RoadHEVE)
		return 1;

	count = count * 100 / total;
	if(count <= O_ID_RoadVE)
		return 0;
	else
		return 1;

}

//#define DRAW_SET_ARRAY
short O_VE_VP_SetArray(short Srow_,short Erow_,short Scol,short Ecol,ITS *iTS)
{
	short col,row_;
	unsigned char *CrRow;
	short weight;
	int r;

	iTS->O_VE_VP_ArrPtr.MaxValue=0;

	if(O_TestVEDensity(Srow_,Erow_,iTS)==0)
		return 0;

	memset(&iTS->O_VE_VP_Array[Scol],0,(Ecol-Scol+1)*sizeof(short));

	weight=(Erow_-Srow_+1);

	r=SinkDataIndexStart(Srow_);

#ifdef DRAW_SET_ARRAY
	OSD_Color_Setup(OCN_BLUE, iTS);    //畫出上列描述式運算範圍
	DrawRectangle(Srow_, Erow_, Scol, Ecol, iTS);
#endif

	for(row_=Srow_; row_<=Erow_; row_++)
	{
		r=SinkDataIndexNextRow(r);
		CrRow =&iTS->O_InfoPlane[r];

		for(col=Scol; col<=Ecol; col++)
		{
			if(	(CrRow[col]&HEVEINFO) == VEINFO )
				iTS->O_VE_VP_Array[col]+=weight;
		}
		if(--weight<1)
			weight=1;
	}
	iTS->O_VE_VP_ArrPtr.MaxValue = O_FindMax(iTS->O_VE_VP_Array,Scol,Ecol,&iTS->O_VE_VP_ArrPtr.MaxIndex);
	if(iTS->O_VE_VP_ArrPtr.MaxValue==0)
		return 0;

	weight=O_NormalizeArray(iTS->O_VE_VP_Array,Scol,Ecol,iTS->O_VE_VP_ArrPtr.MaxValue);
	if(weight==0)
	{
		iTS->O_VE_VP_ArrPtr.MaxValue=0;
		return 0;
	}

#ifdef O_DRAW_VE_VP_DETECT_AREA
	CrRow = &iTS->Showimage[(iTS->F_H-Srow_)*iTS->F_W];
	memset(&CrRow[Scol],50,Ecol-Scol);
	for(row_=Srow_; row_<=Erow_; row_++)
	{
		CrRow = &iTS->Showimage[(iTS->F_H-row_)*iTS->F_W];

		CrRow[Scol]=50;
		CrRow[Ecol]=50;
	}
#endif

	return 1;
}

#ifdef O_SHOW_VERTICAL_PROJECTION_ON_OFF
short O_ShowVEVP(unsigned char *Dst,ITS *iTS)
{
	const unsigned char color=255;
	short col,row_;
	for(col = 0; col < iTS->F_W; col++)
	{
		for(row_ = iTS->F_H - 70; row_ < iTS->F_H; row_++)
		{
			if((iTS->O_VE_VP_Array[col] >> 1) - (row_-(iTS->F_H - 70)) > 0)
				Dst[(iTS->F_H-1-row_) * iTS->F_W + col] = 128;
			else
				Dst[(iTS->F_H-1-row_) * iTS->F_W + col] = color;
		}
	}
	return 1;
}
#endif

void O_VE_VP_FindPairGroupFlow(short Srow_,short Scol,short Ecol,short RefIW,ITS *iTS)
{
	short th;
	short MaxW,MinW;
	short EDC;
	FORWARD_CROI O_VE_VP_SingleGroupArray[N_MaxSingleGroup];
	short O_VE_VP_SingleGroupNum=0;

	iTS->O_VE_VP_PairGroupNum=0;
	if(iTS->O_VE_VP_ArrPtr.MaxValue==0)
		return ;

	EDC=O_DWtoDI(O_IW_EDC_SingalGroup,Srow_,iTS);
	MaxW = S_IMGW;
	MinW = O_DWtoDI(O_RW_MinCarWidth,Srow_,iTS);

	for(th = O_TH_PVE_First; th >= 20; th -= O_TH_PVE_Interval)
	{
		O_VE_VP_SingleGroupNum = O_FindSingleGroup(th, EDC, 0, Scol, Ecol, iTS->O_VE_VP_Array, O_VE_VP_SingleGroupArray);
		if(O_VE_VP_SingleGroupNum >= (N_MaxSingleGroup >> 1))
			return ;

		if(O_VE_VP_SingleGroupNum>=2)
		{
			iTS->O_VE_VP_PairGroupNum=O_FindPairGroup(MinW,MaxW,O_VE_VP_SingleGroupArray,O_VE_VP_SingleGroupNum,iTS->O_VE_VP_PairGroupArray);
			if(iTS->O_VE_VP_PairGroupNum>=1)
				break;
		}
	}

#ifdef O_DRAW_PAIR_GROUP
	if(iTS->O_VE_VP_PairGroupNum>=1)
	{
		REC_ OP;
		short i;
		for(i=0; i<iTS->O_VE_VP_PairGroupNum; i++)
		{
			OP.BB = LimitH(Srow_-(Ecol-Scol)+(short)((Ecol-Scol)*((double)(i)/iTS->O_VE_VP_PairGroupNum)));
			OP.TB = Srow_;
			OP.BH = OP.TB-OP.BB+1;

			OP.LB = iTS->O_VE_VP_PairGroupArray[i].Frt;
			OP.RB = iTS->O_VE_VP_PairGroupArray[i].Scd;
			OP.BW = OP.RB - OP.LB;

			DrawRec(OP,0,iTS->Showimage,iTS);
		}
	}
#endif

}


