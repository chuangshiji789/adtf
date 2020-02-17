#include "InitialVariable.h"
#include "FunctionType.h"

void MeanSquareAddingData(double weight,double x,double y,double MatrixA[5],double VectorB[3])
{
	short i;
	for(i=0;i<5;i++)
	{
		MatrixA[i] += weight;
		if(i<3)
			VectorB[i] += weight * y;
		weight *=x;
	}
}
		
void AddLanMdl(register short weight,register short row_,register short L,register short R, ITS *iTS)
{
	double du_,u_,v_;
	if((R-L)>0)
		(du_=(R-L+1));
	else
		(du_=(L-R+1));
	u_ = (((R+L)>>1)-iTS->F_W_C) * du_;
    v_ = row_  - iTS->F_H_C-1;
    //v_ = row_ - L_IB_BB_SearchLane - iTS->F_H_C-1;
    MeanSquareAddingData(weight,du_,u_,iTS->dU,iTS->U);
    iTS->Um[0]+=weight*(((R+L)>>1)-iTS->F_W_C);
    iTS->V[0] += weight * v_;
    iTS->V[1] += weight * v_ * du_;
}

short UpdateBoth(short row_,ITS *iTS)
{
	iTS->L_Ctr_DetectedRows++;
	iTS->L_WCur = DItoDW(iTS->LaneR.CurCol-iTS->LaneL.CurCol,iTS->L_Vrow);
	iTS->L_ROI = SUB;	
	iTS->LaneL.PixCtr++;
	iTS->LaneR.PixCtr++;
	AddLanMdl(L_PW_FindBothSide, row_, iTS->LaneL.CurCol, iTS->LaneR.CurCol, iTS);

#ifdef DRAW_LANE_MARK
	iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+iTS->LaneR.CurCol]=0;
	iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+iTS->LaneL.CurCol]=0;
#endif
	return 1;
}

short UpdateOneSide(short row_,LaneInfo *Lane,ITS *iTS)
{
	short temp;
	if(Lane->Sd==-1)
		temp = Lane->CurCol + DWtoDI(iTS->L_WCur,iTS->L_Vrow) + 1;
	else
		temp = Lane->CurCol - DWtoDI(iTS->L_WCur,iTS->L_Vrow) - 1;
	if(Lane->Sd == L_RtSd)
		iTS->L_ROI = RSUB;
	else
		iTS->L_ROI = LSUB;
	if(temp<S_IMGLB || temp>S_IMGRB)
		return 0;
	Lane->PixCtr++;
	iTS->L_Ctr_DetectedRows++;
	AddLanMdl(L_PW_FindOneSide,row_,Lane->CurCol,temp,iTS);

#ifdef DRAW_LANE_MARK
	iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->CurCol]=0;
#endif
	return 1;
}

short IsWidth(ITS *iTS)
{
	short DW = DItoDW(iTS->LaneR.CurCol-iTS->LaneL.CurCol+1,iTS->L_Vrow);
    if (DW > L_RW_MaxLWInSig || DW< L_RW_MinLWInSig)
		return 0;
	return 1;
}

void L_DualLM_DecisionTree(short row_,ITS *iTS) 
{
	unsigned char Find;
	short FinRMrk;
	short FinLMrk;

#ifdef DRAW_LANE_MARKING_AND_ROAD_SURFACE_AREA
	iTS->LM_R_GrayScale_AdaptiveThresholding.ScanRow = (int) (iTS->F_H-row_ - 1) * iTS->F_W;
	iTS->LM_L_GrayScale_AdaptiveThresholding.ScanRow = (int) (iTS->F_H-row_ - 1) * iTS->F_W;
#endif

	FinRMrk = IsMark(&iTS->LaneR,row_,&iTS->YImg[(iTS->F_H-row_-1)*iTS->F_W],iTS);
	FinLMrk = IsMark(&iTS->LaneL,row_,&iTS->YImg[(iTS->F_H-row_-1)*iTS->F_W],iTS);

	Find = (FinLMrk << 1) | FinRMrk;

	if(Find>0)		
		iTS->L_CurFinalRow_ = row_;
	
	switch (Find)
	{
		case 3:		//(11) BOTH 
			if(IsWidth(iTS)) 
				UpdateBoth(row_,iTS); 
			else
				(iTS->L_ROI = MAIN);
			iTS->L_DStatus = BOTH;
			break;
		case 2:		//(10) LEFT
            if(iTS->L_ROI == SUB || iTS->L_ROI == LSUB || iTS->L_DStatus == LEFT || iTS->L_DStatus == BOTH)
                UpdateOneSide(row_,&iTS->LaneL,iTS);
			else
                (iTS->L_ROI = MAIN);
			iTS->L_DStatus = LEFT;
			break;
		case 1:		//(01) Right
            if(iTS->L_ROI == SUB || iTS->L_ROI == RSUB || iTS->L_DStatus == RIGHT || iTS->L_DStatus == BOTH)
                UpdateOneSide(row_,&iTS->LaneR,iTS);
			else
				(iTS->L_ROI = MAIN);
            iTS->L_DStatus = RIGHT;
			break;
		default:	//(00) None
			iTS->L_ROI = MAIN;
			iTS->L_DStatus = NONE;
			break;
	}
	if(FinRMrk)
		iTS->LaneR.LstCol = iTS->LaneR.CurCol;
	else
		iTS->LaneR.LstCol = 0;
	if(FinLMrk)
		iTS->LaneL.LstCol = iTS->LaneL.CurCol;
	else
		iTS->LaneL.LstCol = 0;
}

short L_DualLM_FindInitBothPoints(short row_,ITS *iTS)
{
	short FinRMrk,FinLMrk;
	SpfMainRoi_DftROI_KMB(&iTS->LaneL,iTS);
	SpfMainRoi_DftROI_KMB(&iTS->LaneR,iTS);

#ifdef DRAW_LANE_MARKING_AND_ROAD_SURFACE_AREA
	iTS->LM_R_GrayScale_AdaptiveThresholding.ScanRow = (int) (iTS->F_H-row_ - 1) * iTS->F_W;
	iTS->LM_L_GrayScale_AdaptiveThresholding.ScanRow = (int) (iTS->F_H-row_ - 1) * iTS->F_W;
#endif

	FinRMrk = IsMark(&iTS->LaneR,row_,&iTS->YImg[(iTS->F_H-row_-1)*iTS->F_W],iTS);
	FinLMrk = IsMark(&iTS->LaneL,row_,&iTS->YImg[(iTS->F_H-row_-1)*iTS->F_W],iTS);
	
	if (FinRMrk && FinLMrk)
	{
		if ( IsWidth(iTS) )
		{		
			UpdateBoth(row_,iTS);
			iTS->L_ROI = SUB;
			iTS->L_DStatus = BOTH;
			iTS->LaneL.LstCol = iTS->LaneL.CurCol;
			iTS->LaneR.LstCol = iTS->LaneR.CurCol;
            return 1;
		}
	}
	return 0;
	
}

