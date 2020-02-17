#include "InitialVariable.h"
#include "FunctionType.h"

void SetROI(FORWARD_CROI *roi, short Rmin, short Rmax, short last, short d) 
{
	if (last-d < Rmin)
		roi->Frt = Rmin; 
	else if (last-d <= Rmax)
		roi->Frt = last-d;
	else
		roi->Frt = Rmax;

	if (last+d > Rmax)
		roi->Scd = Rmax;
	else if (last+d >= Rmin)
		roi->Scd = last+d;
	else
		roi->Scd = Rmin;
}

void SpfMainRoi_DftROI_KMB(LaneInfo *Lane,ITS *iTS)
{	//	u_ = K/Vrow + M + B*Vrow
	double dBU;
	double P;

    if(iTS->SL_LaneModel.L_StbCtr > 15)    //11072017
        P = iTS->F_W_C + LaneModel(iTS->L_Vrow, iTS->SL_LaneModel.K, iTS->SL_LaneModel.M, Lane->B_i);
    else
        P = iTS->F_W_C + LaneModel(iTS->L_Vrow, iTS->K, iTS->M, Lane->B_i);

	dBU = iTS->L_Vrow * L_RW_ROILRegion / S_HOCam * S_euv;	

	if(Lane->Sd==1)	//right
        SetROI(&Lane->SchRoi,(S_IMGCW>>2), S_IMGRB-1,(short)P,(short)(dBU));
	else			//Left
        SetROI(&Lane->SchRoi,S_IMGLB, (S_IMGCW + (S_IMGCW>>2)),(short)P,(short)(dBU));

#ifdef DRAW_LANE_MARK_ROI
	//Draw ROI Debug
	short row_;
	row_ = (short)(iTS->L_evRodSlp -iTS->L_Vrow) +iTS->F_H_C;
        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Frt]=255;
        iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+Lane->SchRoi.Scd]=255;
#endif
}

void SpfMainRoi_CtnRoi_KMB(LaneInfo *Lane,ITS *iTS)
{
	short col,range;
	col = (short) ( iTS->F_W_C + LaneModel(iTS->L_Vrow, iTS->K, iTS->M, Lane->B_i) );
    //if(iTS->L_LaneModelType==CURVE)
    //	range=DWtoDI(iTS->L_WCur>>3,iTS->L_Vrow);
    //else
		range=DWtoDI(iTS->L_WCur>>2,iTS->L_Vrow);

	SetROI(&Lane->SchRoi, S_IMGLB, S_IMGRB, col, range);
}

void SpecifyROI(SpecifyMainROIFunc *SpecifyMainROI,ITS *iTS	)
{
	short MarkWidth =  DWtoDI(L_RW_DefaultMarkW,iTS->L_Vrow);
	switch (iTS->L_ROI)
    {
		case MAIN:
			(*SpecifyMainROI)(&iTS->LaneL,iTS);
			(*SpecifyMainROI)(&iTS->LaneR,iTS);
			break;
		case SUB:
			if(iTS->LaneL.LstCol > 0)
				SetROI(&iTS->LaneL.SchRoi,S_IMGLB, S_IMGRB,iTS->LaneL.LstCol, MarkWidth);
			else
				(*SpecifyMainROI)(&iTS->LaneL,iTS);
			if(iTS->LaneR.LstCol > 0)
				SetROI(&iTS->LaneR.SchRoi,S_IMGLB, S_IMGRB,iTS->LaneR.LstCol, MarkWidth);
			else
				(*SpecifyMainROI)(&iTS->LaneR,iTS);
			break;
		case LSUB:
			(*SpecifyMainROI)(&iTS->LaneR,iTS);
			if(iTS->LaneL.LstCol > 0)
				SetROI(&iTS->LaneL.SchRoi,S_IMGLB, S_IMGRB,iTS->LaneL.LstCol, MarkWidth);
			else
				(*SpecifyMainROI)(&iTS->LaneL,iTS);
			break;
		case RSUB:
			(*SpecifyMainROI)(&iTS->LaneL,iTS);
			if(iTS->LaneR.LstCol > 0)
				SetROI(&iTS->LaneR.SchRoi,S_IMGLB, S_IMGRB,iTS->LaneR.LstCol, MarkWidth);
			else
				(*SpecifyMainROI)(&iTS->LaneR,iTS);
			break;
	}

#ifdef DRAW_LANE_MARK_ROI

	short row_;
	row_ = (short)(iTS->L_evRodSlp -iTS->L_Vrow) +iTS->F_H_C;
	iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+iTS->LaneL.SchRoi.Frt]=0;
	iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+iTS->LaneL.SchRoi.Scd]=0;
	iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+iTS->LaneR.SchRoi.Frt]=0;
	iTS->Showimage[(iTS->F_H-1-row_)*iTS->F_W+iTS->LaneR.SchRoi.Scd]=0;
#endif
}
