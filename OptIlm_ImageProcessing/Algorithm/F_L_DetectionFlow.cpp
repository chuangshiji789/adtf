#include "InitialVariable.h"
#include "FunctionType.h"

void RowSchRst(ITS *iTS)
{
	iTS->L_ROI = MAIN;
	iTS->L_DStatus = NONE;
	iTS->L_Ctr_DetectedRows = 0;
	iTS->LaneR.PixCtr =0;
	iTS->LaneL.PixCtr =0;
	iTS->LaneR.LstCol=0;
	iTS->LaneL.LstCol=0;
    iTS->LaneL.mark_width = 0;
    iTS->LaneR.mark_width = 0;
}

short DWtoDI(double DW,double Vrow_)
{
	return (short) ( S_euv * Vrow_ * DW / S_HOCam + TRANSFER_ERROR );
}

//short DItoDW(short DI,double Vrow_)
//{
//	return (short) ( S_evu * DI * S_HOCam / Vrow_ + TRANSFER_ERROR);
//}
/*short DWtoDI(short DW,double Vrow_)
{
	return (short) ( S_euv * Vrow_ * (DW+0) / S_HOCam);
}*/
double DItoDW(double DI,double Vrow_)
{
    return ( S_evu * DI * S_HOCam / Vrow_ + TRANSFER_ERROR);
}

short L_DualLM_Searching(ITS *iTS)
{
	short FindRow_=-1;
	short row_;
	RowSchRst(iTS);
    for (row_ = L_IB_BB_SearchLane; row_ < L_IB_TB_SearchLane; row_++)
	{
		iTS->L_Vrow = iTS->L_evRodSlp - (row_ - iTS->F_H_C);

		if(L_DualLM_FindInitBothPoints(row_,iTS))
		{
            FindRow_=row_;
            break;
		}
	}
    if(FindRow_ <= L_IB_BB_SearchLane)
		return 0;
	for (row_ = FindRow_+1; row_ < L_IB_TB_SearchLane; row_++) 
	{
		iTS->L_Vrow = iTS->L_evRodSlp - (row_ - iTS->F_H_C);
        if (iTS->L_Ctr_DetectedRows >= L_IP_DetectedRowSetROIwithLM)
		{
			if(!L_DualLM_Estimation(iTS))
            {
                return 0;
            }
            if (iTS->L_Ctr_DetectedRows >= 30)
				SpecifyROI(SpfMainRoi_CtnRoi_KMB,iTS);
            else
				SpecifyROI(SpfMainRoi_DftROI_KMB,iTS);
		}
		else
			SpecifyROI(SpfMainRoi_DftROI_KMB,iTS);
		
		L_DualLM_DecisionTree(row_,iTS);
	}
	iTS->L_LastFinalRow_=iTS->L_CurFinalRow_;
    if(iTS->L_Ctr_DetectedRows < L_IP_DetectedRowPromoteToCtn)
    {
        iTS->dual_lane_debug = 1;
        return 0;
    }
    if(iTS->LaneL.PixCtr<L_IP_LinePixelPromoteToCtn || iTS->LaneR.PixCtr<L_IP_LinePixelPromoteToCtn)
    {
        iTS->dual_lane_debug = 2;
        return 0;
    }
    if(!L_DualLM_Estimation(iTS))
    {
        return 0;
    }
    if(fabs(iTS->bm) > (iTS->L_WAvg - (iTS->L_WAvg>>2)))
    {
        iTS->dual_lane_debug = 3;
        return 0;
    }
    else
    {
        return 1;
    }
}

short L_DualLM_Tracking(ITS *iTS)
{   
	short FinLan,Final_row;
	short row_,colL,	colR;
	RowSchRst(iTS);
	iTS->U[2] /= L_PW_ReserveLaneModel;
	for(row_=0;row_<5;row_++)
	{
		iTS->dU[row_] /= L_PW_ReserveLaneModel;
		if(row_<2)
		{
			iTS->U[row_] /= L_PW_ReserveLaneModel;
			iTS->V[row_] /= L_PW_ReserveLaneModel;
		}
	}
	iTS->Um[0] /= L_PW_ReserveLaneModel;

	Final_row=L_IB_TB_TrackingLane;

	for ( row_ = L_IB_BB_TrackingLane; row_ < Final_row; row_++)
	{
		iTS->L_Vrow = iTS->L_evRodSlp - (row_ - iTS->F_H_C);
		SpecifyROI(SpfMainRoi_CtnRoi_KMB,iTS);
		L_DualLM_DecisionTree(row_,iTS);
	}
	iTS->L_LastFinalRow_=iTS->L_CurFinalRow_;
  
	FinLan = 0x0000;
	if(iTS->LaneL.PixCtr>= L_IP_MaintainInCtn_LossOneSide )
		FinLan |= (L_FinLan<<1);
	if(iTS->LaneR.PixCtr>= L_IP_MaintainInCtn_LossOneSide)
		FinLan |= (L_FinLan);
	if(iTS->L_Ctr_DetectedRows < L_IP_DetectedRowPromoteToCtn)
	{
        if(iTS->L_StbCtr<L_FC_GoingStable)
        {
            iTS->dual_lane_debug = 7;
            return 0;
        }
		if(iTS->L_Ctr_LossBothLane > L_FC_LossBothLaneInCtn)
        {
            iTS->dual_lane_debug = 8;
            return 0;
        }
		else
		{
			FinLan = 0x0000;
			iTS->L_Ctr_LossBothLane=LimitCount(++iTS->L_Ctr_LossBothLane,30);
		}
	}
	else
	{
		iTS->L_Ctr_LossBothLane=LimitCount(--iTS->L_Ctr_LossBothLane,30);
		if(!(FinLan==3))
		{
            if(iTS->L_StbCtr<L_FC_GoingStable)
            {
                iTS->dual_lane_debug = 9;
                return 0;
            }
			if(iTS->L_WAvg >L_RW_MaxLWInLossOneSide || iTS->L_WAvg<L_RW_MinLWInLossOneSide)
            {
                iTS->dual_lane_debug = 10;
                return 0;
            }
			if(iTS->L_Ctr_LossSingleLane > L_FC_LossOneLaneInCtn)
            {
                iTS->dual_lane_debug = 11;
                return 0;
            }
			iTS->L_Ctr_LossSingleLane=LimitCount(++iTS->L_Ctr_LossSingleLane,30);
		}
		else
			iTS->L_Ctr_LossSingleLane=LimitCount(--iTS->L_Ctr_LossSingleLane,30);
	}
	if(!L_DualLM_Estimation(iTS))
		return 0;

    iTS->L_Vrow = iTS->L_evRodSlp - (L_IB_BB_TrackingLane - iTS->F_H_C);

    colL = iTS->F_W_C + LaneModel(iTS->L_Vrow, iTS->K, iTS->M, iTS->LaneL.B_i);
	colR = iTS->F_W_C + LaneModel(iTS->L_Vrow, iTS->K, iTS->M, iTS->LaneR.B_i);

    if(colL>(iTS->F_W_C+(iTS->F_W>>1)) || colR<(iTS->F_W_C-(iTS->F_W>>1)))
    {
        iTS->dual_lane_debug = 12;
        return 0;
    }
    else
        return 1;

}

