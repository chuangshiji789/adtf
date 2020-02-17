#include "InitialVariable.h"
#include "FunctionType.h"


void SetLaneBnd(ITS *iTS)
{
	short row_,Mr,Ml,w;
	double K,M,Bl,Br,Vrow;

	short O_Mr,O_Ml,O_w;
	double O_K = 0,O_M = 0, O_Bl, O_Br, O_Vrow, O_evRodSlp = 0;

#if(SINGLE_OR_DUAL_LANE_DETECTION == SINGLE_LANE_DETECTION_PROCESS || SINGLE_OR_DUAL_LANE_DETECTION == MERGE_SINGLE_AND_DUAL_LANE_DETECTION)
	if(iTS->L_DetectMode == SL_SEARCH || iTS->L_DetectMode == SL_TRACE)
	{
		K=iTS->SL_LaneModel.K;
		M=iTS->SL_LaneModel.M;
		Bl=iTS->SL_LaneModel.LaneL.B_i;
		Br=iTS->SL_LaneModel.LaneR.B_i;
		iTS->O_evRodSlp=iTS->SL_LaneModel.evRodSlp;
        iTS->O_WAvg = iTS->SL_LaneModel.WAvg;
	}
	else if(iTS->L_DetectMode == LSEARCH || iTS->L_DetectMode == LTRACE)
	{
		K=iTS->K;
		M=iTS->M;
		Bl=iTS->LaneL.B_i;
		Br=iTS->LaneR.B_i;
		iTS->O_evRodSlp=iTS->L_evRodSlp;
		iTS->O_WAvg=iTS->L_WAvg;
	}
#else
	if(iTS->L_DetectMode == LSEARCH || iTS->L_DetectMode == LTRACE)
	{
		K=iTS->K;
		M=iTS->M;
		Bl=iTS->LaneL.B_i;
		Br=iTS->LaneR.B_i;
		iTS->O_evRodSlp=iTS->L_evRodSlp;
		iTS->O_WAvg=iTS->L_WAvg;
	}
#endif

	else
	{
		K=0;
		M=0;				
		Bl=-(double)(O_RW_DefaultRoadWidth* S_euv)/(2*S_HOCam);
		Br=+(double)(O_RW_DefaultRoadWidth* S_euv)/(2*S_HOCam);
		iTS->O_evRodSlp=iTS->L_evRodSlp;
		iTS->O_WAvg=O_RW_DefaultRoadWidth;
	}

  /*  if(iTS->L_DetectMode == LTRACE && iTS->L_StbCtr > 13 &&
#ifdef L_PROTECTION_MAIN_ON_OFF
		iTS->L_Protection_Main == PROTECTION_OFF &&
		iTS->L_UsingPerception_Main == USING_PERCEPTION_OFF &&
#endif
		(iTS->CARM.RealDistance > O_IS_TOO_CLOSE_STOP_LANE_DETECTION ||
		iTS->CARM.RealDistance == 0))
	{
		O_K=iTS->K;
		O_M=iTS->M;
		O_Bl=iTS->LaneL.B_i;
		O_Br=iTS->LaneR.B_i;
		O_evRodSlp = iTS->L_evRodSlp; 
    }
    else*/
	{
//		O_K=0;
//		O_M=0;
//		O_Bl=-(double)(O_RW_DefaultRoadWidth* S_euv)/(2*S_HOCam);
//		O_Br=+(double)(O_RW_DefaultRoadWidth* S_euv)/(2*S_HOCam);
//        O_evRodSlp = EV_ROAD_SLOPE;


        O_K=iTS->K;
        O_M=iTS->M;
        O_Bl=iTS->LaneL.B_i;
        O_Br=iTS->LaneR.B_i;
        iTS->O_evRodSlp=EV_ROAD_SLOPE;
        iTS->O_WAvg=iTS->L_WAvg;
	}

    iTS->O_LaneBound = iTS->O_LaneMBound;
	for(row_=0; row_<=S_IMGCH; row_++)
	{

        Vrow = iTS->O_evRodSlp - (row_ - iTS->F_H_C);
		Ml = LimitW(iTS->F_W_C+LaneModel(Vrow,K,M,Bl));
		Mr = LimitW(iTS->F_W_C+LaneModel(Vrow,K,M,Br));
		w = Mr-Ml+1;

		if(w<5)
		{
			iTS->L_LaneMBound[row_].Frt=iTS->L_LaneMBound[row_-1].Frt;
			iTS->L_LaneMBound[row_].Scd=iTS->L_LaneMBound[row_-1].Scd;
		}
		else
		{
			iTS->L_LaneMBound[row_].Frt=Ml;
			iTS->L_LaneMBound[row_].Scd=Mr;
		}

		if(w<10)
		{
			iTS->L_LaneLBound[row_].Scd=iTS->L_LaneLBound[row_-1].Scd;
			iTS->L_LaneLBound[row_].Frt=iTS->L_LaneLBound[row_-1].Frt;
			iTS->L_LaneRBound[row_].Frt=iTS->L_LaneRBound[row_-1].Frt;
			iTS->L_LaneRBound[row_].Scd=iTS->L_LaneRBound[row_-1].Scd;
		}
		else
		{
			w = O_GetImgLaneWidth(row_,iTS);
			iTS->L_LaneLBound[row_].Scd=Ml;
			iTS->L_LaneLBound[row_].Frt=LimitW(Ml-w);
			iTS->L_LaneRBound[row_].Frt=Mr;
			iTS->L_LaneRBound[row_].Scd=LimitW(Mr+w);
        }

		O_Vrow = O_evRodSlp - (row_ - iTS->F_H_C);
		O_Ml = LimitW(iTS->F_W_C+LaneModel(O_Vrow, O_K, O_M, O_Bl));
		O_Mr = LimitW(iTS->F_W_C+LaneModel(O_Vrow, O_K, O_M, O_Br));
		O_w = O_Mr-O_Ml+1;

		if(O_w<15)
		{	
			iTS->O_LaneMBound[row_].Frt=iTS->O_LaneMBound[row_-1].Frt;
			iTS->O_LaneMBound[row_].Scd=iTS->O_LaneMBound[row_-1].Scd;
		}
		else
		{
			iTS->O_LaneMBound[row_].Frt=O_Ml;
			iTS->O_LaneMBound[row_].Scd=O_Mr;
		}
	}
}

#ifdef L_DRAW_LAND_BOUND_ON_OFF
short L_DrawLandBnd(unsigned char *Img,ITS *iTS)
{
	unsigned char *CurRow;
	short row_;
	for(row_=0; row_<S_IMGCH; row_++)
	{
		CurRow=&Img[(iTS->F_H-1-row_)*iTS->F_W];
		CurRow[iTS->L_LaneRBound[row_].Scd]=255;
		CurRow[iTS->L_LaneLBound[row_].Frt]=255;
		CurRow[iTS->L_LaneMBound[row_].Frt]=0;
		CurRow[iTS->L_LaneMBound[row_].Scd]=0;
	}
	return 1;
}
#endif

#ifdef O_DRAW_LAND_BOUND_ON_OFF
short O_DrawLandBnd(unsigned char *Img,ITS *iTS)
{
	unsigned char *CurRow;
	short row_;
	for(row_=O_IB_BB_SearchVehicle; row_ < S_IMGCH; row_++)
	{
		CurRow=&Img[(iTS->F_H-1-row_)*iTS->F_W];
		CurRow[iTS->O_LaneMBound[row_].Frt]=0;
		CurRow[iTS->O_LaneMBound[row_].Scd]=0;
	}
	return 1;
}
#endif

short O_CheckInTrackingROI(short LB,short RB,ITS *iTS)
{
	short rt;
	if(iTS->CAR->StableCtr < O_FC_GoingStable)	
		return 0;
	if(iTS->CAR->CarInfo.Position.LB > RB)	
		return 0;
	if(iTS->CAR->CarInfo.Position.RB < LB)	
		return 0;
	if(iTS->CAR->CarInfo.Position.BW <= 0)	
		return 0;
	rt = abs(RB-LB+1-iTS->CAR->CarInfo.Position.BW);
	rt = rt*100/iTS->CAR->CarInfo.Position.BW;
	rt = 100-rt;
	if(rt > 100)	
		return 100;
	else if(rt < 0)	
		return 0;
	else 
		return rt;

}
