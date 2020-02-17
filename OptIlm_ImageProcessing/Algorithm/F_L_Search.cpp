#include "InitialVariable.h"
#include "FunctionType.h"

short MaxL(unsigned char *pSrc,  short L,  short R)
{
	short MaxIndex = L;
	short i;

	for ( i = L; i <= R; ++i)
	{
		if((pSrc[i] & VEINFO) == VEINFO)
		{
			MaxIndex = i;
			return MaxIndex;
		}
	}
	return MaxIndex;
}

short MaxR(unsigned char *pSrc,  short L,  short R)
{
	short MaxIndex = R;
	short i;

	for ( i = R; i >= L; --i)
	{
		if((pSrc[i] & VEINFO) == VEINFO)
		{
			MaxIndex = i;
			return MaxIndex;
		}
	}
	return MaxIndex;
}

UBYTE Mean(const UBYTE *pSrc, short L, short R)
{
	int val = 0;
	short i;
	if (R-L+1 <= 0)
		return 0;

	for (i = L; i <= R; ++i)
		val += (int)pSrc[i];

	return (UBYTE)( (double)(val+0.5) / (R-L+1) );
}

short MarkSearch(short index,short row_,LaneInfo *Lane,short maxMW,short minMW,UBYTE *pSrc,ITS *iTS)
{
	short mean, mean_th ;
	short t1,t2,L,R;
	short DiffGrayTH;
	short DiffMeanTH;

#ifdef L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF
	if(iTS->L_Protection_Smear == SMEAR_SKIP_ON || iTS->L_Protection_Smear == SMEAR_ON)
	{
		if(iTS->L_Smear_LB[0] != 0 && iTS->L_Smear_RB[0] != 0)
		{
			if(index >= (iTS->L_Smear_LB[0] - minMW) && index <= (iTS->L_Smear_RB[0] + minMW))
			{
				return 0;
			}
		}
		if(iTS->L_Smear_LB[1] != 0 && iTS->L_Smear_RB[1] != 0)
		{
			if(index >= (iTS->L_Smear_LB[1] - minMW) && index <= (iTS->L_Smear_RB[1] + minMW))
			{
				return 0;
			}
		}
		if(iTS->L_Smear_LB[2] != 0 && iTS->L_Smear_RB[2] != 0)
		{
			if(index >= (iTS->L_Smear_LB[2] - minMW) && index <= (iTS->L_Smear_RB[2] + minMW))
			{
				return 0;
			}
		}
	}
#endif    //L_PROTECTION_CCD_CAMERA_SMEAR_ON_OFF

#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF	
	if(Lane->Sd == L_RtSd)
	{
        DiffGrayTH = iTS->LM_R_GrayScale_AdaptiveThresholding.GrayThreshold;
		DiffMeanTH = iTS->LM_R_GrayScale_AdaptiveThresholding.MeanThreshold;
	}
	else
	{
		DiffGrayTH = iTS->LM_L_GrayScale_AdaptiveThresholding.GrayThreshold;
		DiffMeanTH = iTS->LM_L_GrayScale_AdaptiveThresholding.MeanThreshold;
    }
//    DiffGrayTH = L_TH_DiffGrayIMS;
//    DiffMeanTH = L_TH_DiffMeanIMS;
#else
	DiffGrayTH = L_TH_DiffGrayIMS;
	DiffMeanTH = L_TH_DiffMeanIMS;
#endif

     maxMW = (maxMW >> 1)+1;
	if(index-maxMW<Lane->SchRoi.Frt)
		t1=Lane->SchRoi.Frt;
        else
		t1=index-maxMW;
	if(index+maxMW>Lane->SchRoi.Scd)
		t2=Lane->SchRoi.Scd;
	else
		t2=index+maxMW;

	if (pSrc[index]-DiffGrayTH > pSrc[t1] && pSrc[index]-DiffGrayTH > pSrc[t2])
	{
		L = MaxL(&iTS->L_ColProjection[(iTS->F_H-row_-1)*iTS->F_W], t1 , index);
		R = MaxR(&iTS->L_ColProjection[(iTS->F_H-row_-1)*iTS->F_W], index, t2);

		while (L != R  &&  R-L+1 > minMW)
		{
			t1 = L+1;
			t2 = R-1;
			mean = Mean(pSrc, t1, t2);
            mean_th = mean - DiffMeanTH;
			if (mean_th > pSrc[L-1] && mean_th > pSrc[R+1])
			{

#ifdef LM_GRAYSCALE_ADAPTIVE_THRESHOLDING_ON_OFF
				if(Lane->Sd == L_RtSd)
					LM_GrayScale_Adaptive_Thresholding_PreProcess(&iTS->LM_R_GrayScale_AdaptiveThresholding, L, R, pSrc, iTS);
				else
					LM_GrayScale_Adaptive_Thresholding_PreProcess(&iTS->LM_L_GrayScale_AdaptiveThresholding, L, R, pSrc, iTS);


#else
#ifdef GRAYSCALE_REMAPPING_ON_OFF
					iTS->L_Grayscale_Remapping.LastMarkMeanValue = mean;
#endif
#endif

				Lane->MrkEdg.Frt=L;
				Lane->MrkEdg.Scd=R;

                if(Lane->mark_width == 0)
                    Lane->mark_width = (DItoDW((double)(R - L), iTS->L_evRodSlp - (LimitVP(row_) - iTS->F_H_C + 1 )) - 1);
                else
                {
                    Lane->mark_width += (DItoDW((double)(R - L), iTS->L_evRodSlp - (LimitVP(row_) - iTS->F_H_C + 1 )) - 1);
                    Lane->mark_width *= 0.5;
                }


#ifdef L_WATER_STREAK_DETECTION_ON_OFF
				if(row_ > (iTS->L_Block_Position - 30) && row_ < (iTS->L_Block_Position + 30))
				{
					if(Lane->Sd == L_LtSd)
					{
						iTS->L_Left_Lane_Pixel_Total += pSrc[((R - L) >> 1) + L];
						iTS->L_Left_Lane_Pixel_Point++;
					}
					else
					{
						iTS->L_Right_Lane_Pixel_Total += pSrc[((R - L) >> 1) + L];
						iTS->L_Right_Lane_Pixel_Point++;
					}
				}
#endif    //L_WATER_STREAK_DETECTION_ON_OFF

				return 1;
			}
			if (index == L)
			{
				t1 = R-1;
				R = MaxR(&iTS->L_ColProjection[(iTS->F_H-row_-1)*iTS->F_W], index, t1);
			}
			else if (index == R)
			{
				t1 = L+1;
				L = MaxL(&iTS->L_ColProjection[(iTS->F_H-row_-1)*iTS->F_W],t1 , index);
			}
			else
			{
				if(Mean(pSrc, L+1, index) > Mean(pSrc, index, R-1))
					R = MaxR(&iTS->L_ColProjection[(iTS->F_H-row_-1)*iTS->F_W], index, R-1);
				else
					L = MaxL(&iTS->L_ColProjection[(iTS->F_H-row_-1)*iTS->F_W], L+1, index);
			}
		}
	}
	return 0;
}

short IsMark(LaneInfo *Lane,short row_,UBYTE *pSrc, ITS *iTS)
{
	short i;
	short begin,end;
	short MinMW = DWtoDI(L_RW_MinMarkW,iTS->L_Vrow);
	short MaxMW = DWtoDI(L_RW_MaxMarkW,iTS->L_Vrow);

	//if(MaxMW<=2)
	//	return 0;
	//if(MinMW<=1)
	//	return 0;
	if ((Lane->SchRoi.Scd - Lane->SchRoi.Frt+1)<=MinMW)
		return 0;

	if(Lane->Sd == L_RtSd)	//dirction 1 = right
	{	
		begin = Lane->SchRoi.Frt;
		end = Lane->SchRoi.Scd;	
	}
	else					//dirction-1 = left
	{	
		begin = Lane->SchRoi.Scd;	
		end = Lane->SchRoi.Frt;	
	}
	i=begin;
	while(1)
	{
		if(Lane->Sd == 1)	//right
		{
			if(i>end)
				break;
		}
		else   				//left
		{	
			if(i<end)
			   break;
		}
		if (MarkSearch(i,row_,Lane,MaxMW,MinMW,pSrc,iTS))
		{
			Lane->CurCol = ((Lane->MrkEdg.Frt + Lane->MrkEdg.Scd) >> 1);
#ifdef L_LOSS_LANE_FOR_BIAS_TO_LARGE
			if(iTS->L_DualLeftFlag == 1 && Lane->Sd == L_LtSd)
			{
				iTS->L_LeftFirstRow = row_;
				iTS->L_LeftFirstCol = Lane->CurCol;
				iTS->L_DualLeftFlag = 2;
			}
			if(iTS->L_DualRightFlag == 1 && Lane->Sd == L_RtSd)
			{
				iTS->L_RightFirstRow = row_;
				iTS->L_RightFirstCol = Lane->CurCol;
				iTS->L_DualRightFlag = 2;
			}
#endif    //L_LOSS_LANE_FOR_BIAS_TO_LARGE

			if((row_<iTS->L_LastFinalRow_) && (iTS->L_StbCtr >L_FC_InStable))
			{
				if((abs(Lane->CurCol-iTS->L_LaneMBound[row_].Frt)<(MaxMW << 1)) && Lane->Sd==-1)		 
						return 1;	
				else if(abs(Lane->CurCol-iTS->L_LaneMBound[row_].Scd)<(MaxMW << 1))
						return 1;
			}
			else
				return 1;
		}
		i+=Lane->Sd;
	}
	return 0;
}
