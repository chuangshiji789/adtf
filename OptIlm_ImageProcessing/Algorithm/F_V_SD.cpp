#include "InitialVariable.h"
#include "FunctionType.h"

void O_SetPixelArea(ITS *iTS)
{
	double v1,v2,A;
	short row_;
	for(row_=0; row_<S_IMGCH; row_++)
	{
		v1=row_;
		v2=row_+1;
		v1-=iTS->F_H_C;
		v2-=iTS->F_H_C;
		if(v1 == 0)
			v1 = 0;
		else
			v1=1/v1;
		if(v2 == 0)
			v2 = 0;
		else
			v2=1/v2;
		A=v1*(v2-v1);
		A=(double)(A*S_PixelArea);
		A/=0.8349;
		if(A<1)
			A=1;
		if(A>65535)
		{
			A=65535;
			//break;
		}
		iTS->PixelArea[row_]=(unsigned short)(A);
	}
}

short O_CheckShadowInPariVE(short Srow_,short LB,short RB,ITS *iTS)
{
	short MinW;

	short TempGrpNum=0;
	FORWARD_CROI TempGrpArr[N_MaxSingleGroup];
	MinW = O_DWtoDI(O_RW_ShadowGroupEDC,Srow_,iTS);

	if(iTS->O_SD_VerPrjArrMean<=1)
		return 0;

	TempGrpNum=O_FindSingleGroup(iTS->O_SD_VerPrjArrMean,2,MinW,LB,RB,
		                         iTS->O_SD_VerPrjArray,TempGrpArr);

	
	if(TempGrpNum>=2 || TempGrpNum==0)
		return 0;

	return (short)( (TempGrpArr[0].Scd)*100/(RB-LB+1) +0.5);
}

void O_ProjectRoadShadow(short Srow_,short Erow_,short Scol,short Ecol,ITS *iTS)
{
	short row_,col;
	unsigned char *CrRow;
	int r;

	memset(&iTS->O_SD_VerPrjArray[Scol],0,(Ecol-Scol+1)*sizeof(short));

	r=SinkDataIndexStart(Srow_);
	for(row_=Srow_; row_<=Erow_; row_++)
	{
		r=SinkDataIndexNextRow(r);
		CrRow =&iTS->O_InfoPlane[r];

		for(col=Scol; col<=Ecol; col++)
			if( (CrRow[col]&RSDINFO) == RSDINFO	)
				iTS->O_SD_VerPrjArray[col]++;
	}
	iTS->O_SD_VerPrjArrMean=O_GetMean(iTS->O_SD_VerPrjArray,Scol,Ecol);
}

void O_CheckBridgeShadow(short Srow_,short Erow_,ITS *iTS)
{
	short d1,d2,row_,col,Scol,Ecol;
	unsigned char *CrRow;
	int total=0,total1=0,total2=0,total3=0,total4=0;
	int count=0,count1=0,count2=0,count3=0,count4=0;
	int r;
	iTS->O_BridgeShadow = 0;
	r=SinkDataIndexStart(Srow_);
	for(row_=Srow_; row_<=Erow_; row_++)
	{
		r=SinkDataIndexNextRow(r);
		CrRow =&iTS->O_InfoPlane[r];

		d1 = O_DWtoDI(20,row_,iTS);
		d2 = (d1<<1);

		Scol=LimitW(iTS->O_LaneBound[row_].Frt-(d2+d1));
		Ecol=LimitW(iTS->O_LaneBound[row_].Frt-(d1));
		for(col=Scol; col<=Ecol; col++)
		{	
			total++;	total1++;
			if( (CrRow[col]&RSDINFO) == RSDINFO	)
			{
				count1++;
				count++;
			}
		}

		Scol=LimitW(iTS->O_LaneBound[row_].Frt+d1);
		Ecol=LimitW(iTS->O_LaneBound[row_].Frt+d1+d2);
		for(col=Scol; col<=Ecol; col++)
		{	
			total++;	total2++;
			if( (CrRow[col]&RSDINFO) == RSDINFO	)
			{	
				count2++;
				count++;
			}
		}

		Scol=LimitW(iTS->O_LaneBound[row_].Scd-(d2+d1));
		Ecol=LimitW(iTS->O_LaneBound[row_].Scd-(d1));
		for(col=Scol; col<=Ecol; col++)
		{	
			total++;	total3++;
			if( (CrRow[col]&RSDINFO) == RSDINFO	)
			{	
				count3++;	
				count++;	
			}
		}

		Scol=LimitW(iTS->O_LaneBound[row_].Scd+d1);
		Ecol=LimitW(iTS->O_LaneBound[row_].Scd+d1+d2);
		for(col=Scol; col<=Ecol; col++)
		{	
			total++;	total4++;
			if( (CrRow[col]&RSDINFO) == RSDINFO	)
			{	
				count4++;	
				count++;	
			}
		}

	}
	if(total<O_IA_FewTotal)
	{
		iTS->O_BridgeShadow=100;
		return ;
	}
	count=count*100/total;
	count1=count1*100/total1;
	count2=count2*100/total2;
	count3=count3*100/total3;
	count4=count4*100/total4;

	iTS->O_BridgeShadow=count;

	count=0;
	if(count1>O_ID_BlockOfBSD)	
		count++;
	if(count2>O_ID_BlockOfBSD)	
		count++;
	if(count3>O_ID_BlockOfBSD)
		count++;
	if(count4>O_ID_BlockOfBSD)
		count++;

	if(count>=3)
		iTS->O_BridgeShadow=100;
}




