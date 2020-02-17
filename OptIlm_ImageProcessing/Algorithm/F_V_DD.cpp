#include "InitialVariable.h"
#include "FunctionType.h"


short O_CheckAllDarkLightInPairVE(short BB,short TB,short LB,short RB,ITS *iTS)
{
	short col,row_;
	unsigned char *CrRow;
	int Total,ShadowExist,r;
	Total=0;
	ShadowExist=0;
	
	r=SinkDataIndexStart(BB);
	for(row_=BB; row_<=TB; row_++)
	{
		r=SinkDataIndexNextRow(r);
		CrRow =&iTS->O_InfoPlane[r];

		for(col=LB; col<=RB; col++)
		{
			Total+=1;
			if((CrRow[col]&RSDINFO) == RSDINFO)
				ShadowExist+=1;
		}
	}

	if(ShadowExist>Total)
		return (100);
	else
		return (short)(ShadowExist*100/Total);
}

short O_CheckVEDensityInPairVE(short BB,short TB,short LB,short RB,ITS *iTS)
{
	unsigned char *CrRow;
	short col,row_;
	int Total,Exist,r;
	short EDC;

	Total=0;
	Exist=0;
	EDC=O_DWtoDI(O_RW_JumpLaneMark,BB,iTS);
	LB=LimitW(LB-EDC);
	RB=LimitW(RB+EDC);

	r=SinkDataIndexStart(BB);
	for(row_=BB; row_<=TB; row_++)
	{
		r=SinkDataIndexNextRow(r);
		CrRow =&iTS->O_InfoPlane[r];

		for(col=LB; col<=RB; col++)
		{
			Total+=1;
			if((CrRow[col]&VEINFO) == VEINFO)
				Exist+=1;
		}
	}
	if(Total<O_IA_FewTotal)
		return 0;
	if(Exist>Total)
		return 100;
	return (short)(Exist*100/Total);
}

short O_CheckHEDensityInPairVE(short BB,short TB,short LB,short RB,ITS *iTS)
{
	short col,row_;
	unsigned char *CrRow;
	int Total,Exist,r;
	Total=0;
	Exist=0;

	r=SinkDataIndexStart(BB);
	for(row_=BB; row_<=TB; row_++)
	{
		r=SinkDataIndexNextRow(r);
		CrRow =&iTS->O_InfoPlane[r];

		for(col=LB; col<=RB; col++)
		{
			Total+=1;
		
			if(	  (CrRow[col  ] & HEINFO) == HEINFO 
				&&(CrRow[col-1] & HEINFO) == HEINFO 	
				&&(CrRow[col+1] & HEINFO) == HEINFO)
				Exist+=2;
			else if ( (CrRow[col  ] & HEINFO) == HEINFO)
				Exist+=1;
		}
	}
	if(Total<O_IA_FewTotal)
		return 0;
	if(Exist>Total)
		return 100;
	return (short)(Exist*100/Total);
}
