#include "InitialVariable.h"
#include "FunctionType.h"

short O_RLSign(unsigned char value)
{
	if((value&RLTINFO) == RLTINFO)
		return 1;
	else if((value&RSDINFO) == RSDINFO)
		return -1;
	else
		return 0;
}

short O_MEI(short table[3])
{
	short max=table[0];
	short maxindex=0;
	short i;
	for(i=0; i<3; i++)
	{
		if(table[i] >= max)
		{
			max = table[i];
			maxindex = i;
		}
	}
	return maxindex-1;
}

short O_CheckCarLightInPairVE(short BB,short TB,short LB,short RB,ITS *iTS)
{
	short d=RB-LB+1;
	short row_,col;
	short table[3];	//0:shadow,1:none,2:light
	short TEST[4];
	short result=0;
	short forB;
	int r;
	unsigned char *CrRow;
	d = (short)((double)d/5+0.5);

	r=SinkDataIndexStart(BB);
	for(row_=BB; row_<=TB; row_++)
	{
		CrRow = &iTS->O_InfoPlane[r];

		table[0]=table[1]=table[2]=0;
		forB=LB+d;
		for(col=LB;col<=forB;col++)
			table[1+O_RLSign(CrRow[col])]++;
		table[2] *=2;
		TEST[0]=O_MEI(table);


		table[0]=table[1]=table[2]=0;
		forB=LB+2*d;
		for(col=LB+d+1; col<=forB; col++)
			table[1+O_RLSign(CrRow[col])]++;
		TEST[1]=O_MEI(table);


		table[0]=table[1]=table[2]=0;
		forB=RB-2*d;
		for(col=RB-d-1; col>=forB; col--)
			table[1+O_RLSign(CrRow[col])]++;
		TEST[2]=O_MEI(table);


		table[0]=table[1]=table[2]=0;
		forB=RB-d;
		for(col=RB;     col>=forB;   col--)
			table[1+O_RLSign(CrRow[col])]++;
		table[2] *=2;
		TEST[3]=O_MEI(table);

		if((TEST[0]==1) && (TEST[1]!=1)  && (TEST[2]!=1) && (TEST[3]==1) /*&& (TEST[1]==TEST[2])*/)
		{

#ifdef O_DRAW_PAIR_CAR_LIGHT
			//畫出成對車燈配對
			memset(&iTS->Showimage[(iTS->F_H-row_-1)*iTS->F_W+LB],255,d);
			memset(&iTS->Showimage[(iTS->F_H-row_-1)*iTS->F_W+LB+d+1],0,d);
			memset(&iTS->Showimage[(iTS->F_H-row_-1)*iTS->F_W+RB-d-d-1],0,d);
			memset(&iTS->Showimage[(iTS->F_H-row_-1)*iTS->F_W+RB-d],255,d);
#endif

			result++;
		}
		r = SinkDataIndexNextRow(r);
	}
	return result;

}
