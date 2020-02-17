#include "InitialVariable.h"
#include "FunctionType.h"

short O_JUDGE(short row_,ITS *iTS);
short O_FuzzyFindBest(enum CarType_t Ctype,enum Feature_t Ftype,char TypeRec[N_CarType],char FuzzyResult[N_MaxPairGroup],ITS *iTS);
short O_FuzzyCheckAll(ITS *iTS);
void O_FuzzyFlow(short row_,ITS *iTS);
short O_FuzzyResultProcess_row(short row_,ITS *iTS);
void O_FZ_UndefineAssign(short row_,short SCOURE,enum CarType_t CarType,ITS *iTS);

//#define TEST_IR_RULE

void O_FuzzyResultClean(ITS *iTS)
{
	V_CleanCarInfo(&iTS->V_FZCar);
}

void O_FinishREC(REC_* rec,ITS *iTS)
{
	rec->BW=LimitW(rec->RB-rec->LB+1);
	rec->BH=((rec->BW*3)>>2);
	rec->TB=LimitH(rec->BB+rec->BH-1);
	rec->BH=rec->TB-rec->BB+1;
}

void O_FuzzyUpdatePerFrame(ITS *iTS)
{
	short row_;
	for(row_=0; row_<S_IMGCH; row_++)
	{
		iTS->CAR->FZArr[row_]--;
		iTS->CAR->FZArr[row_]=LimitCount(iTS->CAR->FZArr[row_],O_FC_MaxCountFuzzyArray);
	}
}

short O_FuzzySystem_row(short row_,ITS *iTS)
{
	O_FuzzyFlow(row_,iTS);
	return O_FuzzyResultProcess_row(row_,iTS);
}

short O_FuzzyResultProcess_row(short row_,ITS *iTS)
{
	short count;
	if(row_>=S_IMGCH)
		return 0;
	if(iTS->V_FZCar.Ctype==None)
		return 0;
	else if(iTS->CAR->CarInfo.Ctype == Night_CL)
	{
		if(iTS->O_CL_Array[row_])
			return 4;
	}
	else if(iTS->V_FZCar.Ctype==BigBlack)
	{
		O_FZ_UndefineAssign(row_,3,BigBlack,iTS);

		count=(iTS->CAR->FZArr[row_]);
		if(count>=O_FC_BigBlackOK)
			return 1;
		else
			return 2;
	}
	else if(iTS->V_FZCar.Ctype==UnknownObj)
	{
		O_FZ_UndefineAssign(row_,2,UnknownObj,iTS);

		count=(iTS->CAR->FZArr[row_]);
		if(count>=O_FC_UnknownObjOK)
			return 1;
		else
			return 3;
	}
	return 1;
}

void O_FuzzyFlow(short row_,ITS *iTS)
{
	O_FuzzyResultClean(iTS);
	iTS->V_FZCar.Position.BB = row_;
	O_JUDGE(row_,iTS);
	O_FuzzyCheckAll(iTS);
}

void O_JUDGE_iPG(short iPG,short row_,ITS *iTS)
{
	short Distance = 0;

	//規則 1 垂直邊緣密度
	if(iTS->O_FZ_PVEInfo[iPG][VE_]>=15)
		iTS->O_FZ_Feature[iPG][VE_]=1;
	else
		iTS->O_FZ_Feature[iPG][VE_]=0;

	//規則 2 車燈特徵數量
	if(iTS->O_FZ_PVEInfo[iPG][CL_]>=3)
		iTS->O_FZ_Feature[iPG][CL_]=2;
	else if(iTS->O_FZ_PVEInfo[iPG][CL_]>=2)
		iTS->O_FZ_Feature[iPG][CL_]=1;
	else
		iTS->O_FZ_Feature[iPG][CL_]=0;

	//規則 3  陰影分布程度
	if(iTS->O_FZ_PVEInfo[iPG][SD_]>=85)
		iTS->O_FZ_Feature[iPG][SD_]=1;
	else
		iTS->O_FZ_Feature[iPG][SD_]=0;

	//規則 4 區塊陰影密度
	if(iTS->O_FZ_PVEInfo[iPG][BB_]>=60)
		iTS->O_FZ_Feature[iPG][BB_]=1;
	else
		iTS->O_FZ_Feature[iPG][BB_]=0;

	//規則 5 水平邊緣密度
	if(iTS->O_FZ_PVEInfo[iPG][HE_]>=40)
		iTS->O_FZ_Feature[iPG][HE_]=2;
	else if(iTS->O_FZ_PVEInfo[iPG][HE_]>=20)
		iTS->O_FZ_Feature[iPG][HE_]=1;
	else
		iTS->O_FZ_Feature[iPG][HE_]=0;

	//規則 6 追蹤重疊程度
	if(iTS->O_FZ_PVEInfo[iPG][IR_]>=95)
		iTS->O_FZ_Feature[iPG][IR_]=2;
	else if(iTS->O_FZ_PVEInfo[iPG][IR_]>=90)
		iTS->O_FZ_Feature[iPG][IR_]=1;
	else
		iTS->O_FZ_Feature[iPG][IR_]=0;

	//規則 7 相對距離
	Distance=O_GetDistance(row_,iTS);
	iTS->O_Distance = Distance;
	if(Distance>4000)
		iTS->O_FZ_Feature[iPG][DS]=2;
	else if(Distance>3000)
		iTS->O_FZ_Feature[iPG][DS]=1;
	else
		iTS->O_FZ_Feature[iPG][DS]=0;

	//規則 8 橋梁陰影密度
	if(iTS->O_BridgeShadow<=70 )
		iTS->O_FZ_Feature[iPG][BSD]=0;
	else
		iTS->O_FZ_Feature[iPG][BSD]=1;

	//規則 0 是否有成對垂直邊緣存在
	if(iTS->O_VE_VP_PairGroupNum<1)
		iTS->O_FZ_Feature[iPG][PV]=0;
	else
		iTS->O_FZ_Feature[iPG][PV]=1;
}

short O_JUDGE(short row_,ITS *iTS)
{
	short iPG=0;
	if(iTS->O_VE_VP_PairGroupNum==0)
	{	return 0;	}
	else
	{
		for(iPG=0; iPG<iTS->O_VE_VP_PairGroupNum; iPG++)
			O_JUDGE_iPG(iPG,row_,iTS);
	}

	return 1;
}

void O_FuzzyRule(short iPG,char TypeRec[N_CarType],char FuzzyResult[N_MaxPairGroup],ITS *iTS)
{

	if(   iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][VE_]==1
		&&iTS->O_FZ_Feature[iPG][CL_]==0
		&&iTS->O_FZ_Feature[iPG][SD_]==1
		&&iTS->O_FZ_Feature[iPG][BB_]==0
		&&iTS->O_FZ_Feature[iPG][HE_]>=1
		&&iTS->O_FZ_Feature[iPG][BSD]==0
		//&&iTS->O_FZ_Feature[iPG][DS]>=1
		)
	{

		FuzzyResult[iPG]=Day;
		TypeRec[Day]++;
	}
	else if(
		  iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][CL_]>=1
		&&iTS->O_FZ_Feature[iPG][DS]>=1)
	{
		FuzzyResult[iPG]=Night_CL;
		TypeRec[Night_CL]++;
	}
	else if(
		  iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][CL_]>=2
		&&iTS->O_FZ_Feature[iPG][DS]==0)
	{
		FuzzyResult[iPG]=Night_CL;
		TypeRec[Night_CL]++;
	}
	else if(
		  iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][HE_]>=1
		&&iTS->O_FZ_Feature[iPG][CL_]>=1
		&&iTS->O_FZ_Feature[iPG][DS]==0)
	{
		FuzzyResult[iPG]=Night_CL;
		TypeRec[Night_CL]++;
	}
	else if(
		  iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][VE_]==1
		&&iTS->O_FZ_Feature[iPG][HE_]==2
		&&iTS->O_FZ_Feature[iPG][DS]<=1
		&&iTS->O_FZ_Feature[iPG][BSD]==0
		&&iTS->find_corner_flag == 0)
	{
		FuzzyResult[iPG]=Night_HE;
		TypeRec[Night_HE]++;
	}
	else if(
		  iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][SD_]==0
		&&iTS->O_FZ_Feature[iPG][BB_]==1
		&&iTS->O_FZ_Feature[iPG][DS]<=1
		&&iTS->O_FZ_Feature[iPG][BSD]==0)
	{
		FuzzyResult[iPG]=BigBlack;
		TypeRec[BigBlack]++;
	}

	else if(
		  iTS->find_corner_flag == 1
		&&iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][DS]<=1
		/*&&iTS->O_FZ_Feature[iPG][HE_]>=1*/
		&&iTS->O_FZ_Feature[iPG][BSD]==0)
	{
		FuzzyResult[iPG]=CarCorner;
		TypeRec[CarCorner]++;
	}

	/*else if(
		  iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][VE_]==1
		&&iTS->O_FZ_Feature[iPG][DS]<=2)
	{
		FuzzyResult[iPG]=UnknownObj;
		TypeRec[UnknownObj]++;
	}*/

#ifdef TEST_IR_RULE
	else if(
		  iTS->O_FZ_Feature[iPG][PV]==1
		&&iTS->O_FZ_Feature[iPG][IR_]>=1)
	{
		FuzzyResult[iPG]=Tracking;
		TypeRec[Tracking]++;
	}
#endif

	else
	{
		FuzzyResult[iPG]=None;
		TypeRec[None]++;
	}
//	OSD_Color_Setup(OCN_GREEN, iTS);
//	ScalableNumber((short) FuzzyResult[iPG], 5, S_IMGCW+150, 120, iTS);

}

short O_FuzzyCheckAll(ITS *iTS)
{
	char TypeRec[N_CarType]={0};
	char FuzzyResult[N_MaxPairGroup]={0};
	char iPG;


	if(iTS->O_VE_VP_PairGroupNum==0)
		return None;

	for(iPG=0; iPG<iTS->O_VE_VP_PairGroupNum; iPG++)
		O_FuzzyRule(iPG,TypeRec,FuzzyResult,iTS);

	if(O_FuzzyFindBest(Day,SD_,TypeRec,FuzzyResult,iTS))
		return Day;
	if(O_FuzzyFindBest(Night_CL,CL_,TypeRec,FuzzyResult,iTS))
		return Night_CL;

#ifdef TEST_IR_RULE
	if(O_FuzzyFindBest(Tracking,IR_,TypeRec,FuzzyResult,iTS))
		return Tracking;
#endif
	if(O_FuzzyFindBest(Night_HE,CL_,TypeRec,FuzzyResult,iTS))
		return Night_HE;
	if(O_FuzzyFindBest(Night_HE,HE_,TypeRec,FuzzyResult,iTS))
		return Night_HE;
	if(O_FuzzyFindBest(BigBlack,BB_,TypeRec,FuzzyResult,iTS))
		return BigBlack;
	if(O_FuzzyFindBest(CarCorner,VE_,TypeRec,FuzzyResult,iTS))
		return CarCorner;
	if(O_FuzzyFindBest(UnknownObj,VE_,TypeRec,FuzzyResult,iTS))
		return UnknownObj;
	return None;
}

short O_FuzzyFindBest(enum CarType_t Ctype,enum Feature_t Ftype,char TypeRec[N_CarType],char FuzzyResult[N_MaxPairGroup],ITS *iTS)
{
	short Compare = 0;
	short RECiPG = 0;
	short iPG = 0;

	if(TypeRec[Ctype]==0)
		return 0;

	for(iPG=0; iPG<iTS->O_VE_VP_PairGroupNum; iPG++)
	{

#ifndef TEST_IR_RULE
		if(FuzzyResult[iPG] == Ctype && iTS->O_FZ_Feature[iPG][IR_] >= 1)
		{
			if(iTS->O_FZ_PVEInfo[iPG][IR_] > Compare)
			{
				Compare = iTS->O_FZ_PVEInfo[iPG][IR_];
				RECiPG = iPG;
			}
		}
#endif

		if(FuzzyResult[iPG] == Ctype && abs(iTS->O_FZ_PVEInfo[iPG][Ftype]) > Compare)
		{
			Compare = abs(iTS->O_FZ_PVEInfo[iPG][Ftype]);
			RECiPG = iPG;
		}

	}
	if(Compare==0)
		return 0;
	iTS->V_FZCar.Ctype = Ctype;
	iTS->V_FZCar.Position.LB = iTS->O_VE_VP_PairGroupArray[RECiPG].Frt;
	iTS->V_FZCar.Position.RB = iTS->O_VE_VP_PairGroupArray[RECiPG].Scd;

	O_FinishREC(&iTS->V_FZCar.Position,iTS);

	return 1;
}

void O_FZ_UndefineAssign(short row_,short SCOURE,enum CarType_t CarType,ITS *iTS)
{
	short i;
	for(i=0;i<=SCOURE;i++)
	{
		iTS->CAR->FZArr[LimitVP(row_+i)]+=SCOURE-i;
	}
}

