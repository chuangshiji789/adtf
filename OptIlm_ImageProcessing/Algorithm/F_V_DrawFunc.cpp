#include "InitialVariable.h"
#include "FunctionType.h"

const Num NumberMap = {1,1,1,1,0,1,1,0,1,1,0,1,1,1,1, /* 0 */
					   0,1,0,0,1,0,0,1,0,0,1,0,0,1,0, /* 1 */
					   1,1,1,0,0,1,1,1,1,1,0,0,1,1,1, /* 2 */
					   1,1,1,0,0,1,1,1,1,0,0,1,1,1,1, /* 3 */
					   1,0,1,1,0,1,1,1,1,0,0,1,0,0,1, /* 4 */
					   1,1,1,1,0,0,1,1,1,0,0,1,1,1,1, /* 5 */
					   1,1,1,1,0,0,1,1,1,1,0,1,1,1,1, /* 6 */
					   1,1,1,0,0,1,0,1,0,0,1,0,0,1,0, /* 7 */
					   1,1,1,1,0,1,1,1,1,1,0,1,1,1,1, /* 8 */
					   1,1,1,1,0,1,1,1,1,0,0,1,1,1,1, /* 9 */
					   0,0,0,0,1,0,1,1,1,0,1,0,0,0,0, /* + */
					   0,0,0,0,0,0,1,1,1,0,0,0,0,0,0, /* - */
					   1,1,1,1,0,0,1,1,1,1,0,0,1,0,0};/* F */


void DrawRec(REC_ rc,unsigned char color,unsigned char *Img,ITS *iTS)
{
	short row_;
	unsigned char *Proc;

	Proc = &Img[(iTS->F_H-1-rc.BB)*iTS->F_W];
	memset(&Proc[rc.LB],color,rc.BW);
	Proc = &Img[(iTS->F_H-1-rc.TB)*iTS->F_W];
	memset(&Proc[rc.LB],color,rc.BW);

	for(row_=rc.BB;row_<=rc.TB;row_++)
	{
		Proc = &Img[(iTS->F_H-row_-1 )*iTS->F_W];
		Proc[rc.LB]=color;
		Proc[rc.RB]=color;
	}
	
}

void DrawRectangle(short BB, short TB, short LB, short RB, ITS *iTS)
{
	short row_;
	unsigned char *Proc_Y;
	unsigned char *Proc_U;
	unsigned char *Proc_V;

	LB = LimitW(LB);
	RB = LimitW(RB);
	BB = LimitH(BB);
	TB = LimitH(TB);

	Proc_Y = &iTS->Showimage[(iTS->F_H - 1 - BB) * iTS->F_W];
	Proc_U = &iTS->ShowUImg[((iTS->F_H - 1 - BB) * iTS->F_W) >> 1];
	Proc_V = &iTS->ShowVImg[((iTS->F_H - 1 - BB) * iTS->F_W) >> 1];
	memset(&Proc_Y[LB], iTS->OSD_Color.Y, RB - LB + 1);
	memset(&Proc_U[LB>>1], iTS->OSD_Color.U, (RB - LB + 1)>>1);
	memset(&Proc_V[LB>>1], iTS->OSD_Color.V, (RB - LB + 1)>>1);
	
	Proc_Y = &iTS->Showimage[(iTS->F_H - 1 - TB) * iTS->F_W];
	Proc_U = &iTS->ShowUImg[((iTS->F_H - 1 - TB) * iTS->F_W) >> 1];
	Proc_V = &iTS->ShowVImg[((iTS->F_H - 1 - TB) * iTS->F_W) >> 1];
	memset(&Proc_Y[LB], iTS->OSD_Color.Y, RB - LB + 1);
	memset(&Proc_U[LB>>1], iTS->OSD_Color.U, (RB - LB + 1)>>1);
	memset(&Proc_V[LB>>1], iTS->OSD_Color.V, (RB - LB + 1)>>1);

	for(row_ = BB; row_ <= TB;row_++)
	{
		Proc_Y = &iTS->Showimage[(iTS->F_H - 1 - row_) * iTS->F_W];
		Proc_U = &iTS->ShowUImg[((iTS->F_H - 1 - row_) * iTS->F_W) >> 1];
		Proc_V = &iTS->ShowVImg[((iTS->F_H - 1 - row_) * iTS->F_W) >> 1];

		Proc_Y[LB] = iTS->OSD_Color.Y;
		Proc_U[LB>>1] = iTS->OSD_Color.U;
		Proc_V[LB>>1] = iTS->OSD_Color.V;
		Proc_Y[RB] = iTS->OSD_Color.Y;
		Proc_U[RB>>1] = iTS->OSD_Color.U;
		Proc_V[RB>>1] = iTS->OSD_Color.V;
	}
	
}

void DrawColorLine(short row, short LB, short RB, ITS *iTS)
{
	int r = 0;

	LB = LimitW(LB);
	RB = LimitW(RB);

	if(row >= S_IMGTB)
            row = S_IMGTB;
	else if(row <= S_IMGBB)
            row = S_IMGTB;

        row = S_IMGH - row;

	r = (iTS->F_W * row + LB);

	memset((iTS->Showimage + iTS->F_W * row + LB), iTS->OSD_Color.Y, (RB - LB + 1));
	memset(&iTS->ShowUImg[(r>>1)], iTS->OSD_Color.U, (RB - LB + 1)>>1);
	memset(&iTS->ShowVImg[(r>>1)], iTS->OSD_Color.V, (RB - LB + 1)>>1);

}

void DrawBigF(short Number, short StartCol, short StartRow, ITS *iTS)
{
	short i;
	for(i=0; i<VNumberSet; i++)
	{
		if (1==NumberMap.element[Number*VNumberSet+i])
			DrawRect_Has_UV(StartRow+(i/3)*12, StartCol+(i%3)*12, 12, iTS);
	}
}  

void DrawRect_Has_UV(short RectStartRow, short RectStartCol, short DrawSize, ITS *iTS)
{
	short row_;
	int r=(iTS->F_W * RectStartRow + RectStartCol);
	for (row_ = RectStartRow; row_ < RectStartRow + DrawSize; row_ ++)
	{
		memset((iTS->Showimage + iTS->F_W * row_ + RectStartCol), iTS->OSD_Color.Y, DrawSize);
		memset(&iTS->ShowUImg[(r>>1)], iTS->OSD_Color.U, DrawSize>>1);
		memset(&iTS->ShowVImg[(r>>1)], iTS->OSD_Color.V, DrawSize>>1);
		r+=iTS->F_W;
	}
}

void DrawTriangle_Has_UV_Right(short start_row, short start_column, short draw_size, ITS *iTS)
{
	short row = 0;
	short half_height = draw_size >> 1;
	short current_width = 0;
	int image_start_row_position = (iTS->F_W * start_row + start_column);

	for(row = start_row; row < start_row + draw_size; row ++)
	{
		if(row <= start_row + half_height)
		{
			current_width ++;
			memset(&iTS->Showimage[image_start_row_position], iTS->OSD_Color.Y, current_width);
			memset(&iTS->ShowUImg[(image_start_row_position >> 1)], iTS->OSD_Color.U, current_width >> 1);
			memset(&iTS->ShowVImg[(image_start_row_position >> 1)], iTS->OSD_Color.V, current_width >> 1);
			image_start_row_position += iTS->F_W;
		}
		else
		{
			current_width --;
			memset(&iTS->Showimage[image_start_row_position], iTS->OSD_Color.Y, current_width);;
			memset(&iTS->ShowUImg[(image_start_row_position >> 1)], iTS->OSD_Color.U, current_width >> 1);
			memset(&iTS->ShowVImg[(image_start_row_position >> 1)], iTS->OSD_Color.V, current_width >> 1);
			image_start_row_position += iTS->F_W;
		}
	}
}

void DrawTriangle_Has_UV_Left(short start_row, short start_column, short draw_size, ITS *iTS)
{
	short row = 0;
	short half_height = draw_size >> 1;
	short current_width = 0;
	int image_start_row_position = (iTS->F_W * start_row + start_column);

	for(row = start_row; row < start_row + draw_size; row ++)
	{
		if(row <= start_row + half_height)
		{
			current_width ++;
			memset(&iTS->Showimage[image_start_row_position + (draw_size - current_width)], iTS->OSD_Color.Y, current_width);
			memset(&iTS->ShowUImg[((image_start_row_position + (draw_size - current_width)) >> 1)], iTS->OSD_Color.U, current_width >> 1);
			memset(&iTS->ShowVImg[((image_start_row_position + (draw_size - current_width)) >> 1)], iTS->OSD_Color.V, current_width >> 1);
			image_start_row_position += iTS->F_W;
		}
		else
		{
			current_width --;
			memset(&iTS->Showimage[image_start_row_position + (draw_size - current_width)], iTS->OSD_Color.Y, current_width);
			memset(&iTS->ShowUImg[((image_start_row_position + (draw_size - current_width)) >> 1)], iTS->OSD_Color.U, current_width >> 1);
			memset(&iTS->ShowVImg[((image_start_row_position + (draw_size - current_width)) >> 1)], iTS->OSD_Color.V, current_width >> 1);
			image_start_row_position += iTS->F_W;
		}
	}
}

void DrawBar(short BarStartRow, short BarStartCol, short Size_of_Column, short Size_of_Row, ITS *iTS)
{
	short row_;
	int r=(iTS->F_W * BarStartRow + BarStartCol);
	for (row_ = BarStartRow; row_ < BarStartRow + Size_of_Column; row_ ++)
	{
		memset((iTS->Showimage + iTS->F_W * row_ + BarStartCol), iTS->OSD_Color.Y, Size_of_Row);
		memset(&iTS->ShowUImg[(r>>1)], iTS->OSD_Color.U, Size_of_Row>>1);
		memset(&iTS->ShowVImg[(r>>1)], iTS->OSD_Color.V, Size_of_Row>>1);
		r+=iTS->F_W;
	}
}

void ScalableNumber(int Number, short DrawSize, unsigned short UPosition, unsigned short Vposition, ITS *iTS)
{
	short UnitsDigit, TensDigit, HundredsDigit, ThousandsDigit;
	short UnitSpece = 0, TenSpece = 0, HundredSpece = 0;
	short index;

	if(Number > 9999)
		Number = 9999;

	ThousandsDigit = (short) ((double) Number / 1000);
	HundredsDigit = (short) ((double) (Number - (ThousandsDigit * 1000)) / 100);
	TensDigit = (short) ((double) (Number - (ThousandsDigit * 1000) - (HundredsDigit * 100)) / 10);
	UnitsDigit = (short) Number % 10;

	if(ThousandsDigit)
	{
		HundredSpece = (DrawSize * 3) + 2;
		TenSpece = (DrawSize * 6) + 4;
		UnitSpece = (DrawSize * 9) + 6;
	}
	else if(!ThousandsDigit && HundredsDigit)
	{
		TenSpece = (DrawSize * 3) + 2;
		UnitSpece = (DrawSize * 6) + 4;
	}
	else if(!ThousandsDigit && !HundredsDigit && TensDigit)
	{
		UnitSpece = (DrawSize * 3) + 2;
	}

	for(index = 0; index < VNumberSet; index++)
	{
		if (ThousandsDigit && NumberMap.element[ThousandsDigit * VNumberSet + index] == 1)
		{
			DrawRect_Has_UV(Vposition+(index / 3) * DrawSize, UPosition + (index % 3) * DrawSize, DrawSize, iTS);
		}
			
		if ((ThousandsDigit || HundredsDigit) && NumberMap.element[HundredsDigit * VNumberSet + index] == 1)
		{
			DrawRect_Has_UV(Vposition+(index / 3) * DrawSize, (UPosition + (index % 3) * DrawSize) + HundredSpece, DrawSize, iTS);
		}

		if ((ThousandsDigit || HundredsDigit || TensDigit) && NumberMap.element[TensDigit * VNumberSet + index] == 1)
		{
			DrawRect_Has_UV(Vposition + (index / 3) * DrawSize, (UPosition + (index % 3) * DrawSize) + TenSpece, DrawSize, iTS);
		}

		if (NumberMap.element[UnitsDigit * VNumberSet + index] == 1)
		{
			DrawRect_Has_UV(Vposition + (index / 3) * DrawSize, (UPosition + (index % 3) * DrawSize) + UnitSpece, DrawSize, iTS);
		}
	}
}

void OSD_Color_Setup(unsigned char OSD_Color_Number, ITS *iTS)
{
	switch(OSD_Color_Number)
	{								//YUV;RGB
		case OCN_RED:
			iTS->OSD_Color.Y = 65;	//255
			iTS->OSD_Color.U = 90;	//0
			iTS->OSD_Color.V = 238;	//0
			break;
		case OCN_GREEN:
			iTS->OSD_Color.Y = 105;	//0
			iTS->OSD_Color.U = 68;	//180
			iTS->OSD_Color.V = 52;	//0
			break;
		case OCN_BLUE:
			iTS->OSD_Color.Y = 88;	//0
			iTS->OSD_Color.U = 221;	//102
			iTS->OSD_Color.V = 64;	//255
			break;
		case OCN_YELLOW:
			iTS->OSD_Color.Y = 255;	//255
			iTS->OSD_Color.U = 0;	//0
			iTS->OSD_Color.V = 148;	//148
			break;
		case OCN_BLACK:
			iTS->OSD_Color.Y = 0;	//0
			iTS->OSD_Color.U = 128;	//0
			iTS->OSD_Color.V = 128;	//0
			break;
		case OCN_WHITE:
			iTS->OSD_Color.Y = 255;	//255
			iTS->OSD_Color.U = 127;	//255
			iTS->OSD_Color.V = 127;	//255
			break;
		case OCN_GRAY:
			iTS->OSD_Color.Y = 188;	//188
			iTS->OSD_Color.U = 127;	//188
			iTS->OSD_Color.V = 127;	//188
			break;
		case OCN_ORANGE:
			iTS->OSD_Color.Y = 136;	//255
			iTS->OSD_Color.U = 51;	//102
			iTS->OSD_Color.V = 212;	//0
			break;
		case OCN_PURPLE:
			iTS->OSD_Color.Y = 95;	//255
			iTS->OSD_Color.U = 169;	//0
			iTS->OSD_Color.V = 241;	//169
			break;
		default:
			printf("Color setup ERROR!! Error function call for L_SundriesFunction.c OSD_Color_Setup()\n");
			getchar();
			break;
	}
}







