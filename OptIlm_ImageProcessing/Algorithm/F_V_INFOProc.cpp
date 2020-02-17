#include "InitialVariable.h"
#include "FunctionType.h"

void CreateEdge(ITS *iTS)
{
    short col,row_;
    unsigned char *CrRow,*UpRow,*DnRow,*O_Result,*L_Result;
    int SobelResult;
    int LSobel;


    unsigned short top_bound = S_IMGTB ;
    unsigned short bottom_bound = L_IB_BB_SearchLane;//S_IMGBB;
    unsigned short left_bound = S_IMGLB + 1;
    unsigned short right_bound = S_IMGRB - 1;

    //int r = SinkDataIndexStart(S_IMGBB);
    int r = SinkDataIndexStart(bottom_bound);

    unsigned char *Gxy_result;
    unsigned char *Axy_result;

    int Gx = 0;
    int Gy = 0;
    int Gxy = 0;
    float Axy = 0;


    int i = 0;
    int Total = 0;
    unsigned long int Gxy_level[256] = {0};
    float Probability[256], probabilityDistribution[256], Avg=0, Ans[256];
    float W1 = 0, W1_old = 0, W2 = 0, u1 = 0, u2 = 0, max = 0;




    unsigned int gray_level[256] = {0};
    unsigned int gray_level_counter = 0;

    unsigned short tail_light_max_gray_level = 0;
    unsigned short shadow_max_gray_level = 0;
    unsigned int gray_level_accumulation_result = 0;
    char shawow_flag = 0;
    char tail_light_flag = 0;


    memset(Ans, 0, 256 * sizeof(float));
    memset(Probability, 0, 256 * sizeof(float));
    memset(probabilityDistribution, 0, 256 * sizeof(float));


    for(row_= bottom_bound; row_< top_bound; row_++)
    {
        r=SinkDataIndexNextRow(r);
        CrRow = &iTS->YImg[r];
        UpRow = &iTS->YImg[r-iTS->F_W];
        DnRow = &iTS->YImg[r+iTS->F_W];
        O_Result = &iTS->O_InfoPlane[r];
        L_Result = &iTS->L_ColProjection[r];
        Gxy_result = &iTS->Gxy_InfoPlane[r];
        Axy_result = &iTS->Axy_InfoPlane[r];

        for(col = left_bound;col < right_bound; col++)
        {
            //initialization
            O_Result[col] &= ~HEVEINFO;
            O_Result[col] &= ~VE2INFO;
            L_Result[col] &= ~0x03;

            SobelResult=0;

            //horizontal edge detection
            /* 1   2   1		P1	P2	P3
               0   0   0		P4	P5	P6
              -1  -2  -1		P7	P8	P9 */

#ifdef GRAYSCALE_REMAPPING_ON_OFF
            LSobel =  iTS->L_Grayscale_Remapping.NewMapping[UpRow[col-1]] + (iTS->L_Grayscale_Remapping.NewMapping[UpRow[col]] << 1) + iTS->L_Grayscale_Remapping.NewMapping[UpRow[col+1]]
                    - iTS->L_Grayscale_Remapping.NewMapping[DnRow[col-1]] - (iTS->L_Grayscale_Remapping.NewMapping[DnRow[col]] << 1) - iTS->L_Grayscale_Remapping.NewMapping[DnRow[col+1]];

            SobelResult = UpRow[col-1] + (UpRow[col] << 1) + UpRow[col+1]
                        - DnRow[col-1] - (DnRow[col] << 1) - DnRow[col+1];
#else
#ifdef INCREASE_CONTRAST_ON_OFF
            SobelResult = iTS->increase_contrast_map[UpRow[col-1]] + (iTS->increase_contrast_map[UpRow[col]] << 1) + iTS->increase_contrast_map[UpRow[col+1]]
                        - iTS->increase_contrast_map[DnRow[col-1]] - (iTS->increase_contrast_map[DnRow[col]] << 1) - iTS->increase_contrast_map[DnRow[col+1]];

#else
            SobelResult = UpRow[col-1] + (UpRow[col] << 1) + UpRow[col+1]
                        - DnRow[col-1] - (DnRow[col] << 1) - DnRow[col+1];
#endif
#endif


            if(abs(SobelResult)> /*iTS->sobel_V_TH*/O_TH_HE)
            {
                O_Result[col] |= HEINFO;
#ifdef DRAW_HORIZANTAL_EDGE
                iTS->Showimage[(S_IMGH - row_) * S_IMGW + col] = 255;
#endif
            }

            if(abs(SobelResult) > 20)
            {
                L_Result[col] |= 0x04;
            }


#ifdef L_HORIZONTAL_MARKING_DETECTION_ON_OFF
            if(abs(SobelResult) > L_MARKING_HORIZONTAL_EDGE_THRESHOLD)
            {
                L_Result[col] |= 0x08;
            }
#endif    //L_HORIZONTAL_MARKING_DETECTION_ON_OFF

            SobelResult=0;

            //vertical edge detection
            /* 1   0  -1		P1	P2	P3
               2   0  -2		P4	P5	P6
               1   0  -1		P7	P8	P9 */

#ifdef GRAYSCALE_REMAPPING_ON_OFF

            LSobel += iTS->L_Grayscale_Remapping.NewMapping[UpRow[col-1]] - iTS->L_Grayscale_Remapping.NewMapping[UpRow[col+1]]
                    + (iTS->L_Grayscale_Remapping.NewMapping[CrRow[col-1]] << 1) - (iTS->L_Grayscale_Remapping.NewMapping[CrRow[col+1]] << 1)
                    + iTS->L_Grayscale_Remapping.NewMapping[DnRow[col-1]] - iTS->L_Grayscale_Remapping.NewMapping[DnRow[col+1]];

            SobelResult =  UpRow[col-1]       -  UpRow[col+1]
                        + (CrRow[col-1] << 1) - (CrRow[col+1] << 1)
                        +  DnRow[col-1]       -  DnRow[col+1];

#else

#ifdef INCREASE_CONTRAST_ON_OFF
            LSobel =  UpRow[col-1]       -  UpRow[col+1]
                   + (CrRow[col-1] << 1) - (CrRow[col+1] << 1)
                   +  DnRow[col-1]       -  DnRow[col+1];

            SobelResult =  iTS->increase_contrast_map[UpRow[col-1]]       -  iTS->increase_contrast_map[UpRow[col+1]]
                        + (iTS->increase_contrast_map[CrRow[col-1]] << 1) - (iTS->increase_contrast_map[CrRow[col+1]] << 1)
                        +  iTS->increase_contrast_map[DnRow[col-1]]       -  iTS->increase_contrast_map[DnRow[col+1]];

#else
            LSobel = SobelResult =  UpRow[col-1]       -  UpRow[col+1]
                                 + (CrRow[col-1] << 1) - (CrRow[col+1] << 1)
                                 +  DnRow[col-1]       -  DnRow[col+1];
#endif
#endif


            //車道線使用
            if(abs(LSobel) > L_TH_VE)
            {
                L_Result[col] |= VEINFO;
            }
            //反光使用
            if(abs(LSobel) > 25)
                L_Result[col] |= 0x01;
            if(abs(SobelResult) > /*iTS->sobel_H_TH*/O_TH_VE)
            {
                O_Result[col] |= VEINFO;
#ifdef DRAW_VERTICAL_EDGE
                iTS->Showimage[(S_IMGH - row_) * S_IMGW + col] = 128;
#endif
            }


            //initialization
            Gxy_result[col] = 0;
            Axy_result[col] = 0;


            //horizontal edge detection
            /* Gx =  -1   0   1 */
            Gx = -(CrRow[col - 1]) + (CrRow[col + 1]);

            //vertical edge detection
            /*    -1
                   0
                   1  		 */
            Gy = -(UpRow[col])   + (DnRow[col]);

            Gxy = (int)sqrt(pow(Gx, 2)+ pow(Gy,2));

            if(Gxy > 255)
                Gxy = 255;
            if(Gxy < 0)
                Gxy = 0;

            Gxy_level[Gxy]++;
            Avg += Gxy;
            Total++;


            if(Gxy > 15/*iTS->O_Objekt_TH*/)
            {
                Gxy_result[col] =  (unsigned char)Gxy;
                Gxy = 1;
            }
            else
            {
                Gxy = 0;
                Gxy_result[col] = 0;
            }


            if(Gxy == 1)
            {
                //            Axy = atan((Gy / Gx));
                Axy = atan2(Gy, Gx);

                if(Axy < 0)
                    Axy_result[col] = (unsigned char)(-Axy / DIRECTION) + 1;
                else
                    Axy_result[col] = (unsigned char)(Axy / DIRECTION) + 1;

//                Axy_result[col] = (unsigned char)(fabs(Axy) * 180/3.14);
            }
            else
            {
                Axy = 0;
                Axy_result[col] = (unsigned char)Axy;
            }




            gray_level[CrRow[col]]++;
            //gray_level_counter++;
            gray_level_counter+=CrRow[col];
        }
    }

    Avg = Avg / Total;

    for(i = 0; i <= 255; i++){
        Probability[i] = ((float)Gxy_level[i] / (float)Total);
        probabilityDistribution[i] = Probability[i];
    }

    for(i = 1; i <= 255; i++)
        probabilityDistribution[i] += probabilityDistribution[i-1];

    for(i = 0; i <= 255; i++){
        W1 = probabilityDistribution[i];
        W2 = 1 - probabilityDistribution[i];
        if(W1 == W1_old || W1 == 0){
            W1_old = W1;
            continue;
        }
        for(int j = 0; j <= 255; j++){
            if(j <= i)
                u1 += (Probability[j]/W1)*j;
            else
                u2 += (Probability[j]/W2)*j;
        }
        Ans[i] = (W1*((u1-Avg)*(u1-Avg)))+(W2*((u2-Avg)*(u2-Avg)));
        if(Ans[i] > max){
            max = Ans[i];
            iTS->O_Objekt_TH = i;
        }
        u1 = 0;
        u2 = 0;
        W1_old = W1;
    }

    shadow_max_gray_level = (short)((double)gray_level_counter * 0.1);
    tail_light_max_gray_level = (short)((double)gray_level_counter * 0.97);
//    mark_light_max_gray_level = (short)((double)gray_level_counter * 0.9);

    for(row_ = 0; row_ < 256;  row_ ++)
    {
        gray_level_accumulation_result += gray_level[row_];
        if(gray_level_accumulation_result >= shadow_max_gray_level && shawow_flag == 0)
        {
            iTS->O_SD_ShadowTh = row_;
            shawow_flag = 1;
        }

        if(gray_level_accumulation_result >= tail_light_max_gray_level && tail_light_flag == 0)
        {
            iTS->O_CarLightTH = row_;
            tail_light_flag = 1;
        }
//        if(gray_level_accumulation_result >= mark_light_max_gray_level && mark_light_flag == 0)
//        {
//            iTS->O_MarkLightTH = row_;
//            mark_light_flag = 1;
//        }
    }
    //memcpy ( iTS->L_InfoPlane, iTS->L_ColProjection, strlen(iTS->L_ColProjection)+1 );
}

void O_CreateMarkInfo(ITS *iTS)
{
    short col,row_;
    unsigned char *CrRow;

    unsigned short top_bound = L_IB_TB_SearchLane;//S_IMGCH ;
    unsigned short bottom_bound = L_IB_BB_SearchLane;
//    unsigned short left_bound = S_IMGLB + 1;
//    unsigned short right_bound = S_IMGRB - 1;


    unsigned long int gray_level[256] = {0};
    unsigned int gray_level_counter = 0;

//    unsigned int gray_level_accumulation_result = 0;
//    unsigned short mark_light_max_gray_level = 0;
//    char mark_light_flag = 0;





    for(row_= bottom_bound; row_< top_bound; row_++)
    {
        CrRow = &iTS->YImg[GetImageDataIndex(row_)];


        //for(col = left_bound;col < right_bound; col++)
        for(col = iTS->O_LaneMBound[row_].Frt - 20 ;col < iTS->O_LaneMBound[row_].Scd + 20; col++)
        {


            gray_level[CrRow[col]]++;
            gray_level_counter+=CrRow[col];

        }
    }


//    mark_light_max_gray_level = (short)((double)gray_level_counter * 0.95);

//    for(row_ = 0; row_ < 256;  row_ ++)
//    {
//        gray_level_accumulation_result += gray_level[row_];
//        if(gray_level_accumulation_result >= mark_light_max_gray_level && mark_light_flag == 0)
//        {
//            iTS->O_MarkLightTH = row_;
//            mark_light_flag = 1;
//        }


//    }


    int i = 0;
    int Total = 0;
    float Probability[256], probabilityDistribution[256], Avg=0, Ans[256];
    float W1 = 0, W1_old = 0, W2 = 0, u1 = 0, u2 = 0, max = 0;

    memset(Ans, 0, 256 * sizeof(float));
    memset(Probability, 0, 256 * sizeof(float));
    memset(probabilityDistribution, 0, 256 * sizeof(float));

//    for(int i=0;i<=255;i++){
//        Y_Array[i]     = 0;
//        Probability[i] = 0;
//    	probabilityDistribution[i] = 0;
//    	Ans[i] = 0;
//    }


    for(row_= bottom_bound; row_< top_bound; row_++)
    {
        CrRow = &iTS->YImg[GetImageDataIndex(row_)];

        //for(col = left_bound;col < right_bound; col++)
        for(col = iTS->O_LaneMBound[row_].Frt - 20 ;col < iTS->O_LaneMBound[row_].Scd + 20; col++)
        {
            gray_level[CrRow[col]]++;
            Avg += CrRow[col];
            Total++;
        }
    }

    Avg = Avg / Total;

    for(i = 0; i <= 255; i++){
        Probability[i] = ((float)gray_level[i] / (float)Total);
        probabilityDistribution[i] = Probability[i];
    }

    for(i = 1; i <= 255; i++)
        probabilityDistribution[i] += probabilityDistribution[i-1];

    for(i = 0; i <= 255; i++){
        W1 = probabilityDistribution[i];
        W2 = 1 - probabilityDistribution[i];
        if(W1 == W1_old || W1 == 0){
            W1_old = W1;
            continue;
        }
        for(int j = 0; j <= 255; j++){
            if(j <= i)
                u1 += (Probability[j]/W1)*j;
            else
                u2 += (Probability[j]/W2)*j;
        }
        Ans[i] = (W1*((u1-Avg)*(u1-Avg)))+(W2*((u2-Avg)*(u2-Avg)));
        if(Ans[i] > max){
            max = Ans[i];
            iTS->O_MarkLightTH = i;
        }
        u1 = 0;
        u2 = 0;
        W1_old = W1;
    }

    if(iTS->O_MarkLightTH < 120)
        iTS->O_MarkLightTH = 120;
}

//#define O_SHOW_TRI_LEVEL_RESULT
void O_CreateRtgRoadLightShadowInfo(short LightTH,short ShadowTH,short Srow_,short Erow_,short Scol,short Ecol,ITS *iTS)
{
    short col,row_;
    unsigned char *CrRow,*Result;
    int r = 0;

//DSP debugging
#ifdef O_SHOW_TRI_LEVEL_RESULT
    unsigned char *Output;
#endif

    Srow_ = LimitH(Srow_);
    r = SinkDataIndexStart(Srow_);

    for(row_ = Srow_; row_ <= Erow_ && row_ <= S_IMGTB; row_++)
    {
        r=SinkDataIndexNextRow(r);
        CrRow =&iTS->YImg[r];
        Result=&iTS->O_InfoPlane[r];

#ifdef O_SHOW_TRI_LEVEL_RESULT
        //Output = &iTS->pc_based_debug_image[r];    //PC
        Output = &iTS->Showimage[r];
#endif

        for(col = Scol; col <= Ecol && col >= S_IMGLB && col <= S_IMGRB; col++)
        {
            Result[col] &= ~RSDRLTINFO;
            if(CrRow[col] >= LightTH)
            {
                Result[col] |= RLTINFO;

#ifdef O_SHOW_TRI_LEVEL_RESULT
                Output[col] = 255;
#endif

            }
            else if(CrRow[col] <= ShadowTH)
            {
                Result[col] |= RSDINFO;

#ifdef O_SHOW_TRI_LEVEL_RESULT
                Output[col] = 0;
#endif

            }

#ifdef O_SHOW_TRI_LEVEL_RESULT
            else
                Output[col] = 128;
#endif

        }
    }
}


void F_O_CreateInfoPlan(ITS *iTS)
{

    short col,row_;

    unsigned char *CrRow,*DnRow,*Dn2Row;
    unsigned char *Hog_result;

    unsigned short top_bound = S_IMGTB ;
    unsigned short bottom_bound = L_IB_BB_SearchLane + 1;
    unsigned short left_bound = S_IMGLB + 1;
    unsigned short right_bound = S_IMGRB - 1;


    int r = SinkDataIndexStart(bottom_bound);

    for(row_= bottom_bound; row_< top_bound; row_++)
    {
        r=SinkDataIndexNextRow(r);
        CrRow = &iTS->Axy_InfoPlane[r];
        DnRow = &iTS->Axy_InfoPlane[r + iTS->F_W];
        Dn2Row = &iTS->Axy_InfoPlane[r + iTS->F_W + iTS->F_W];

        Hog_result = &iTS->Hog_InfoPlane[r];

        for(col = left_bound;col < right_bound; col++)
        {
            Hog_result[col] = 0;

//            if(CrRow[col] != 0)
            {
                Hog_result[col] = CrRow[col] + CrRow[col + 1] +
                                  DnRow[col] + DnRow[col + 1];

//                Hog_result[col] = CrRow[col] + CrRow[col + 1] + CrRow[col + 2] +
//                                  DnRow[col] + DnRow[col + 1] + DnRow[col + 2] +
//                                  Dn2Row[col] + Dn2Row[col + 1] + Dn2Row[col + 2];

            }
        }
    }


//    int r = SinkDataIndexStart(bottom_bound);
//    for(row_= bottom_bound; row_< top_bound; row_++)
//    {
//        r=SinkDataIndexNextRow(r);

////        Gxy_result = &Gxy_InfoPlane[r];
//        O_Result = &iTS->O_P_InfoPlane[r];
//        Axy_result = &iTS->Axy_InfoPlane[r];


//        for(col = left_bound;col < right_bound; col++)
//        {
//            //initialization
////            O_Result[col] = (unsigned char)(Gxy_result[col] * par);

////            if(Gxy_result[col] > 255)
////                O_Result[col] = 255;
////            else if(Gxy_result[col] < 0)
////                O_Result[col] = 0;
////            else
////                O_Result[col] = Gxy_result[col];


//            //initialization
//            switch (Axy_result[col])
//            {
//                case 1:
//                    O_Result[col] = 0x01;
//                    break;
//                case 2:
//                    O_Result[col] = 0x02;
//                    break;
//                case 3:
//                    O_Result[col] = 0x04;
//                    break;
//                case 4:
//                    O_Result[col] = 0x08;
//                    break;
//                case 5:
//                    O_Result[col] = 0x09;
//                    break;
//                case 6:
//                    O_Result[col] = 0x10;
//                    break;
//                case 7:
//                    O_Result[col] = 0x20;
//                    break;
//                case 8:
//                    O_Result[col] = 0x40;
//                    break;
//                case 9:
//                    O_Result[col] = 0x80;
//                    break;
//                case 0:
//                    O_Result[col] = 0x5F;
//                    break;
//                default:
//                    O_Result[col] = 0xFF;
//                    break;
//            }

//        }
//    }


}
