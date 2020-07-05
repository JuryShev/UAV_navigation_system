#include <math.h>
#include "NAVgo_earthrate.h"
#include "fix16.h"
#include "fixarray.h"
#include "fixquat.h"
#include "fixstring.h"
#include "fixmatrix.h"
#include "unittests.h"
#include "Kalman_extended.h"
#include "GPS_conf.h"


//____________
void conf_cor(void)
{
   int numb;
   int k=0;

    for( numb=0; numb<=2; numb++)
    {
        imu_gp_fix [numb]=x->me[numb+9][0];
        imu_ap_fix [numb]=x->me[numb+12][0];
        imu_gb_drift [numb]=x->me[numb+15][0];
        imu_gb_drift [numb]=x->me[numb+18][0];


    }
}
//___________________________________________________________________
//______________Kalman_Init___________________________________________
void Kalman_init(void)
{
     S_R=m_get(6,6);
     S_Pp=m_get(21,21);
     F=m_get(21,21);
     G=m_get(21,12);
     x=m_get(21,1);
     S_Q=m_get(12,12);
     unsigned char i;

     for(i=0; i<=5; i++)
    {
        if (i<=2)
        {
            S_R->me[i][i]=gps_stdv[i]*gps_stdv[i];
            S_Pp->me[i][i]=imu_ini_align_err[i]*imu_ini_align_err[i];
            S_Pp->me[i+3][i+3]=gps_stdv[i]*gps_stdv[i];
            S_Pp->me[i+6][i+6]=gps_std[i]*gps_std[i];
            S_Pp->me[i+9][i+9]=imu_gp_fix[i]*imu_gp_fix[i];
            x->me[i+9][0]=imu_gp_fix[i];
            x->me[i+12][0]=imu_ap_fix[i];
            S_Q->me[i][i]=imu_arw[i]*imu_arw[i];
            S_Q->me[i+3][i+3]=imu_vrw [i]*imu_vrw [i];



        }
        else
        {
            S_R->me[i][i]=gps_stdm[i-3]*gps_stdm[i-3];
            S_Pp->me[i+9][i+9]=imu_ap_fix [i-3]*imu_ap_fix [i-3];
            S_Pp->me[i+12][i+12]=imu_gb_drift [i-3]*imu_gb_drift [i-3];
            S_Pp->me[i+15][i+15]=imu_ab_drift [i-3]*imu_ab_drift [i-3];
            x->me[i+12][0]=imu_gb_drift [i-3];
            x->me[i+15][0]=imu_ab_drift [i-3];
            S_Q->me[i+3][i+3]=imu_gpsd[i-3]*imu_gpsd[i-3];
            S_Q->me[i+6][i+6]=imu_apsd[i-3]*imu_apsd[i-3];

        }
    }


}
//________________________________________________________________________


//_____________________updates Kalman filter transition matrix F__________
void F_update(double upd[], mf16 *DCMbn, double RM_RN[], double dt)
{

    MAT *I;
    //MAT *Z;
    I=m_get(3,3);
    m_ident(I); // is the N-by-N identity matrix.
    //Z=m_get(3,3);
    float Om=0.0000729;
    float RO;
    float a[3][3];
    uint8_t n, n2=6;

    radius(upd[3],  RM_RN);
    RO=sqrt((RM_RN[0]*RM_RN[1]));
    //printf("RO=%f=\n",RO);
    //a[0][0]=0;
    a[0][1]=-((Om * sin(upd[3])) + (upd[1]/(RO) * tan(upd[3])));
    a[0][2]=upd[0]/(RO);
    a[1][0]=(Om * sin(upd[3])) + (upd[1]/(RO) * tan(upd[3]) );
    //a[1][1]=0;
    a[1][2]=(Om * cos(upd[3])) + (upd[1]/(RO)) ;
    a[2][0]=(-upd[0])/RO;
    a[2][1]=(-Om * cos(upd[3]) - (upd[1]/(RO)));
    //a[2][2]=0;

///________________Fee__________________________________________
    //  F[0][0]         F[0][1]             F[0][2]
                F->me[0][1]=a[0][1];    F->me[0][2]=a[0][2];
    //  F[1][0]         F[1][1]             F[1][2]
    F->me[1][0]=a[1][0];                F->me[1][2]=a[1][2];
    //  F[2][0]         F[2][1]             F[2][2]
    F->me[2][0]=a[2][0]; F->me[2][1]=a[2][1];

///_____________________________________________________________

    //a[0][0]=0;
    a[0][1]=1/(RO);
    //a[0][2]=0;
    a[1][0]=-a[0][1];
    //a[1][1]=0;
    //a[1][2]=0;
    //a[2][0]=0;
    a[2][1]=-tan(upd[3])/RO;
    //a[2][2]=0;

///________________Fev__________________________________________
    //  F[0][3]         F[0][4]             F[0][5]
                F->me[0][4]=a[0][1];
    //  F[1][3]         F[1][4]             F[1][5]
    F->me[1][3]=a[1][0];
    //  F[2][3]         F[2][4]             F[2][5]
                F->me[2][4]=a[2][1];

///_____________________________________________________________

    a[0][0]=-Om * sin(upd[3]);
    //a[0][1]=0;
    a[0][2]=-upd[1]/(RO*RO);
    //a[1][0]=0;
    //a[1][1]=0;
    a[1][2]=upd[0]/(RO*RO);
    a[2][0]=-Om*cos(upd[3]) - (upd[1]/((RO)*(cos(upd[3])*cos(upd[3]))));
    //a[2][1]=0 ;
    a[2][2]=(upd[1] * tan(upd[3])) / (RO*RO);

///________________Fep__________________________________________
    //  F[0][6]         F[0][7]             F[0][8]
    F->me[0][6]=a[0][0];                F->me[0][8]=a[0][2];
    //  F[1][6]         F[1][7]             F[1][8]
                                        F->me[1][8]=a[1][2];
    //  F[2][6]         F[2][7]             F[2][8]
    F->me[2][6]=a[2][0];                F->me[2][8]=a[2][2];

///_____________________________________________________________

    //a[0][0]=0;
    a[0][1]=-upd[7] ;
    a[0][2]=upd[6];
    a[1][0]=upd[7];
    //a[1][1]=0;
    a[1][2]=-upd[5];
    a[2][0]=-upd[6];
    a[2][1]=upd[5];
    //a[2][2]=0;

///________________Fve__________________________________________
    //  F[3][0]         F[3][1]             F[3][2]
                F->me[3][1]=a[0][1];    F->me[3][2]=a[0][2];
    //  F[4][0]         F[4][1]             F[4][2]
    F->me[4][0]=a[1][0];                F->me[4][2]=a[1][2];
    //  F[5][0]         F[5][1]             F[5][2]
    F->me[5][0]=a[2][0]; F->me[5][1]=a[2][1];


///_____________________________________________________________

    a[0][0]=upd[2]/(RO);
    a[0][1]=-2*((Om * sin(upd[3])) + ((upd[1]/(RO)) * tan(upd[3]))) ;
    a[0][2]=upd[0]/RO ;
    a[1][0]=(2*Om * sin(upd[3])) + ( (upd[1]/(RO)) * tan(upd[3]) );
    a[1][1]=(1/(RO)) * ((upd[0] * tan(upd[3])) + upd[2]);
    a[1][2]=2*Om * cos(upd[3]) + (upd[1]/(RO));
    a[2][0]=(-2*upd[0])/(RO);
    a[2][1]=-2*(Om * cos(upd[3]) +  (upd[1]/(RO))) ;
    //a[2][2]=0;

///________________Fvv__________________________________________
    //  F[3][3]         F[3][4]             F[3][5]
    F->me[3][3]=a[0][0]; F->me[3][4]=a[0][1]; F->me[3][5]=a[0][2];
    //  F[4][3]         F[4][4]             F[4][5]
    F->me[4][3]=a[1][0]; F->me[4][4]=a[1][1]; F->me[4][5]=a[1][2];
    //  F[5][3]         F[5][4]             F[5][5]
    F->me[5][3]=a[2][0]; F->me[5][4]=a[2][1];

///_____________________________________________________________

    a[0][0]=-upd[1]*( (2*Om * cos(upd[3])) + (upd[1]/((RO)*(cos(upd[3])*cos(upd[3])))));
    //a[0][1]=0 ;
    a[0][2]=1 / (RO*RO) * ( ((upd[1]*upd[1]) * tan(upd[3])) - (upd[0] * upd[2]) );
    a[1][0]=2*Om * ( (upd[0] * cos(upd[3])) - (upd[2] * sin(upd[3])) ) + ( (upd[0] * upd[1]) / (RO * (cos(upd[3])*cos(upd[3]))) );
    //a[1][1]=0;
    a[1][2]=-(upd[1]/(RO*RO)) * (upd[0]*tan(upd[3])+upd[2]);
    a[2][0]=2 * Om * upd[1] * sin(upd[3]);
    //a[2][1]=0;
    a[2][2]=1/((RO*RO)) * ((upd[0]*upd[0]) + (upd[1]*upd[1]));

///________________Fvp__________________________________________
    //  F[3][6]         F[3][7]             F[3][8]
    F->me[3][6]=a[0][0];                F->me[3][8]=a[0][2];
    //  F[4][6]         F[4][7]             F[4][8]
    F->me[4][6]=a[1][0];                F->me[4][8]=a[1][2];
    //  F[5][6]         F[5][7]             F[5][8]
    F->me[5][6]=a[2][0];                F->me[5][8]=a[2][2];

///_____________________________________________________________

    a[0][0]=1/(RO);
   // a[0][1]=0;
    a[0][2]=0;
   // a[1][0]=0;
    a[1][1]=1/((RO)*cos(upd[3]));
   // a[1][2]=0;
    //a[2][0]=0;
    //a[2][1]=0;
    a[2][2]=-1;
///________________Fpv__________________________________________
    //  F[6][3]         F[6][4]             F[6][5]
    F->me[6][3]=a[0][0];
    //  F[7][3]         F[7][4]             F[7][5]
                    F->me[7][4]=a[1][1];
    //  F[8][3]         F[8][4]             F[8][5]
                                        F->me[8][5]=a[2][2];

///_____________________________________________________________

    //a[0][0]=0;
    //a[0][1]=0;
    a[0][2]=-upd[0]/(RO*RO);;
    a[1][0]=(upd[1]*tan(upd[3])) / (RO * cos(upd[3]));
    //a[1][1]=0;
    a[1][2]=-upd[1] / ((RO*RO) * cos(upd[3]));
    //a[2][0]=0;
   // a[2][1]=0;
    //a[2][2]=0;
///________________Fpp__________________________________________
    //  F[6][6]         F[6][7]             F[6][8]
                                        F->me[6][8]=a[0][2];
    //  F[7][6]         F[7][7]             F[7][8]
    F->me[7][6]=a[1][0];                F->me[7][8]=a[1][2];
    //  F[8][6]         F[8][7]             F[8][8]


///_____________________________________________________________

//15 16 17
//18 19 20

//DM-7 8 9| 0 1 2
//DM-15 16 17| 0 1 2

//-DM 12 13 14| 3 4 5
//DM 18 19 20| 3 4 5
///________________Faa&Fgg______________________________________
    for(n=15; n<=17; n++)
    {

        F->me[n][n]=1/imu_ab_corr[n-15];
        F->me[n+3][n+3]=1/imu_gb_corr[n-15];

        // проверить att_update
        F->me[0][n]=fix16_to_dbl(DCMbn->data[0][n-15]);
        F->me[1][n]=fix16_to_dbl(DCMbn->data[1][n-15]);
        F->me[2][n]=fix16_to_dbl(DCMbn->data[2][n-15]);

        F->me[3][n-3]=-F->me[0][n];
        F->me[4][n-3]=-F->me[1][n];
        F->me[5][n-3]=-F->me[2][n];

        F->me[3][n+3]=-F->me[0][n];
        F->me[4][n+3]=-F->me[1][n];
        F->me[5][n+3]=-F->me[2][n];

        F->me[0][n-6]=F->me[0][n];
        F->me[1][n-6]=F->me[1][n];
        F->me[2][n-6]=F->me[2][n];

        G->me[0][n-15]=F->me[0][n]; G->me[3][n-12]=-F->me[0][n];
        G->me[1][n-15]=F->me[1][n]; G->me[4][n-12]=-F->me[1][n];
        G->me[2][n-15]=F->me[2][n]; G->me[5][n-12]=-F->me[2][n];

        G->me[n][n2]=1;
        G->me[n+3][n2+3]=1;
        n2++;



    }

///_____________________________________________________________


}
//___________________________________________________________________________________

void kalman(mf16 *z,  double dtg)
{

    MAT *I;
    MAT *A;
    MAT *SQd0;//buf

    char size;
    Tr=m_get(21,21);

    if(F->max_m>F->max_n)
        I=m_get(F->max_m,F->max_m);
    else
        I=m_get((F->max_n),(F->max_n));

    m_ident(I);

    //Discretization of continous-time system
    SQd=m_get(21,21);
    A=m_get(21,21);
    sm_mlt(dtg,F,F);
    m_exp(F, 0.1, A );
    m_mlt(G, S_Q, SQd); mmtr_mlt(SQd, G, SQd0);   sm_mlt(dtg,SQd0,SQd);

    //Step 1, update the a priori covariance matrix Pi
    MAT *S_Pi;
    S_Pi=m_get(21,21);
   // mmtr_mlt(S_Pp, A, S_Pi); m_mlt(S_Pi, A, SQd0); m_add(SQd0,SQd,S_Pi);
    m_mlt(A, S_Pp, SQd0); mmtr_mlt(SQd0, A, S_Pi); m_add(S_Pi,SQd,S_Pi);
    m_transp(S_Pi,SQd0); m_add(S_Pi, SQd0, S_Pi); sm_mlt(0.5,S_Pi,S_Pi);


    //Step 2, update Kalman gain
    MAT *S_C; S_C=m_get(6,6);
    MAT *S_K; S_K=m_get(21,6);

    m_mlt(S_H, S_Pi, S_C); mmtr_mlt(S_C, S_H, SQd0); m_add(SQd0,S_R,S_C);
    mmtr_mlt(S_Pi, S_H, SQd0);  m_inverse(S_C, S_C); m_mlt(SQd0, S_C, S_K);

    //Step 3, update the a posteriori state xp
    MAT *S_Xi; S_Xi=m_get(21,1);
    MAT *z_dbl; z_dbl=m_get(6,1);

    m_mlt(A, x, S_Xi);

    z_dbl->me[0][0]=fix16_to_dbl(z->data[0][0]);
    z_dbl->me[1][0]=fix16_to_dbl(z->data[1][0]);
    z_dbl->me[2][0]=fix16_to_dbl(z->data[2][0]);

    z_dbl->me[3][0]=fix16_to_dbl(z->data[0][1]);
    z_dbl->me[4][0]=fix16_to_dbl(z->data[1][1]);
    z_dbl->me[5][0]=fix16_to_dbl(z->data[2][1]);

    m_mlt(S_H, S_Xi, x); m_sub(z_dbl,x,z_dbl);
    m_mlt(S_K, z_dbl, x); m_add(x, S_Xi,x);


    //Step 4, update the a posteriori covariance matrix Pp
    MAT *J; J=m_get(21,21);
    MAT *J0; J0=m_get(21,21);

    m_mlt(S_K, S_H, J);
    m_sub(I,J,J);
    m_mlt( J, S_Pi, S_Pp);
    mmtr_mlt(S_Pp, J, SQd0);

    m_mlt(S_K, S_R, S_Pp);  mmtr_mlt(S_Pp, S_K, J0);
    m_add(J0, SQd0, S_Pp);



}
//_____________________updates Kalman filter transition matrix F_____________________

//___________________________________________________________________________________



