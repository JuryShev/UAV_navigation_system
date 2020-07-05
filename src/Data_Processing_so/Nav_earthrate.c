#include <math.h>
#include "NAVgo_earthrate.h"
#include "fix16.h"
#include "fixarray.h"
#include "fixquat.h"
#include "fixstring.h"
#include "fixmatrix.h"
#include "unittests.h"
#include "Kalman_extended.h"
#include "matrix.h"
#include "matrix2.h"


#define a_a 6378137.0
#define e 0.0818191908426

//static const fix16_t af=6378137.0;
//static const fix16_t EF=0.0818;

//_____компенсация_вращения_земли______________
void earthrate( double lat, double omega_ie_N[])
{

    omega_ie_N[0]= (cos(lat)*0.000072);
    omega_ie_N[1]=0*0.000072;
    omega_ie_N[2]=(-sin(lat))*0.000072;


}
//_______________________________________________

//________calculates meridian and normal radii of curvature___
void radius(double lat, double RM_RN[])
{
    double e2;
    double den;
    double sin_lat2;
    sin_lat2=sin(lat)*sin(lat);
    e2=e*e;
    den=1-e2*sin_lat2;
    //RM_RN[0]=den;

    RM_RN[1]=a_a/sqrt(den);
    den=den*den*den;
    den=sqrt(den);
    RM_RN[0]=a_a*(1-e2)/den;
}
//________________________________________________________________

//_______calculates the transport rate in the navigation frame____________________
 void transportrate(double lat, double Vn, double Ve, double h, double omega_en_N[])
 {
     double RM_RN[2]={0,0};
     radius(0.5730,RM_RN);
     omega_en_N[0]=Ve/(RM_RN[1]+h);
     omega_en_N[1]=(-(Vn/(RM_RN[0]+h)));
     omega_en_N[2]=(-(Ve*tan(lat)/(RM_RN[1]+h)));

    omega_en_N[0]=omega_en_N[0]*1000;
    omega_en_N[1]=omega_en_N[1]*1000;
    omega_en_N[2]=omega_en_N[2]*1000;

 }
//______________updates attitude using quaternion or DCM____
void att_update(double wb[],  mf16 *DCMbn, float qua[], double omega_ie_N[],  double dt)//double omega_en_N не учитываем
{
    int i;
    int j;
    double DCMbn16[3][3];
    double wb_n[3]={0,0,0};
    double wnorm;
    double co,si;
    double n1, n2,n3;
    double qw1, qw2, qw3, qw4;
    qf16 qua16 = {fix16_from_dbl(qua[0]), fix16_from_dbl(qua[1]), fix16_from_dbl(qua[2]), fix16_from_dbl(qua[3])};

    qf16_to_matrix(DCMbn,&qua16);
    
    mf16_transpose(DCMbn,DCMbn);
    

    for(j=0; j<=2; j++)
    {

        double b,c,d;
        DCMbn16[j][0]=fix16_to_dbl(DCMbn->data[j][0]);
        DCMbn16[j][1]=fix16_to_dbl(DCMbn->data[j][1]);
        DCMbn16[j][2]=fix16_to_dbl(DCMbn->data[j][2]);

        b=DCMbn16[j][0]*omega_ie_N[0];
        c=DCMbn16[j][1]*omega_ie_N[1];
        d=DCMbn16[j][2]*omega_ie_N[2];

        wb_n[j]=b+c+d;
        wb_n[j]=wb[j]-wb_n[j];
    }
    
    wnorm=sqrt(wb_n[0]*wb_n[0]+wb_n[1]*wb_n[1]+wb_n[2]*wb_n[2]);
    co=(cos(0.5*wnorm*dt));
    si=sin(0.5*wnorm*dt);

    n1=wb_n[0]/wnorm;
    n2=wb_n[1]/wnorm;
    n3=wb_n[2]/wnorm;

    qw1=n1*si;
    qw2=n2*si;
    qw3=n3*si;
    qw4=co;

    printf("qw1=%f\n,qw2=%f\n,qw3=%f\n,qw4=%f\n",qw1, qw2, qw3, qw4);
    mf16 Om = {4, 4,0,
            {{fix16_from_dbl(qw4), fix16_from_dbl(qw3), fix16_from_dbl(-qw2),fix16_from_dbl(qw1)},
             {fix16_from_dbl(-qw3), fix16_from_dbl(qw4), fix16_from_dbl(qw1), fix16_from_dbl(qw2)},
             {fix16_from_dbl(qw2), fix16_from_dbl(-qw1), fix16_from_dbl(qw4), fix16_from_dbl(qw3)},
             {fix16_from_dbl(-qw1), fix16_from_dbl(-qw2), fix16_from_dbl(-qw3), fix16_from_dbl(-qw4)}}};

    mf16 qua_T= {4, 1, 0,
            {{fix16_from_dbl(qua[1])},
             {fix16_from_dbl(qua[2])},
             {fix16_from_dbl(qua[3])},
             {fix16_from_dbl(qua[0])}}};

    mf16_mul(&qua_T, &Om, &qua_T);
    qua[0]=fix16_to_dbl(qua_T.data[3][0]);
    qua[1]=fix16_to_dbl(qua_T.data[0][0]);
    qua[2]=fix16_to_dbl(qua_T.data[1][0]);
    qua[3]=fix16_to_dbl(qua_T.data[2][0]);

    qua16.a=fix16_from_dbl(qua[0]);
    qua16.b=fix16_from_dbl(qua[1]);
    qua16.c=fix16_from_dbl(qua[2]);
    qua16.d=fix16_from_dbl(qua[3]);

    qf16_to_matrix(DCMbn,&qua16);
    mf16_transpose(DCMbn,DCMbn);

    


}

//_____calculates gravity vector in the navigation frame______________
double gravity(double lat, double h, double RM_RN[])
{
    double sin1=sin(lat);
    double sin2=sin(2*lat);
    float g0, R0, g;

   


    g0=9.780318 * ( 1 + 0.0053024*(sin1*sin1) - 0.0000059*(sin2*sin2) );

    radius(lat,  RM_RN);

    R0=sqrt(RM_RN[0]*RM_RN[1]);
      g=h/R0+1;
    g=g0/(g*g);

    

    return g;

}

//________updates velocity in the NED frame______________________________________
 void vel_update(mf16 *fn, mf16 *vel_e, mf16*vel_n, double omega_ie_N[], double g[], double dti)
 {
     mf16 S = {3, 3, 0,
            {{fix16_from_dbl(0), (-vel_e->data[2][0]), vel_e->data[1][0]},
             {vel_e->data[2][0], fix16_from_dbl(0), (-vel_e->data[0][0])},
             {(-vel_e->data[1][0]), vel_e->data[0][0], fix16_from_dbl(0)}}};

    mf16  omega_ie_N_16= {3, 1, 0,
            {{fix16_from_dbl(omega_ie_N[0]*20)},
            {fix16_from_dbl(omega_ie_N[1]*20)},
            {fix16_from_dbl(omega_ie_N[2]*20)}}};

    mf16  fn_c= {3, 1, 0,
            {{fix16_from_dbl(0)},
             {fix16_from_dbl(0)},
             {fix16_from_dbl(0)}}};
    mf16  g_16= {3, 1, 0,
            {{fix16_from_dbl(g[0])},
             {fix16_from_dbl(g[1])},
             {fix16_from_dbl(g[2])}}};
     mf16  corriolis = {3, 1, 0,
            {{fix16_from_dbl(0)},
             {fix16_from_dbl(0)},
             {fix16_from_dbl(0)}}};


    //mf16_div_s(&S, &S, fix16_from_dbl(10) );
    mf16_transpose(&omega_ie_N_16,&omega_ie_N_16);
    mf16_mul(&corriolis, &omega_ie_N_16, &S);
   // mf16_div_s(&corriolis, &corriolis, fix16_from_dbl(10) );
    mf16_transpose(&corriolis,&corriolis);
    mf16_sub(&fn_c, fn, &corriolis);
    mf16_sub(&fn_c, &fn_c, &g_16);
    mf16_mul_s(&fn_c, &fn_c, fix16_from_dbl(dti));
    mf16_add(vel_n, &fn_c, vel_e );
    mf16_transpose(vel_e, vel_n);// операция присвоения

    //printf("vel_e=\n");
    //print_mf16(stdout, vel_e);



 }
//____________________________________________________

//____updates position in the navigation frame._______
void pos_update(double pos[], mf16 *vel_e, double RM_RN[],double dt)
{
    double lat, lon,h;
    double Vn, Ve, Vd;
    double h_n, Vn_c, Ve_c, lat_n, lon_n; // update attitude
    lat=pos[0];
    lon=pos[1];
    h=pos[2];

    Vn=fix16_to_dbl(vel_e->data[0][0]);
    Ve=fix16_to_dbl(vel_e->data[0][1]);
    Vd=fix16_to_dbl(vel_e->data[0][2]);
    //printf("Vd")
    printf("h=%f\n",h);
    h_n=h-Vd*dt;
    printf("h_n=%f\n",h_n);
    if(h_n<0) h_n=0;
    radius(lat,  RM_RN);


    Vn_c=Vn/(RM_RN[0]+h_n);
    lat_n=lat+Vn_c*dt;
    radius(lat_n,  RM_RN);

    Ve_c  = Ve / ((RM_RN[1] + h_n) * cos (lat_n));
    lon_n = lon + Ve_c * dt;
    //printf("Ve_c=%f\n",Ve_c);
    //printf("lon_n=%f\n",lon_n);
    pos[0]=lat_n;
    pos[1]=lon_n;
    pos[2]=h_n;

}

/*****************INOVATON***************************/
//________________Function_inovation__________________

void inovation(double lat, double lat_g, double lon, double lon_g, double h, double h_g, double RM_RN[], mf16 *DCMbn, mf16 *vel_e, mf16 *z)
{

     radius(lat,  RM_RN);
     double i=RM_RN[0]+h;
     double j=(RM_RN[1]+h)*cos(lat);
     unsigned char k;
     double zp[3];
     double Trp[3][3]={{i,0,0},
                       {0,j,0},
                       {0,0,(-1)}};

     double pos_new[3];
     S_H=m_get(6,21);// For Kalman Filter
											 
     mf16 gps_larm = {3, 1, 0,
            {{fix16_from_dbl(0)},
             {fix16_from_dbl(0)},
             {fix16_from_dbl(0)}}};

    mf16 gps_vel = {3, 1, 0,
            {{fix16_from_dbl(0.8760)},
             {fix16_from_dbl(0.2094)},
             {fix16_from_dbl(0.0492)}}};

    mf16  zv_16= {3, 1, 0,
            {{fix16_from_dbl(0)},
             {fix16_from_dbl(0)},
             {fix16_from_dbl(0)}}};


     pos_new[0]=lat-lat_g;//lat-lat_g;
     pos_new[1]=lon-lon_g;
     pos_new[2]=h-h_g;


      for(k=0; k<=2; k++)
    {
        // при возможности уменьшить.
        double b,c,d;
        b=Trp[k][0]*pos_new[0];
        c=Trp[k][1]*pos_new[1];
        d=Trp[k][2]*pos_new[2];
        zp[k]=b+c+d;


        S_H->me[k+3][k+6]=Trp[k][k];
        S_H->me[k][k+3]=1;

    }

     mf16  zp_16= {3, 1, 0,
            {{fix16_from_dbl(zp[0])},
             {fix16_from_dbl(zp[1])},
             {fix16_from_dbl(zp[2])}}};

    mf16_mul(&gps_larm, DCMbn, &gps_larm);
    mf16_sub(&zp_16, &zp_16, &gps_larm);
    mf16_transpose(vel_e,vel_e);
    mf16_sub(&zv_16, vel_e, &gps_vel);

    for(k=0; k<=2; k++)
    {
        z->data[k][0]=zv_16.data[k][0];
        z->data[k][1]=zp_16.data[k][0];

    }

}


