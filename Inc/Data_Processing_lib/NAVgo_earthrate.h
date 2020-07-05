#ifndef NAVGO_EARTHRATE_H_INCLUDED
#define NAVGO_EARTHRATE_H_INCLUDED
#include "fix16.h"
#include "fixmatrix.h"
#include <stdint.h>
#include <stdbool.h>

static const fix16_t EF=0.0818;

void earthrate( double lat, double omega_ie_N[]);//компенсация_вращения_земли
void radius(double lat, double RM_RN[]);//calculates meridian and normal radii of curvature
void transportrate(double lat, double Vn, double Ve, double h,double omega_en_N[]);//calculates the transport rate in the navigation frame
void earthrate2(double value, mf16 *omega_ie_N2);
void att_update(double wb[],  mf16 *DCMbn, float qua[], double omega_ie_N[],  double dt);
double gravity(double lat, double h, double RM_RN[]);//calculates gravity vector in the navigation frame.
void vel_update(mf16 *fn, mf16 *vel_e, mf16*vel_n, double omega_ie_N[], double g[], double dti);//updates velocity in the NED frame.
void pos_update(double pos[], mf16 *vel_e, double RM_RN[],double dt);//updates position in the navigation frame.
void inovation(double lat, double lat_g, double lon, double lon_g, double h, double h_g, double RM_RN[],mf16 *DCMbn, mf16 *vel_e, mf16 *z);

#endif // NAVGO_EARTHRATE_H_INCLUDED





//_________________2______________________
//_________________2_____________________
/*void earthrate2( double value, mf16 *omega_ie_N2)
{
     int row, column;
     row=0;
     column=0;
     value=fix16_from_dbl(value);
    omega_ie_N2->data[row][column]=fix16_cos(value)*0.0072;
    omega_ie_N2->data[1][0]=value*0;
    omega_ie_N2->data[2][0]=(-fix16_sin(value)*0.072);
    double ie_d=fix16_to_dbl(omega_ie_N2->data[2][0]);
    //omega_ie_N2->data[0][0]=value;
    //mf16_fill(lat,&omega_ie_N2);
    //printf("ie_d=%f\n",ie_d/1000);
    //print_mf16(stdout, omega_ie_N2);

}
//_______________________________________________

//________calculates meridian and normal radii of curvature___
void radius2(double lat, double RM_RN2[], double ef)
{
    //static const fix16_t EF=0.0818;
    double af=6378137.0;
     af=af/10000;
    lat=fix16_from_dbl(lat);
    ef=fix16_from_dbl(ef);
    af=fix16_from_dbl(af);
    //double e2=5;
    fix16_t e2=fix16_from_dbl(0);
    fix16_t j=fix16_from_dbl(1);
    //double den;
    fix16_t den=fix16_from_dbl(0);
    //double sin_lat2;
    fix16_t sin_lat2=fix16_from_dbl(0);

    sin_lat2=fix16_sin(lat);*fix16_sin(lat);
    sin_lat2=fix16_mul(sin_lat2,sin_lat2);
    e2=fix16_mul(ef,ef);
    den=j-fix16_mul(e2, sin_lat2);
    //RM_RN2->data[0][0]=den;
    //print_mf16(stdout, RM_RN2);
    //double den_d=fix16_to_dbl(den);
    den=fix16_div(af,fix16_sqrt(den));

    double den_d=fix16_to_dbl(den);
    RM_RN2[0]=den_d*10000;
    //printf("den_d=%f",den_d);
    //RM_RN2->data[1][0]=af/fix16_sqrt(den);
    //den=den*den*den;
    den=fix16_mul(fix16_mul(den,den),den);
    den=fix16_sqrt(den);
    RM_RN2->data[0][0]=af*(1-e2)/den;

    print_mf16(stdout, RM_RN2);
}
//________________________________________________________________

//_______calculates the transport rate in the navigation frame____________________
 void transportrate2(double lat, double Vn, double Ve, double h, double omega_en_N[])
 {
     double RM_RN[2]={0,0};
     radius(0.5730,RM_RN);
     omega_en_N[0]=Ve/(RM_RN[1]+h);
     omega_en_N[1]=(-(Vn/(RM_RN[0]+h)));
     omega_en_N[2]=(-(Ve*tan(lat)/(RM_RN[1]+h)));

    omega_en_N[0]=omega_en_N[0]*1000;
    omega_en_N[1]=omega_en_N[1]*1000;
    omega_en_N[2]=omega_en_N[2]*1000;

 }*/
//______________updates attitude using quaternion or DCM____
//______________________________________________________________________________________
