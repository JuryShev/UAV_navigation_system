#ifndef KALMAN_EXTENDED_H_INCLUDED
#define KALMAN_EXTENDED_H_INCLUDED

#include "matrix.h"
#include "matrix2.h"

MAT   *S_R;
MAT   *S_Pp;
MAT   *S_Q;
MAT   *F;
MAT   *G;
MAT   *S_H;//init in the "Function_inovation"(cause-Trp)
MAT   *x;
MAT   *SQd;
MAT   *Tr;

void Kalman_init(void); // Init Kalman
void F_update(double upd[], mf16 *DCMbn, double RM_RN[], double dt);//updates Kalman filter transition matrix F
void kalman(mf16 *z,  double dtg);// Kalman filter
void conf_cor(void);//configuration_ins_correction




#endif // KALMAN_EXTENDED_H_INCLUDED
