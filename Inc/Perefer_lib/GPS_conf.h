#ifndef GPS_CONF_H_INCLUDED
#define GPS_CONF_H_INCLUDED

/**__________GPS_CONF________________________*/
double gps_stdm[3]={5,5,10};
double gps_stdv[3]={0.0514, 0.0514, 0.0514};
double gps_larm[3]={0,0,0};
double gps_std[3]={0.0000007,0.0000009, 10};
double gps_freq;
double gps_t;
double gps_lat;
double gps_lon;
double gps_h;
double gps_vel;
/**__________________________________________*/

/**__________INS_CONF________________________*/
float imu_arw[3]={0.00058178, 0.00058178, 0.00058178};
float imu_arww[3]={0,0,0};
float imu_vrw [3]={0.0033, 0.0033,0.0033};
float imu_vrrw [3]={0, 0,0};
float imu_gp_fix [3]={0.0524, 0.0524, 0.0524};
float imu_ap_fix [3]={0.4905, 0.4905,0.4905,};
float imu_gb_drift[3]={0.000122,0.000122, 0.000122};
float imu_ab_drift[3]={0.0020, 0.0020, 0.0020};
float imu_gb_corr[3]={100,100,100};
float imu_ab_corr[3]={100,100,100};
float imu_freq=100;
float imu_astd[3]={0.0033, 0.0033,0.0033};
float imu_gstd[3]={0.0058,0.0058,0.0058};
float imu_gpsd[3]={0.0012, 0.0012,0.0012};
float imu_apsd[3]={0.0196, 0.0196,0.0196 };
float imu_ini_align_err[3]={0.0524, 0.0524, 0.1745};
float imu_ini_align[3]={0,0,0.2618};
float imu_fb;
float imu_wb;
/**__________________________________________*/





#endif // GPS_CONF_H_INCLUDED
