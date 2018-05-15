
#include <rssi_crazyradio_localization.h>
#include "debug.h"

uint8_t rssi;
uint8_t start_loc = 0;
static bool isInit;
static float distance;
static float pos_beacon_x;
static float pos_beacon_y;
float gamma_rrsi = 4;
float Pn = 45.0f;
ekf ekf_rl;

//static discrete_ekf ekf_rl;




void rssiCrazyradioLocalizationInit(void)
{


  if (isInit)
    return;

  ekf_rl.dt = 0.01;
  discrete_ekf_new(&ekf_rl);

  xTaskCreate(rssiCrazyRadioLocalizationTask,"RSSI_CRAZYRADIO_LOCALIZATION",ZRANGER_TASK_STACKSIZE, NULL,ZRANGER_TASK_PRI,NULL );
  isInit = true;

}

void rssiCrazyRadioLocalizationTask(void* arg)
{
  systemWaitStart();

  // Initialize EKF for relative localization

  float start_time = 0.02;
  float end_time = 0.03;
  //int h = 0;
  while(1) {
    //TODO: adjust difference in time (dt) to be actually measured from the loop;

    vTaskDelay(100);

    //DEBUG_PRINT("%d\n",rssi);
    if(start_loc == 1){
      // get range by rssi crazy radio
      float temp = (-Pn+(float)rssi)/(10*gamma_rrsi);
      distance = pow(10,temp);

      // get estimated velocities in NED
      point_t estimatedVel;
      estimatorKalmanGetEstimatedVelGlobal(&estimatedVel);



      // Filter predict with model
      discrete_ekf_predict(&ekf_rl);


      // DEBUG_PRINT("check2\n");

      // Filter update with measurements
      float Z[EKF_M];
      Z[0]=distance;
      Z[1]=estimatedVel.x;
      Z[2]=estimatedVel.y;

      discrete_ekf_update(&ekf_rl, Z);

      pos_beacon_x = ekf_rl.X[0];
      pos_beacon_y = ekf_rl.X[1];
      //pos_beacon_x = 0.0f;
      //pos_beacon_y = 0.0f;

    }
  }


}

void discrete_ekf_new(ekf* filter)
{
  uint8_t row, col;



  // Identity matrix
  for (row = 0 ; row < EKF_N ; row++ ) {


    for (col = 0 ; col < EKF_N ; col++ ) {
      // x, y, and z pos columns are affected by the range measurement
      // All other values are 1
      if (row == col ){
        filter->P[row][col] = 1.0;
        if(row ==0 || row==1)
          filter->Q[row][col] =   0.01;
        else
          filter->Q[row][col] =   0.5*0.5;
      }

      else {
        filter->P[row][col] = 0.0;
        filter->Q[row][col] = 0.0;

      }
      filter->A[row][col] = 0.0;
    }

  }


  // measurement noise matrix matrix
  for (row = 0 ; row < EKF_M ; row++ ) {

    for (col = 0 ; col < EKF_M ; col++ ) {
      // x, y, and z pos columns are affected by the range measurement
      // All other values are 1
      if (row == col ){
        if(row ==0 )
          filter->R[row][col] =   2;
        else
          filter->R[row][col] =  0.1*0.1;
      }

      else {
        filter->R[row][col] = 0.0;

      }
    }
 //   DEBUG_PRINT("%f %f %f %f\n",filter->R[row][0],filter->R[row][1],filter->R[row][2]);

  }



  // Initialize state vector
  filter->X[0] = 0;
  filter->X[1] = -2;
  filter->X[2] = 0;
  filter->X[2] = 0;

}

/* Perform the prediction step

    Predict state
      x_p = f(x);
      A = Jacobian of f(x)

    Predict P
      P = A * P * A' + Q;

    Predict measure
      z_p = h(x_p)
      H = Jacobian of h(x)

 */
void discrete_ekf_predict(ekf* filter)
{

  arm_matrix_instance_f32 _P = { EKF_N, EKF_N, (float *)filter->P};
  arm_matrix_instance_f32 _Pp = { EKF_N, EKF_N, (float *)filter->Pp};
  arm_matrix_instance_f32 _Q = { EKF_N, EKF_N, (float *)filter->Q};
  arm_matrix_instance_f32 _A = { EKF_N, EKF_N, (float *)filter->A};


  float tmp1NN[EKF_N][EKF_N];
  float tmp2NN[EKF_N][EKF_N];
  float tmp3NN[EKF_N][EKF_N];


  arm_matrix_instance_f32 _tmp1NN = { EKF_N, EKF_N, (float *)tmp1NN};
  arm_matrix_instance_f32 _tmp2NN = { EKF_N, EKF_N, (float *)tmp2NN};
  arm_matrix_instance_f32 _tmp3NN = { EKF_N, EKF_N, (float *)tmp3NN};



  // Fetch dX and A given X and dt and input u
  linear_filter(filter, filter->dt); // A = [=[NxN]

  //DEBUG_PRINT("init check%d\n", h);

  // Get measurement prediction (Zp) and Jacobian (H)
  linear_measure(filter); // H = [MxN]

  //DEBUG_PRINT("init check%d\n", h);

  //P = A * P * A' + Q
  arm_mat_mult_f32(&_A,&_P,&_tmp2NN); // tmp2 = A*P   [NxN]
  /*DEBUG_PRINT("P= ");
  int row,col;
  for (row = 0 ; row < EKF_N ; row++ ) {
  DEBUG_PRINT("%f %f %f %f\n",filter->P[row][0],filter->P[row][1],filter->P[row][2],filter->P[row][3]);
  }


  DEBUG_PRINT("tmp2= ")
  for (row = 0 ; row < EKF_N ; row++ ) {
  DEBUG_PRINT("%f %f %f %f\n",tmp2NN[row][0],tmp2NN[row][1],tmp2NN[row][2],tmp2NN[row][3]);
  }*/

  arm_mat_trans_f32(&_A,&_tmp1NN); // tmp1 = A' [NxN]
  arm_mat_mult_f32(&_tmp2NN,&_tmp1NN,&_tmp3NN); // tmp3 = tmp2*tmp1 = A*P*A' [NxN]
  arm_mat_add_f32(&_tmp3NN,&_Q,&_Pp); // P = tmp3(A*P*A') + Q [NxN]



}

/* Perform the update step

    Get Kalman Gain
      P12 = P * H';
      K = P12/(H * P12 + R);

    Update x
      x = x_p + K * (z - z_p);

    Update P
      P = (eye(numel(x)) - K * H) * P;
 */
void discrete_ekf_update(ekf* filter, float *Z) {
  arm_matrix_instance_f32 _Pp = { EKF_N, EKF_N, (float *)filter->Pp};
  arm_matrix_instance_f32 _R = { EKF_M, EKF_M, (float *)filter->R};
  arm_matrix_instance_f32 _H = { EKF_M, EKF_N, (float *)filter->H};
  arm_matrix_instance_f32 _Ht = { EKF_N, EKF_M, (float *)filter->Ht};
  arm_matrix_instance_f32 _E = { EKF_M, EKF_M, (float *)filter->E};
  arm_matrix_instance_f32 _K = { EKF_N, EKF_M, (float *)filter->K};
  arm_matrix_instance_f32 _P = { EKF_N, EKF_N, (float *)filter->P};


  float tmp1MM[EKF_M][EKF_M];
  float tmp2NM[EKF_N][EKF_M];
  float tmp3NN[EKF_N][EKF_N];
  float tmp4NN[EKF_N][EKF_N];

  arm_matrix_instance_f32 _tmp1MM = { EKF_M, EKF_M, (float *)tmp1MM};
  arm_matrix_instance_f32 _tmp2NM = { EKF_N, EKF_M, (float *)tmp2NM};
  arm_matrix_instance_f32 _tmp3NN = { EKF_N, EKF_N, (float *)tmp3NN};
  arm_matrix_instance_f32 _tmp4NN = { EKF_N, EKF_N, (float *)tmp4NN};

  //  E = H * P * H' + R

  arm_mat_trans_f32(&_H,&_Ht); // Ht = H'  [Nxm]
  arm_mat_mult_f32(&_Pp,&_Ht,&_tmp2NM); // tmp2 = Pp*H' [NxM]
  arm_mat_mult_f32(&_H,&_tmp2NM,&_tmp1MM); // tmp1 = H*Pp*H'  [MxM]
  arm_mat_add_f32(&_tmp1MM,&_R,&_E); // E = tmp1 + R  [MxM]

  // K = P * H' * inv(E)
  arm_mat_inverse_f32(&_E,&_tmp1MM); // tmp1 = inv(E)  [MxM]
  arm_mat_mult_f32(&_tmp2NM,&_tmp1MM,&_K); // K = tmp2*tmp1 = Pp*H'*inv(E)  [NxM]


  // P = P - K * H * P
  arm_mat_mult_f32(&_K,&_H,&_tmp3NN); // tmp3 = K*H [NxN]
  arm_mat_mult_f32(&_tmp3NN, &_Pp, &_tmp4NN); // tmp4 = K*H*Pp  [NxN]
  arm_mat_sub_f32(&_Pp,&_tmp4NN,&_P); // P = Pp - tmp4 = Pp-K*H*Pp  [NxN]




  /*
  filter->Zp[0] = (float)sqrt(pow(filter->X[0],2.0) + pow(filter->X[1],2.0));
  filter->Zp[1] = filter->X[2]; // x velocity of i (north)
  filter->Zp[2] = filter->X[3]; // y velocity of i (east
   */

  //  X = X + K * err
  float err[EKF_M][1];
  float dx_err[EKF_N][1];
  err[0][0]=Z[0]-filter->Zp[0]; // err = Z - Zp
  err[1][0]=Z[1]-filter->Zp[1];
  err[2][0]=Z[2]-filter->Zp[2];

  //DEBUG_PRINT("Z = %f %f %f\n",Z[0],Z[1],Z[2]);


  arm_matrix_instance_f32 _err = { EKF_M,1, (float*)err};
  arm_matrix_instance_f32 _dx_err = { EKF_N,1, (float*)dx_err};

  arm_mat_mult_f32(&_K, &_err,&_dx_err); // dx_err = K*err  [N*M][M*1]=[Nx1]]

  //dx_err[0]=filter->K[0][0]*err[0]+filter->K[0][1]

  filter->X[0] = filter->Xp[0]+dx_err[0][0]; // X = Xp + dx_err
  filter->X[1] = filter->Xp[1]+dx_err[1][0];
  filter->X[2] = filter->Xp[2]+dx_err[2][0];
  filter->X[3] = filter->Xp[3]+dx_err[3][0];

}


/* Linearized (Jacobian) filter function */
void linear_filter(ekf* filter, float dt)
{
  // Xp
  filter->Xp[0] = filter->X[0] - filter->X[2]*dt;
  filter->Xp[1] =  filter->X[1] - filter->X[3]*dt;
  filter->Xp[2] =  filter->X[2];
  filter->Xp[3] = filter-> X[3];


  // arm_matrix_instance_f32 _A = { EKF_N, EKF_N, (float *)filter->A};

  // A(x)
  filter->A[0][0] = 1.0f;
  filter->A[1][1] = 1.0f;
  filter->A[2][2] = 1.0f;
  filter->A[3][3] = 1.0f;

  filter->A[0][2] = dt;
  filter->A[1][3] = dt;

  /*
  int row,col;
  for (row = 0 ; row < EKF_N ; row++ ) {
  DEBUG_PRINT("%f %f %f %f\n",filter->A[row][0],filter->A[row][1],filter->A[row][2],filter->A[row][3]);
  }
  */

};

/* Linearized (Jacobian) measure function */
void linear_measure(ekf* filter)
{
  uint8_t row, col;
  filter->Zp[0] = (float)sqrt(pow(filter->X[0],2.0) + pow(filter->X[1],2.0));
  filter->Zp[1] = filter->X[2]; // x velocity of i (north)
  filter->Zp[2] = filter->X[3]; // y velocity of i (east)


  // Generate the Jacobian Matrix
  for (row = 0 ; row < EKF_M ; row++ ) {
    for (col = 0 ; col < EKF_N ; col++ ) {
      // x, y, and z pos columns are affected by the range measurement
      if ((row == 0) && (col == 0 || col == 1)) {
        filter->H[row][col] = filter->X[col]/(float)sqrt(pow(filter->X[0],2.0) + pow(filter->X[1],2.0));
      }

      // All other values are 1
      else if (((row == 1) && (col == 2)) ||
          ((row == 2) && (col == 3)) ){
        filter->H[row][col] = 1.0;
      }

      else {
        filter->H[row][col] = 0.0;
      }
    }
  }

  /*
  DEBUG_PRINT("H=\n");
  for (row = 0 ; row < EKF_M ; row++ ) {
  DEBUG_PRINT("%f %f %f %f\n",filter->H[row][0],filter->H[row][1],filter->H[row][2],filter->H[row][3]);
  }
*/


};



LOG_GROUP_START(rssiCR)
LOG_ADD(LOG_UINT8, rssi, &rssi)
LOG_ADD(LOG_FLOAT, distance, &distance)
LOG_ADD(LOG_FLOAT, pos_x, &pos_beacon_x)
LOG_ADD(LOG_FLOAT, pos_y, &pos_beacon_y)
LOG_GROUP_STOP(rssiCR)

PARAM_GROUP_START(rssiCR)
PARAM_ADD(PARAM_UINT8, start, &start_loc)
PARAM_ADD(PARAM_FLOAT, gamma_rrsi, &gamma_rrsi)
PARAM_ADD(PARAM_FLOAT, Pn, &Pn)
PARAM_GROUP_STOP(rssiCR)
