
#include <rssi_crazyradio_localization.h>


uint8_t rssi;
static bool isInit;
static float distance;
static float pos_beacon_x;
static float pos_beacon_y;
float gamma_rrsi = 1.8f;
float Pn = 45.0f;


//static discrete_ekf ekf_rl;




void rssiCrazyradioLocalizationInit(void)
{

  if (isInit)
    return;
  xTaskCreate(rssiCrazyRadioLocalizationTask,"RSSI_CRAZYRADIO_LOCALIZATION",ZRANGER_TASK_STACKSIZE, NULL,ZRANGER_TASK_PRI,NULL );
  isInit = true;

}

void rssiCrazyRadioLocalizationTask(void* arg)
{
  systemWaitStart();

  // Initialize EKF for relative localization
  discrete_ekf ekf_rl;
  discrete_ekf_new(&ekf_rl);

  while(1) {
    vTaskDelay(10);

    // get range by rssi crazy radio
    float temp = (-Pn+(float)rssi)/(10*gamma_rrsi);
    distance = 100*pow(10,temp);

    // get estimated velocities in NED
    point_t estimatedVel;
    estimatorKalmanGetEstimatedVel(&estimatedVel);

    // Filter predict with model
    discrete_ekf_predict(&ekf_rl);

    // Filter update with measurements
    float Z[EKF_M];
    Z[0]=distance;
    Z[1]=estimatedVel.x;
    Z[2]=estimatedVel.y;

    discrete_ekf_update(&ekf_rl, Z);

    pos_beacon_x = ekf_rl.X[0];
    pos_beacon_y = ekf_rl.Y[0];

  }


}

void discrete_ekf_new(discrete_ekf* filter)
{

  // Initialize Covariance matrix
  filter->P[0][0] = 1.0;
  filter->P[1][1] = 1.0;
  filter->P[2][2] = 1.0;
  filter->P[3][3] = 1.0;

  // Initialize Process noise matrix
  filter->Q[0][0]=0.01;
  filter->Q[1][1]=0.01;
  filter->Q[2][2]=0.3*0.3;
  filter->Q[3][3]=0.3*0.3;

  // Initialize Measurements Noise matrix
  filter->R[0][0]=0.2;
  filter->R[1][1]=0.1*0.1;
  filter->R[2][2]=0.1*0.1;

  // Initialize state vector
  filter->X[0] = 2.5;
  filter->X[1] = 0;
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
void discrete_ekf_predict(discrete_ekf* filter)
{
  arm_matrix_instance_f32 _P = { EKF_N, EKF_N, (float *)filter->P};
  arm_matrix_instance_f32 _Pp = { EKF_N, EKF_N, (float *)filter->Pp};
  arm_matrix_instance_f32 _Q = { EKF_N, EKF_N, (float *)filter->Q};
  arm_matrix_instance_f32 _A = { EKF_N, EKF_N, (float *)filter->A};

  arm_matrix_instance_f32 _tmp1 = { EKF_N, EKF_N, (float *)filter->tmp1};
  arm_matrix_instance_f32 _tmp2 = { EKF_N, EKF_N, (float *)filter->tmp2};
  arm_matrix_instance_f32 _tmp3 = { EKF_N, EKF_N, (float *)filter->tmp3};

  // Fetch dX and A given X and dt and input u
  linear_filter(filter->Xp, filter->X, filter->dt, filter->A); // A = [=[NxN]


  // Get measurement prediction (Zp) and Jacobian (H)
  linear_measure(filter->Xp, filter->Zp, filter->H); // H = [MxN]


  //P = A * P * A' + Q
  arm_mat_mult_f32(&_A,&_P,&_tmp2); // tmp2 = A*P   [NxN]
  arm_mat_trans_f32(&_A,&_tmp1); // tmp1 = A' [NxN]
  arm_mat_mult_f32(&_tmp2,&_tmp1,&_tmp3); // tmp3 = tmp2*tmp1 = A*P*A' [NxN]
  arm_mat_add_f32(&_tmp3,&_Q,&_Pp); // P = tmp3(A*P*A') + Q [NxN]
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
void discrete_ekf_update(discrete_ekf* filter, float *Z) {
  arm_matrix_instance_f32 _P = { EKF_N, EKF_N, (float *)filter->P};
  arm_matrix_instance_f32 _Pp = { EKF_N, EKF_N, (float *)filter->Pp};
  arm_matrix_instance_f32 _R = { EKF_M, EKF_M, (float *)filter->R};
  arm_matrix_instance_f32 _H = { EKF_M, EKF_N, (float *)filter->H};
  arm_matrix_instance_f32 _Ht = { EKF_N, EKF_M, (float *)filter->Ht};
  arm_matrix_instance_f32 _K = { EKF_N, EKF_M, (float *)filter->K};
  arm_matrix_instance_f32 _E = { EKF_M, EKF_M, (float *)filter->E};

  arm_matrix_instance_f32 _tmp1 = { EKF_M, EKF_M, (float *)filter->tmp1};
  arm_matrix_instance_f32 _tmp2 = { EKF_N, EKF_M, (float *)filter->tmp2};
  arm_matrix_instance_f32 _tmp3 = { EKF_N, EKF_N, (float *)filter->tmp3};
  arm_matrix_instance_f32 _tmp4 = { EKF_N, EKF_N, (float *)filter->tmp4};

  //  E = H * P * H' + R

  arm_mat_trans_f32(&_H,&_Ht); // Ht = H'  [Nxm]
  arm_mat_mult_f32(&_Pp,&_Ht,&_tmp2); // tmp2 = Pp*H' [NxM]
  arm_mat_mult_f32(&_H,&_tmp2,&_tmp1); // tmp1 = H*Pp*H'  [MxM]
  arm_mat_add_f32(&_tmp1,&_R,&_E); // E = tmp1 + R  [MxM]

  // K = P * H' * inv(E)
  arm_mat_inverse_f32(&_E,&_tmp1); // tmp1 = inv(E)  [MxM]
  arm_mat_mult_f32(&_tmp2,&_tmp1,&_K); // K = tmp2*tmp1 = Pp*H'*inv(E)  [NxM]


  // P = P - K * H * P
  arm_mat_mult_f32(&_K,&_H,&_tmp3); // tmp3 = K*H [NxN]
  arm_mat_mult_f32(&_tmp3, &_Pp, &_tmp4); // tmp4 = K*H*Pp  [NxN]
  arm_mat_sub_f32(&_Pp,&_tmp4,&_P); // P = Pp - tmp4 = Pp-K*H*Pp  [NxN]


  //  X = X + K * err
  float err[EKF_M];
  float dx_err[EKF_N];

  err[0]=Z[0]-filter->Zp[0]; // err = Z - Zp
  err[1]=Z[1]-filter->Zp[1];
  err[2]=Z[2]-filter->Zp[2];


  arm_matrix_instance_f32 _err = { EKF_M,1, err};
  arm_matrix_instance_f32 _dx_err = { EKF_N,1, dx_err};
  arm_mat_mult_f32(&_K, &_err,&_dx_err); // dx_err = K*err  [Nx1]]


  filter->X[0] = filter->Xp[0]+dx_err[0]; // X = Xp + dx_err
  filter->X[1] = filter->Xp[1]+dx_err[1];
  filter->X[2] = filter->Xp[2]+dx_err[2];

}


/* Linearized (Jacobian) filter function */
void linear_filter(float *Xp, float *X, float dt, float A[EKF_N][EKF_N])
{
  // Xp
  Xp[0] = X[0] -X[2]*dt;
  Xp[1] = X[1] -X[3]*dt;
  Xp[2] = X[2];
  Xp[3] = X[3];

  // A(x)
  A[0][0] = 1.0f;
  A[1][1] = 1.0f;
  A[2][2] = 1.0f;
  A[3][3] = 1.0f;

  A[0][2] = -dt;
  A[1][3] = -dt;
};

/* Linearized (Jacobian) measure function */
void linear_measure(float *X, float *Z, float H[EKF_M][EKF_N])
{
  uint8_t row, col;
  Z[0] = (float)sqrt(pow(X[0],2.0) + pow(X[1],2.0));
  Z[1] = X[2]; // x velocity of i (north)
  Z[2] = X[3]; // y velocity of i (east)


  // Generate the Jacobian Matrix
  for (row = 0 ; row < EKF_M ; row++ ) {
    for (col = 0 ; col < EKF_N ; col++ ) {
      // x, y, and z pos columns are affected by the range measurement
      if ((row == 0) && (col == 0 || col == 1)) {
        H[row][col] = X[col]/(float)sqrt(pow(X[0],2.0) + pow(X[1],2.0));
      }

      // All other values are 1
      else if (((row == 1) && (col == 2)) ||
          ((row == 2) && (col == 3)) ){
        H[row][col] = 1.0;
      }

      else {
        H[row][col] = 0.0;
      }
    }
  }

};

LOG_GROUP_START(rssiCrazyradio)
LOG_ADD(LOG_UINT8, rssi, &rssi)
LOG_ADD(LOG_FLOAT, distance, &distance)
LOG_ADD(LOG_FLOAT, pos_beacon_x, &pos_beacon_x)
LOG_ADD(LOG_FLOAT, pos_beacon_y, &pos_beacon_y)
LOG_GROUP_STOP(rssiCrazyradio)
