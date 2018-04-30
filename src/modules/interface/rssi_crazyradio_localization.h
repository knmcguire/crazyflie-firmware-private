/*
 * rssi_crazyradio_localization.h
 *
 *  Created on: Apr 24, 2018
 *      Author: knmcguire
 */

#ifndef SRC_MODULES_INTERFACE_RSSI_CRAZYRADIO_LOCALIZATION_H_
#define SRC_MODULES_INTERFACE_RSSI_CRAZYRADIO_LOCALIZATION_H_
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "arm_math.h"


#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "system.h"


#include <radiolink.h>

#include "estimator_kalman.h"

#define EKF_N 4
#define EKF_M 3

typedef struct discrete_ekf {
  float X[EKF_N];  // state X
  float Xp[EKF_N]; // state prediction
  float Zp[EKF_M]; // measurement prediction
  float P[EKF_N][EKF_N]; // state covariance matrix
  float Pp[EKF_N][EKF_N]; // predicted state covariance matrix
  float Q[EKF_N][EKF_N]; // proces covariance noise
  float R[EKF_M][EKF_M]; // measurement covariance noise
  float H[EKF_M][EKF_N]; // jacobian of the measure wrt X
  float Ht[EKF_N][EKF_M]; // transpose of H
  float A[EKF_N][EKF_N];
  float E[EKF_M][EKF_M];
  float K[EKF_N][EKF_M];
  float tmp1[EKF_N][EKF_N];
  float tmp2[EKF_N][EKF_N];
  float tmp3[EKF_N][EKF_N];
  float tmp4[EKF_N][EKF_N];

  float dt;

} discrete_ekf;

void rssiCrazyradioLocalizationInit(void);
void rssiCrazyRadioLocalizationTask(void* arg);

void discrete_ekf_new(discrete_ekf* filter);
void discrete_ekf_predict(discrete_ekf* filter);
void discrete_ekf_update(discrete_ekf* filter, float *Z);

void linear_filter(float *Xp, float *X, float dt, float A[EKF_N][EKF_N]);
void linear_measure(float *X, float *Z, float H[EKF_M][EKF_N]);


#endif /* SRC_MODULES_INTERFACE_RSSI_CRAZYRADIO_LOCALIZATION_H_ */
