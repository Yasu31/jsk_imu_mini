/*
******************************************************************************
* File Name          : estimator.h
* Description        : super  class for attitude estiamte algorithm
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ATTITUDE_ESTIMATOR_H
#define __ATTITUDE_ESTIMATOR_H

#include <stdint.h>
#include <math.h>
/* #include "arm_math.h" */

#define DELTA_T 0.001f

class EstimatorAlgorithm
{
public:
  EstimatorAlgorithm()
    : acc_b_()
    , acc_v_()
    , gyro_b_()
    , gyro_v_()
    , mag_b_()
    , mag_v_()
    , q_()
    , desire_attitude_roll_(0)
    , desire_attitude_pitch_(0)
    , abs_rel_(ABSOLUTE_COORD)
  {
    r_.identity();
    ;
  };

  ~EstimatorAlgorithm()
  {
  }

  /* coodrinate change  */
  void coordinateUpdate(float desire_attitude_roll, float desire_attitude_pitch)
  {
    desire_attitude_roll_ = desire_attitude_roll;
    desire_attitude_pitch_ = desire_attitude_pitch;

    r_.from_euler(desire_attitude_roll_, desire_attitude_pitch_, 0);
  }

  void update(const Vector3f& gyro, const Vector3f& acc, const Vector3f& mag)
  {
    /* the sensor data in body frame */
    acc_b_ = acc;
    gyro_b_ = gyro;
    mag_b_ = mag;

    /* the sensor data in virtual frame */
    acc_v_ = r_ * acc_b_;
    gyro_v_ = r_ * gyro_b_;
    mag_v_ = r_ * mag_b_;

    estimation();
  }

  virtual void estimation(){};  // please implementation!

  static const uint8_t ABSOLUTE_COORD = 0;
  static const uint8_t RELATIVE_COORD = 1;
  static const uint8_t X = 0;
  static const uint8_t Y = 1;
  static const uint8_t Z = 2;

  Vector3f getAngles()
  {
    return rpy_;
  }
  Vector3f getVels()
  {
    return gyro_v_;
  }  // should be the virtual frame

  Vector3f getAccB()
  {
    return acc_b_;
  }
  Vector3f getGyroB()
  {
    return gyro_b_;
  }
  Vector3f getMagB()
  {
    return mag_b_;
  }

  Vector3f getAccV()
  {
    return acc_v_;
  }
  Vector3f getGyroV()
  {
    return gyro_v_;
  }
  Vector3f getMagV()
  {
    return mag_v_;
  }

protected:
  Vector3f acc_b_, acc_v_;
  Vector3f gyro_b_, gyro_v_;
  Vector3f mag_b_, mag_v_;

  Matrix3f r_;
  Vector3f rpy_;
  Quaternion q_;  // TODO => change to AP_Math

  float desire_attitude_roll_, desire_attitude_pitch_;
  uint8_t abs_rel_;
  uint8_t update_desire_attitude_;
};

#endif
