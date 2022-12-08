
//======================================
// helper functions
// Inverse square root function for MARG Filter (Madgwick and Mahoney)
float invSqrt(float x) {
  int instability_fix = 1;
  if (instability_fix == 0)
  {
    union {
      float f;
      int32_t i;
    } y;

    y.f = x;
    y.i = 0x5f375a86 - (y.i >> 1);
    y.f = y.f * ( 1.5f - ( x * 0.5f * y.f * y.f ) );
    return y.f;
  }
  else if (instability_fix == 1)
  {
    /* close-to-optimal  method with low cost from
    http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
    uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
    float tmp = *(float*)&i;
    return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  }
  else
  {
    /* optimal but expensive method: */
    return 1.0f / sqrt(x);
  }
}


/**
 * Converts a 3 elements array arr of angles expressed in radians into degrees
*/
void arr3_rad_to_deg(float * arr) {
  arr[0] *= 180 / PI;
  arr[1] *= 180 / PI;
  arr[2] *= 180 / PI;
}

/**
 * Converts a 3 elements array arr of angles expressed in degrees into radians
*/
void arr3_deg_to_rad(float * arr) {
  arr[0] /= 180 / PI;
  arr[1] /= 180 / PI;
  arr[2] /= 180 / PI;
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
 * the Earth North and the IMU X a_xis (yaw), the Earth ground plane and the IMU X a_xis (pitch)
 * and the Earth ground plane and the IMU Y a_xis.
 *
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 *
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in radians
*/
void getYawPitchRollRad(float * ypr) {
  float q[4];
  getQ(&q[0], &q[1], &q[2], &q[3]);
  //float grx, gry, grz; // estimated gravity direction

  //grx = 2 * (q[1]*q[3] - q[0]*q[2]);
  //gry = 2 * (q[0]*q[1] + q[2]*q[3]);
  //grz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  //calculate direction of gravity
  grx = 2 * (-q[1] * q[3] + q[0] * q[2]);
  gry = 2 * (-q[0] * q[1] - q[2] * q[3]);
  grz = -q[0] * q[0] + q[1] * q[1] + q[2] * q[2] - q[3] * q[3];

  ypr[0] = -atan2f(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = -atanf(grx / sqrt(gry * gry + grz * grz));
  ypr[2] = atanf(gry / sqrt(grx * grx + grz * grz));
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in degrees between
 * the Earth North and the IMU X a_xis (yaw), the Earth ground plane and the IMU X a_xis (pitch)
 * and the Earth ground plane and the IMU Y a_xis.
 *
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 *
 * @param ypr three floats arra_y which will be populated by Yaw, Pitch and Roll angles in degrees
*/
void getYawPitchRoll(float * ypr) {
  getYawPitchRollRad(ypr);
  arr3_rad_to_deg(ypr);
}

/**
 * Returns the Euler angles in radians defined in the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for
 * inertial and intertial/magnetic sensor arra_ys" Chapter 2 Quaternion representation
 *
 * @param angles three floats arra_y which will be populated by the Euler angles in radians
*/
void getEulerRad(float * angles) {
  float a12, a22, a31, a32, a33;
  float q[4];
  getQ(&q[0], &q[1], &q[2], &q[3]);

  //angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  //angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  //angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi

  a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
  a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
  a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
  a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  angles[1] = -asinf(a32);
  angles[2]  = atan2f(a31, a33);
  angles[0]   = atan2f(a12, a22);
}

/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence (NED).  Conversion
 * based on MATLAB quat2angle.m for an ZXY rotation sequence.
 * Angles are lie between 0-360 degrees in radians.
 *
 * @param angles three floats arra_y which will be populated by the Euler angles in degrees
*/
void getEuler360(float * angles) {
  getEulerRad(angles);
  arr3_deg_to_rad(angles);
}

/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for
 * inertial and intertial/magnetic sensor arra_ys" Chapter 2 Quaternion representation
 *
 * @param angles three floats arra_y which will be populated by the Euler angles in degrees
*/
void getEuler(float * angles) {
  getEulerRad(angles);
  arr3_rad_to_deg(angles);
}

/**
 * Get heading from magnetometer if LSM303 not available
 * code extracted from rob42/FreeIMU-20121106_1323 Github library
 * (https://github.com/rob42/FreeIMU-20121106_1323.git)
 * which is based on
 *
**/
float calcMagHeading(float q0, float q1, float q2, float q3, float bx, float by, float bz) {
  float Head_X, Head_Y;
  float cos_roll, sin_roll, cos_pitch, sin_pitch;
  float grx, gry, grz; // estimated gravity direction
  float ypr[3];

  //grx = 2 * (q1*q3 - q0*q2);
  //gry = 2 * (q0*q1 + q2*q3);
  //grz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  grx = 2 * (-q1 * q3 + q0 * q2);
  gry = 2 * (-q0 * q1 - q2 * q3);
  grz = -q0 * q0 + q1 * q1 + q2 * q2 - q3 * q3;

  ypr[0] = -atan2f(2 * q1 * q2 - 2 * q0 * q3, 2 * q0 * q0 + 2 * q1 * q1 - 1);
  ypr[1] = -atanf(grx / sqrt(gry * gry + grz * grz));
  ypr[2] = atanf(gry / sqrt(grx * grx + grz * grz));

  cos_roll = cosf(-ypr[2]);
  sin_roll = sinf(-ypr[2]);
  cos_pitch = cosf(ypr[1]);
  sin_pitch = sinf(ypr[1]);

  //Example calc
  //Xh = bx * cos(theta) + by * sin(phi) * sin(theta) + bz * cos(phi) * sin(theta)
  //Yh = by * cos(phi) - bz * sin(phi)
  //return wrap((atan2(-Yh, Xh) + variation))
  // Tilt compensated Magnetic field X component:
  Head_X = bx * cos_pitch + by * sin_roll * sin_pitch + bz * cos_roll * sin_pitch;
  // Tilt compensated Magnetic field Y component:
  Head_Y = by * cos_roll - bz * sin_roll;
  // Magnetic Heading
  return (atan2f(-Head_Y, Head_X) * 180. / PI) + MAG_DEC;
}

