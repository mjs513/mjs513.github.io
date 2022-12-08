///////////////////////////////////////////////////////////////////////////////////////
// 
// Implementation of Madgwick's Gradient Descent AHRS Algorithm
///////////////////////////////////////////////////////////////////////////////////////

//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq	512.0f		// sample frequency in Hz
//#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions
#define betaDef   0.515f
volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
//float q[4];

//---------------------------------------------------------------------------------------------------
// Function declarations

//float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float g_x, float g_y, float g_z, float a_x, float a_y, float a_z) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from g_yroscope
  qDot1 = 0.5f * (-q1 * g_x - q2 * g_y - q3 * g_z);
  qDot2 = 0.5f * (q0 * g_x + q2 * g_z - q3 * g_y);
  qDot3 = 0.5f * (q0 * g_y - q1 * g_z + q3 * g_x);
  qDot4 = 0.5f * (q0 * g_z + q1 * g_y - q2 * g_x);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((a_x == 0.0f) && (a_y == 0.0f) && (a_z == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x *= recipNorm;
    a_y *= recipNorm;
    a_z *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * a_x + _4q0 * q1q1 - _2q1 * a_y;
    s1 = _4q1 * q3q3 - _2q3 * a_x + 4.0f * q0q0 * q1 - _2q0 * a_y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a_z;
    s2 = 4.0f * q0q0 * q2 + _2q0 * a_x + _4q2 * q3q3 - _2q3 * a_y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a_z;
    s3 = 4.0f * q1q1 * q3 - _2q1 * a_x + 4.0f * q2q2 * q3 - _2q2 * a_y;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (_dt);
  q1 += qDot2 * (_dt);
  q2 += qDot3 * (_dt);
  q3 += qDot4 * (_dt);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

}

// AHRS algorithm update

void MadgwickAHRSupdate(float g_x, float g_y, float g_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0m_x, _2q0m_y, _2q0m_z, _2q1m_x, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
	//float _2q0q2, _2q2q3; // not used
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float _8bx, _8bz;
	
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((m_x == 0.0f) && (m_y == 0.0f) && (m_z == 0.0f)) {
		MadgwickAHRSupdateIMU(g_x, g_y, g_z, a_x, a_y, a_z);
		return;
	}

	// Rate of change of quaternion from g_yroscope
	qDot1 = 0.5f * (-q1 * g_x - q2 * g_y - q3 * g_z);
	qDot2 = 0.5f * (q0 * g_x + q2 * g_z - q3 * g_y);
	qDot3 = 0.5f * (q0 * g_y - q1 * g_z + q3 * g_x);
	qDot4 = 0.5f * (q0 * g_z + q1 * g_y - q2 * g_x);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((a_x == 0.0f) && (a_y == 0.0f) && (a_z == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(a_x * a_x + a_y * a_y + a_z * a_z);
		a_x *= recipNorm;
		a_y *= recipNorm;
		a_z *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(m_x * m_x + m_y * m_y + m_z * m_z);
		m_x *= recipNorm;
		m_y *= recipNorm;
		m_z *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0m_x = 2.0f * q0 * m_x;
		_2q0m_y = 2.0f * q0 * m_y;
		_2q0m_z = 2.0f * q0 * m_z;
		_2q1m_x = 2.0f * q1 * m_x;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		//_2q0q2 = 2.0f * q0 * q2; // Not used
		//_2q2q3 = 2.0f * q2 * q3; // Not used
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = m_x * q0q0 - _2q0m_y * q3 + _2q0m_z * q2 + m_x * q1q1 + _2q1 * m_y * q2 + _2q1 * m_z * q3 - m_x * q2q2 - m_x * q3q3;
		hy = _2q0m_x * q3 + m_y * q0q0 - _2q0m_z * q1 + _2q1m_x * q2 - m_y * q1q1 + m_y * q2q2 + _2q2 * m_z * q3 - m_y * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0m_x * q2 + _2q0m_y * q1 + m_z * q0q0 + _2q1m_x * q3 - m_z * q1q1 + _2q2 * m_y * q3 - m_z * q2q2 + m_z * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		_8bx = 2.0f * _4bx;
		_8bz = 2.0f * _4bz;

		// Gradient decent algorithm corrective step
		s0= -_2q2*(2.0f*(q1q3 - q0q2) - a_x) + _2q1*(2.0f*(q0q1 + q2q3) - a_y) - _4bz*q2*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - m_x) + (-_4bx*q3+_4bz*q1)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - m_y) + _4bx*q2*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - m_z);
		s1= _2q3*(2.0f*(q1q3 - q0q2) - a_x) + _2q0*(2.0f*(q0q1 + q2q3) - a_y) - 4.0f*q1*(2.0f*(0.5 - q1q1 - q2q2) - a_z) + _4bz*q3*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - m_x) + (_4bx*q2+_4bz*q0)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - m_y) + (_4bx*q3-_8bz*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - m_z);
		s2= -_2q0*(2.0f*(q1q3 - q0q2) - a_x) + _2q3*(2.0f*(q0q1 + q2q3) - a_y) + (-4.0f*q2)*(2.0f*(0.5 - q1q1 - q2q2) - a_z) + (-_8bx*q2-_4bz*q0)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - m_x)+(_4bx*q1+_4bz*q3)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - m_y)+(_4bx*q0-_8bz*q2)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - m_z);
		s3= _2q1*(2.0f*(q1q3 - q0q2) - a_x) + _2q2*(2.0f*(q0q1 + q2q3) - a_y)+(-_8bx*q3+_4bz*q1)*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - m_x)+(-_4bx*q0+_4bz*q2)*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - m_y)+(_4bx*q1)*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - m_z);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
		
		// normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (_dt);
	q1 += qDot2 * (_dt);
	q2 += qDot3 * (_dt);
	q3 += qDot4 * (_dt);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

}

void getQ(float *quant1, float *quant2, float *quant3, float *quant4)
{
    *quant1 = q0;
    *quant2 = q1;
    *quant3 = q2;
    *quant4 = q3;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

//float invSqrt(float x) {
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5f3759df - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
//}

//====================================================================================================
// END OF CODE
//====================================================================================================
