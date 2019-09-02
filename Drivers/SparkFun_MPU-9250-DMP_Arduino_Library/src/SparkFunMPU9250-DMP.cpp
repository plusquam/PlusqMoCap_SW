//Modified by Damian Świerk
/******************************************************************************
SparkFunMPU9250-DMP.cpp - MPU-9250 Digital Motion Processor Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

This library implements motion processing functions of Invensense's MPU-9250.
It is based on their Emedded MotionDriver 6.12 library.
	https://www.invensense.com/developers/software-downloads/

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
******************************************************************************/
#include "SparkFunMPU9250-DMP.h"
#include "MPU9250_RegisterMap.h"

extern "C" {
#include "util/inv_mpu.h"
}
#include <math.h>
#include <string.h>

static unsigned char mpu9250_orientation;
static unsigned char tap_count;
static unsigned char tap_direction;
static bool _tap_available;
static void orient_cb(unsigned char orient);
static void tap_cb(unsigned char direction, unsigned char count);

// My defines
#define PI 3.14159265f
#define constrain(x, a, b) ( (x) < (a) ? (a) : (((x) > (b)) ? (b) : (x)) )


MPU9250_DMP::MPU9250_DMP()
{
	_mSense = 6.665f; // Constant - 4915 / 32760
	_aSense = 0.0f;   // Updated after accel FSR is set
	_gSense = 0.0f;   // Updated after gyro FSR is set
}

inv_error_t MPU9250_DMP::begin(void)
{
	inv_error_t result;
    struct int_param_s int_param;
	
	result = mpu_init(&int_param);
	
	if (result)
		return result;
	
//	mpu_set_bypass(1); // Place all slaves (including compass) on primary bus
	
	setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	
	_gSense = getGyroSens();
	_aSense = getAccelSens();
	
	return result;
}

inv_error_t MPU9250_DMP::enableInterrupt(unsigned char enable)
{
	return set_int_enable(enable);
}

inv_error_t MPU9250_DMP::setIntLevel(unsigned char active_low)
{
	return mpu_set_int_level(active_low);
}

inv_error_t MPU9250_DMP::setIntLatched(unsigned char enable)
{
	return mpu_set_int_latched(enable);
}

short MPU9250_DMP::getIntStatus(void)
{
	short status;
	if (mpu_get_int_status(&status) == INV_SUCCESS)
	{
		return status;
	}
	return 0;
}

// Accelerometer Low-Power Mode. Rate options:
// 1.25 (1), 2.5 (2), 5, 10, 20, 40, 
// 80, 160, 320, or 640 Hz
// Disables compass and gyro
inv_error_t MPU9250_DMP::lowPowerAccel(unsigned short rate)
{
	return mpu_lp_accel_mode(rate);
}

inv_error_t MPU9250_DMP::setGyroFSR(unsigned short fsr)
{
	inv_error_t err;
	err = mpu_set_gyro_fsr(fsr);
	if (err == INV_SUCCESS)
	{
		_gSense = getGyroSens();
	}
	return err;
}

inv_error_t MPU9250_DMP::setAccelFSR(unsigned char fsr)
{
	inv_error_t err;
	err = mpu_set_accel_fsr(fsr);
	if (err == INV_SUCCESS)
	{
		_aSense = getAccelSens();
	}
	return err;
}

unsigned short MPU9250_DMP::getGyroFSR(void)
{
	unsigned short tmp;
	if (mpu_get_gyro_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

unsigned char MPU9250_DMP::getAccelFSR(void)
{
	unsigned char tmp;
	if (mpu_get_accel_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;	
}

unsigned short MPU9250_DMP::getMagFSR(void)
{
	unsigned short tmp;
	if (mpu_get_compass_fsr(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t MPU9250_DMP::setLPF(unsigned short lpf)
{
	return mpu_set_lpf(lpf);
}

unsigned short MPU9250_DMP::getLPF(void)
{
	unsigned short tmp;
	if (mpu_get_lpf(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t MPU9250_DMP::setSampleRate(unsigned short rate)
{
    return mpu_set_sample_rate(rate);
}

unsigned short MPU9250_DMP::getSampleRate(void)
{
	unsigned short tmp;
	if (mpu_get_sample_rate(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	return 0;
}

inv_error_t MPU9250_DMP::setCompassSampleRate(unsigned short rate)
{
	return mpu_set_compass_sample_rate(rate);
}

unsigned short MPU9250_DMP::getCompassSampleRate(void)
{
	unsigned short tmp;
	if (mpu_get_compass_sample_rate(&tmp) == INV_SUCCESS)
	{
		return tmp;
	}
	
	return 0;
}

float MPU9250_DMP::getGyroSens(void)
{
	float sens;
	if (mpu_get_gyro_sens(&sens) == INV_SUCCESS)
	{
		return sens;
	}
	return 0;
}
	
short MPU9250_DMP::getAccelSens(void)
{
	short sens;
	if (mpu_get_accel_sens((unsigned short*)&sens) == INV_SUCCESS)
	{
		return sens;
	}
	return 0;
}

float MPU9250_DMP::getMagSens(void)
{
	return 0.15; // Static, 4915/32760
}

unsigned char MPU9250_DMP::getFifoConfig(void)
{
	unsigned char sensors;
	if (mpu_get_fifo_config(&sensors) == INV_SUCCESS)
	{
		return sensors;
	}
	return 0;
}

inv_error_t MPU9250_DMP::configureFifo(unsigned char sensors)
{
	return mpu_configure_fifo(sensors);
}

inv_error_t MPU9250_DMP::resetFifo(void)
{
	return mpu_reset_fifo();
}

unsigned short MPU9250_DMP::fifoAvailable(void)
{
	unsigned char data[2];
	unsigned short fifo_count;

    if (mpu_read_multi_reg(MPU9250_FIFO_COUNTH, data, 2))
        return 0;
    fifo_count = (data[0] << 8) | data[1];
	
	return fifo_count;
}

inv_error_t MPU9250_DMP::updateFifo(void)
{
	short gyro[3], accel[3];
	unsigned long timestamp;
	unsigned char sensors, more;
	
	if (mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more) != INV_SUCCESS)
		return INV_ERROR;
	
	if (sensors & INV_XYZ_ACCEL)
	{
		ax = accel[X_AXIS];
		ay = accel[Y_AXIS];
		az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		gz = gyro[Z_AXIS];
	
	time = timestamp;
	
	return INV_SUCCESS;
}

inv_error_t MPU9250_DMP::setSensors(unsigned char sensors)
{
	return mpu_set_sensors(sensors);
}

bool MPU9250_DMP::dataReady()
{
	unsigned char intStatusReg;
	
	if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS)
	{
		return (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
	}
	return false;
}

inv_error_t MPU9250_DMP::update(unsigned char sensors)
{
	inv_error_t aErr = INV_SUCCESS;
	inv_error_t gErr = INV_SUCCESS;
	inv_error_t mErr = INV_SUCCESS;
	inv_error_t tErr = INV_SUCCESS;
	
	if (sensors & UPDATE_ACCEL)
		aErr = updateAccel();
	if (sensors & UPDATE_GYRO)
		gErr = updateGyro();
	if (sensors & UPDATE_COMPASS)
		mErr = updateCompass();
	if (sensors & UPDATE_TEMP)
		tErr = updateTemperature();

	return aErr | gErr | mErr | tErr;
}

int MPU9250_DMP::updateAccel(void)
{
	short data[3];
	
	if (mpu_get_accel_reg(data, &time))
	{
		return INV_ERROR;		
	}
	ax = data[X_AXIS];
	ay = data[Y_AXIS];
	az = data[Z_AXIS];
	return INV_SUCCESS;
}

int MPU9250_DMP::updateGyro(void)
{
	short data[3];
	
	if (mpu_get_gyro_reg(data, &time))
	{
		return INV_ERROR;		
	}
	gx = data[X_AXIS];
	gy = data[Y_AXIS];
	gz = data[Z_AXIS];
	return INV_SUCCESS;
}

int MPU9250_DMP::updateCompass(void)
{
	short data[3];
	
	if (mpu_get_compass_reg(data, &time))
	{
		return INV_ERROR;		
	}
	mx = data[X_AXIS];
	my = data[Y_AXIS];
	mz = data[Z_AXIS];
	return INV_SUCCESS;
}

inv_error_t MPU9250_DMP::updateTemperature(void)
{
	return mpu_get_temperature(&temperature, &time);
}

int MPU9250_DMP::selfTest(unsigned char debug)
{
	long gyro[3], accel[3];
	return mpu_run_self_test(gyro, accel);
}

inv_error_t MPU9250_DMP::dmpBegin(unsigned short features, unsigned short fifoRate)
{
	unsigned short feat = features;
	unsigned short rate = fifoRate;

	if (dmpLoad() != INV_SUCCESS)
		return INV_ERROR;
	
	// 3-axis and 6-axis LP quat are mutually exclusive.
	// If both are selected, default to 3-axis
	if (feat & DMP_FEATURE_LP_QUAT)
	{
		feat &= ~(DMP_FEATURE_6X_LP_QUAT);
		dmp_enable_lp_quat(1);
	}
	else if (feat & DMP_FEATURE_6X_LP_QUAT)
		dmp_enable_6x_lp_quat(1);
	
	if (feat & DMP_FEATURE_GYRO_CAL)
		dmp_enable_gyro_cal(1);
	
	if (dmpEnableFeatures(feat) != INV_SUCCESS)
		return INV_ERROR;
	
	rate = constrain(rate, 1, 200);
	if (dmpSetFifoRate(rate) != INV_SUCCESS)
		return INV_ERROR;
	
	return mpu_set_dmp_state(1);
}

inv_error_t MPU9250_DMP::dmpLoad(void)
{
	return dmp_load_motion_driver_firmware();
}

unsigned short MPU9250_DMP::dmpGetFifoRate(void)
{
	unsigned short rate;
	if (dmp_get_fifo_rate(&rate) == INV_SUCCESS)
		return rate;
	
	return 0;
}

inv_error_t MPU9250_DMP::dmpSetFifoRate(unsigned short rate)
{
	if (rate > MAX_DMP_SAMPLE_RATE) rate = MAX_DMP_SAMPLE_RATE;
	return dmp_set_fifo_rate(rate);
}

inv_error_t MPU9250_DMP::dmpUpdateFifo(void)
{
	short gyro[3];
	short accel[3];
	long quat[4];
	unsigned long timestamp;
	short sensors;
	unsigned char more;
	
	if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)
		   != INV_SUCCESS)
    {
	   return INV_ERROR;
    }
	
	if (sensors & INV_XYZ_ACCEL)
	{
		ax = accel[X_AXIS];
		ay = accel[Y_AXIS];
		az = accel[Z_AXIS];
	}
	if (sensors & INV_X_GYRO)
		gx = gyro[X_AXIS];
	if (sensors & INV_Y_GYRO)
		gy = gyro[Y_AXIS];
	if (sensors & INV_Z_GYRO)
		gz = gyro[Z_AXIS];
	if (sensors & INV_WXYZ_QUAT)
	{
		qw = quat[0];
		qx = quat[1];
		qy = quat[2];
		qz = quat[3];
	}
	
	time = timestamp;
	
	return INV_SUCCESS;
}

inv_error_t MPU9250_DMP::dmpEnableFeatures(unsigned short mask)
{
	unsigned short enMask = 0;
	enMask |= mask;
	// Combat known issue where fifo sample rate is incorrect
	// unless tap is enabled in the DMP.
	enMask |= DMP_FEATURE_TAP; 
	return dmp_enable_feature(enMask);
}

unsigned short MPU9250_DMP::dmpGetEnabledFeatures(void)
{
	unsigned short mask;
	if (dmp_get_enabled_features(&mask) == INV_SUCCESS)
		return mask;
	return 0;
}

inv_error_t MPU9250_DMP::dmpSetTap(
        unsigned short xThresh, unsigned short yThresh, unsigned short zThresh,
        unsigned char taps, unsigned short tapTime, unsigned short tapMulti)
{
	unsigned char axes = 0;
	if (xThresh > 0)
	{
		axes |= TAP_X;
		xThresh = constrain(xThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<X_AXIS, xThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (yThresh > 0)
	{
		axes |= TAP_Y;
		yThresh = constrain(yThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<Y_AXIS, yThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (zThresh > 0)
	{
		axes |= TAP_Z;
		zThresh = constrain(zThresh, 1, 1600);
		if (dmp_set_tap_thresh(1<<Z_AXIS, zThresh) != INV_SUCCESS)
			return INV_ERROR;
	}
	if (dmp_set_tap_axes(axes) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_count(taps) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_time(tapTime) != INV_SUCCESS)
		return INV_ERROR;
	if (dmp_set_tap_time_multi(tapMulti) != INV_SUCCESS)
		return INV_ERROR;
	
    dmp_register_tap_cb(tap_cb);
	
	return INV_SUCCESS;
}

unsigned char MPU9250_DMP::getTapDir(void)
{
	_tap_available = false;
	return tap_direction;
}

unsigned char MPU9250_DMP::getTapCount(void)
{
	_tap_available = false;
	return tap_count;
}

bool MPU9250_DMP::tapAvailable(void)
{
	return _tap_available;
}

inv_error_t MPU9250_DMP::dmpSetOrientation(const signed char * orientationMatrix)
{
	unsigned short scalar;
	scalar = orientation_row_2_scale(orientationMatrix);
	scalar |= orientation_row_2_scale(orientationMatrix + 3) << 3;
	scalar |= orientation_row_2_scale(orientationMatrix + 6) << 6;
	
    dmp_register_android_orient_cb(orient_cb);
	
	return dmp_set_orientation(scalar);
}

unsigned char MPU9250_DMP::dmpGetOrientation(void)
{
	return mpu9250_orientation;
}

inv_error_t MPU9250_DMP::dmpEnable3Quat(void)
{
	unsigned short dmpFeatures;
	
	// 3-axis and 6-axis quat are mutually exclusive
	dmpFeatures = dmpGetEnabledFeatures();
	dmpFeatures &= ~(DMP_FEATURE_6X_LP_QUAT);
	dmpFeatures |= DMP_FEATURE_LP_QUAT;
	
	if (dmpEnableFeatures(dmpFeatures) != INV_SUCCESS)
		return INV_ERROR;
	
	return dmp_enable_lp_quat(1);
}
	
unsigned long MPU9250_DMP::dmpGetPedometerSteps(void)
{
	unsigned long steps;
	if (dmp_get_pedometer_step_count(&steps) == INV_SUCCESS)
	{
		return steps;
	}
	return 0;
}

inv_error_t MPU9250_DMP::dmpSetPedometerSteps(unsigned long steps)
{
	return dmp_set_pedometer_step_count(steps);
}

unsigned long MPU9250_DMP::dmpGetPedometerTime(void)
{
	unsigned long walkTime;
	if (dmp_get_pedometer_walk_time(&walkTime) == INV_SUCCESS)
	{
		return walkTime;
	}
	return 0;
}

inv_error_t MPU9250_DMP::dmpSetPedometerTime(unsigned long time)
{
	return dmp_set_pedometer_walk_time(time);
}

float MPU9250_DMP::calcAccel(int axis)
{
	return (float) axis / (float) _aSense;
}

float MPU9250_DMP::calcGyro(int axis)
{
	return (float) axis / (float) _gSense;
}

float MPU9250_DMP::calcMag(int axis)
{
	return (float) axis / (float) _mSense;
}

float MPU9250_DMP::calcQuat(long axis)
{
	return qToFloat(axis, 30);
}
	
float MPU9250_DMP::qToFloat(long number, unsigned char q)
{
	unsigned long mask = 0;
	for (int i=0; i<q; i++)
	{
		mask |= (1<<i);
	}
	return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

void MPU9250_DMP::computeEulerAngles(bool degrees)
{
    float dqw = qToFloat(qw, 30);
    float dqx = qToFloat(qx, 30);
    float dqy = qToFloat(qy, 30);
    float dqz = qToFloat(qz, 30);
    
    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;
  
	// Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
  
    pitch = asin(t2) * 2;
    roll = atan2(t3, t4);
    yaw = atan2(t1, t0);
	
	if (degrees)
	{
		pitch *= (180.0 / PI);
		roll *= (180.0 / PI);
		yaw *= (180.0 / PI);
		if (pitch < 0) pitch = 360.0 + pitch;
		if (roll < 0) roll = 360.0 + roll;
		if (yaw < 0) yaw = 360.0 + yaw;	
	}
}

float MPU9250_DMP::computeCompassHeading(void)
{
	if (my == 0)
		heading = (mx < 0) ? PI : 0;
	else
		heading = atan2(mx, my);
	
	if (heading > PI) heading -= (2 * PI);
	else if (heading < -PI) heading += (2 * PI);
	else if (heading < 0) heading += 2 * PI;
	
	heading*= 180.0 / PI;
	
	return heading;
}

unsigned short MPU9250_DMP::orientation_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}
		
static void tap_cb(unsigned char direction, unsigned char count)
{
	_tap_available = true;
	tap_count = count;
	tap_direction = direction;
}

static void orient_cb(unsigned char orient)
{
	mpu9250_orientation = orient;
}

#ifndef AKM_DATA_READY
#define AKM_DATA_READY      (0x01)
#endif
#ifndef AKM_DATA_OVERRUN
#define AKM_DATA_OVERRUN    (0x02)
#endif
#ifndef AKM_OVERFLOW
#define AKM_OVERFLOW        (0x80)
#endif
#ifndef AKM_DATA_ERROR
#define AKM_DATA_ERROR      (0x40)
#endif

bool MPU9250_DMP::magDataReady()
{
	bool isReady = false;
	unsigned char data;
	if(mpu_read_reg(MPU9250_EXT_SENS_DATA_00, &data) == INV_SUCCESS)
	{
		if(data & AKM_DATA_READY)
			isReady = true;
	}
	return isReady;
}

inv_error_t MPU9250_DMP::allDataUpdate(unsigned char *buffer, unsigned char offset, unsigned char fifoEnabled)
{
	if(!fifoEnabled)
	{
		// Data is stored in sensor registers //

		constexpr unsigned char DATA_SIZE = (sizeof(ax) * 3) + sizeof(short) + (sizeof(gx) * 3) + 8;
		// Data frame: 3acc + temp + 3gyro + 8B mag data
		unsigned char data[DATA_SIZE];

		if(mpu_read_multi_reg(MPU9250_ACCEL_XOUT_H, data, DATA_SIZE))
			return INV_ERROR;

		if(buffer == NULL) {
		// Buffer not given -> data stored in class members
			// Accel
			ax = (data[0] << 8) | data[1];
			ay = (data[2] << 8) | data[3];
			az = (data[4] << 8) | data[5];

			// Temperature // don't care now
		//    temperature = TODO

			//Gyro
			gx = (data[8] << 8) | data[9];
			gy = (data[10] << 8) | data[11];
			gz = (data[12] << 8) | data[13];

			//Mag
			if (!(data[14] & AKM_DATA_READY)) // || (data[14] & AKM_DATA_OVERRUN))
				return -2;
		//    if (data[21] & AKM_OVERFLOW)
		//        return -3;

			short mag_sens_adj[3];
			getMagSensAdj(mag_sens_adj, mag_sens_adj + 1, mag_sens_adj + 2);

			mx = (data[16] << 8) | data[15];
			my = (data[18] << 8) | data[17];
			mz = (data[20] << 8) | data[19];

			mx = ((long)mx * mag_sens_adj[0]) >> 8;
			my = ((long)my * mag_sens_adj[1]) >> 8;
			mz = ((long)mz * mag_sens_adj[2]) >> 8;
		}
		else
		{
			// Buffer given -> data stored in input buffer
			if (!(data[14] & AKM_DATA_READY)) // || (data[14] & AKM_DATA_OVERRUN))
				return -2;

			unsigned char *buffer_with_offset = buffer + offset;

			// Acc
			memcpy(buffer_with_offset, data, sizeof(ax) * 3);

			//Gyro
			memcpy(buffer_with_offset + 6, data + 8, sizeof(gx) * 3);

			// Mag
			short temp_mx = (data[16] << 8) | data[15];
			short temp_my = (data[18] << 8) | data[17];
			short temp_mz = (data[20] << 8) | data[19];

			short mag_sens_adj[3];
			getMagSensAdj(mag_sens_adj, mag_sens_adj + 1, mag_sens_adj + 2);

			temp_mx = ((long)temp_mx * mag_sens_adj[0]) >> 8;
			temp_my = ((long)temp_my * mag_sens_adj[1]) >> 8;
			temp_mz = ((long)temp_mz * mag_sens_adj[2]) >> 8;

			buffer_with_offset[12] = (unsigned char)(temp_mx >> 8);
			buffer_with_offset[13] = (unsigned char)temp_mx;
			buffer_with_offset[14] = (unsigned char)(temp_mx >> 8);
			buffer_with_offset[15] = (unsigned char)temp_mx;
			buffer_with_offset[16] = (unsigned char)(temp_mx >> 8);
			buffer_with_offset[17] = (unsigned char)temp_mx;
		}
	}
	else
	{
		// Data is stored in FIFO //

		constexpr unsigned char DATA_SIZE = (sizeof(ax) * 3) + (sizeof(gx) * 3) + 8;
		// Data frame: 3acc + 3gyro + 8B mag data
		unsigned char data[DATA_SIZE];

	    if (mpu_read_multi_reg(MPU9250_FIFO_R_W, data, DATA_SIZE))
	        return -1;

	    if(buffer == NULL) {
		// Buffer not given -> data stored in class members
			// Accel
			ax = (data[0] << 8) | data[1];
			ay = (data[2] << 8) | data[3];
			az = (data[4] << 8) | data[5];

			// Temperature // don't care now
		//    temperature = TODO

			//Gyro
			gx = (data[6] << 8) | data[7];
			gy = (data[8] << 8) | data[9];
			gz = (data[10] << 8) | data[11];

			//Mag
			if (!(data[12] & AKM_DATA_READY)) // || (data[12] & AKM_DATA_OVERRUN))
				return -2;
		//    if (data[19] & AKM_OVERFLOW)
		//        return -3;

			short mag_sens_adj[3];
			getMagSensAdj(mag_sens_adj, mag_sens_adj + 1, mag_sens_adj + 2);

			mx = (data[14] << 8) | data[13];
			my = (data[16] << 8) | data[15];
			mz = (data[18] << 8) | data[17];

			mx = ((long)mx * mag_sens_adj[0]) >> 8;
			my = ((long)my * mag_sens_adj[1]) >> 8;
			mz = ((long)mz * mag_sens_adj[2]) >> 8;
		}
		else
		{
			// Buffer given -> data stored in input buffer
			if (!(data[12] & AKM_DATA_READY))
				return -2;

			unsigned char *buffer_with_offset = buffer + offset;

			// Acc
			memcpy(buffer_with_offset, data, sizeof(ax) * 3);

			//Gyro
			memcpy(buffer_with_offset + 6, data + 6, sizeof(gx) * 3);

			// Mag
			short temp_mx = (data[14] << 8) | data[13];
			short temp_my = (data[16] << 8) | data[15];
			short temp_mz = (data[18] << 8) | data[17];

			short mag_sens_adj[3];
			getMagSensAdj(mag_sens_adj, mag_sens_adj + 1, mag_sens_adj + 2);

			temp_mx = ((long)temp_mx * mag_sens_adj[0]) >> 8;
			temp_my = ((long)temp_my * mag_sens_adj[1]) >> 8;
			temp_mz = ((long)temp_mz * mag_sens_adj[2]) >> 8;

			buffer_with_offset[12] = (unsigned char)(temp_mx >> 8);
			buffer_with_offset[13] = (unsigned char)temp_mx;
			buffer_with_offset[14] = (unsigned char)(temp_mx >> 8);
			buffer_with_offset[15] = (unsigned char)temp_mx;
			buffer_with_offset[16] = (unsigned char)(temp_mx >> 8);
			buffer_with_offset[17] = (unsigned char)temp_mx;
		}
	}

	return INV_SUCCESS;
}

inv_error_t MPU9250_DMP::resetDevice(void)
{
	return mpu_reset();
}
