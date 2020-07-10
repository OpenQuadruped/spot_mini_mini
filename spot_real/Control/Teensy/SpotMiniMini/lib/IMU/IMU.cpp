#include "IMU.hpp"
#include <Arduino.h>

void IMU::Initialize()
{
	if(!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial1.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
	}

	delay(1000);

	bno.setExtCrystalUse(true);
}

imu::Vector<3> IMU::GetEuler()
{
	return bno.getVector(Adafruit_BNO055::VECTOR_EULER);
}

imu::Vector<3> IMU::GetAcc()
{
	return bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}

imu::Vector<3> IMU::GetGyro()
{
	return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}


