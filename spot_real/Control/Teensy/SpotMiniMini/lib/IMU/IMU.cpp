#include "IMU.hpp"
#include <Arduino.h>

void IMU::Initialize()
{
	if(!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}

	delay(1000);

	bno.setExtCrystalUse(true);
}

imu::Vector<3> GetEuler()
{
	eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
}

imu::Vector<3> GetAcc()
{
	return bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}

imu::Vector<3> GetGyro()
{
	return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}


