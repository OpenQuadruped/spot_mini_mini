#ifndef IMU_INCLUDE_GUARD_HPP
#define IMU_INCLUDE_GUARD_HPP
/// \file
/// \brief IMU Library.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU {
	private:
		Adafruit_BNO055 bno = Adafruit_BNO055(55);
	public:
	void Initialize();
	imu::Vector<3> GetEuler();
	imu::Vector<3> GetAcc();
	imu::Vector<3> GetGyro();

};
#endif