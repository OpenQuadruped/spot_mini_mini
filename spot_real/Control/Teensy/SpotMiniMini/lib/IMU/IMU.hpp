#ifndef IMU_INCLUDE_GUARD_HPP
#define IMU_INCLUDE_GUARD_HPP
/// \file
/// \brief IMU Library.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

class IMU {
	private:
		Adafruit_BNO055 bno = Adafruit_BNO055(55);
		bool ok = true;
	public:
	void displaySensorDetails(void);
	void displaySensorStatus(void);
	void displayCalStatus(void);
	void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
	void Initialize();
	bool available();
	imu::Vector<3> GetEuler();
	imu::Quaternion GetQuat();
	imu::Vector<3> GetAcc();
	imu::Vector<3> GetGyro();

};
#endif