// #ifndef IMU2_INCLUDE_GUARD_HPP
// #define IMU2_INCLUDE_GUARD_HPP
// /// \file
// /// \brief IMU Library.
// #include <Arduino.h>
// #include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_LSM9DS1.h>
// #include <Adafruit_Sensor.h>  // not used in this demo but required!

// #define LSM9DS1_SCK A5
// #define LSM9DS1_MISO 12
// #define LSM9DS1_MOSI A4
// #define LSM9DS1_XGCS 6
// #define LSM9DS1_MCS 5

// class IMU2 {
// 	private:
// 		bool ok = true;
// 	public:
// 	Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
// 	void Initialize();
// 	bool available();

// };
// #endif