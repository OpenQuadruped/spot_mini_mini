// #include "IMU2.hpp"



// void IMU2::Initialize()
// {

// 	if (!lsm.begin())
// 	{
// 	Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
// 	ok = false;
// 	} else
// 	{
// 		// 1.) Set the accelerometer range
// 		lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
// 		//lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
// 		//lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
// 		//lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

// 		// 2.) Set the magnetometer sensitivity
// 		lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
// 		//lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
// 		//lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
// 		//lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

// 		// 3.) Setup the gyroscope
// 		lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
// 		//lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
// 		//lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
// 	}
	
// }

// bool IMU2::available()
// {
// 	return ok;
// }


