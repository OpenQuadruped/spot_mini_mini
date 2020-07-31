#include "IMU.hpp"
#include <Arduino.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void IMU::displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void IMU::displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void IMU::displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void IMU::displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void IMU::Initialize()
{
	if(!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!\n");
		ok = false;
	} else
	{
		int eeAddress = 0;
	    long bnoID;
	    bool foundCalib = false;

	    EEPROM.get(eeAddress, bnoID);

	    adafruit_bno055_offsets_t calibrationData;
	    sensor_t sensor;

	    /*
	    *  Look for the sensor's unique ID at the beginning oF EEPROM.
	    *  This isn't foolproof, but it's better than nothing.
	    */
	    bno.getSensor(&sensor);
	    if (bnoID != sensor.sensor_id)
	    {
	        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
	        delay(500);
	    }
	    else
	    {
	        Serial.println("\nFound Calibration for this sensor in EEPROM.");
	        eeAddress += sizeof(long);
	        EEPROM.get(eeAddress, calibrationData);

	        displaySensorOffsets(calibrationData);

	        Serial.println("\n\nRestoring Calibration data to the BNO055...");
	        bno.setSensorOffsets(calibrationData);

	        Serial.println("\n\nCalibration data loaded into BNO055");
	        foundCalib = true;
	    }

	    delay(1000);

	    /* Display some basic information on this sensor */
	    displaySensorDetails();

	    /* Optional: Display current status */
	    displaySensorStatus();

	   /* Crystal must be configured AFTER loading calibration data into BNO055. */
	    bno.setExtCrystalUse(true);

	    sensors_event_t event;
	    bno.getEvent(&event);
	    /* always recal the mag as It goes out of calibration very often */
	    if (foundCalib){
	    	// NOTE: UNCOMMENT IF PLANNING TO USE MAGNEMOMETER
	        // Serial.println("Move sensor slightly to calibrate magnetometers");
	        // while (!bno.isFullyCalibrated())
	        // {
	        //     bno.getEvent(&event);
	        //     delay(BNO055_SAMPLERATE_DELAY_MS);
	        // }
	        Serial.println("Loading Calibration...");
	    }
	    else
	    {
	        Serial.println("Please Calibrate Sensor: ");
	        while (!bno.isFullyCalibrated())
	        {
	            bno.getEvent(&event);

	            Serial.print("X: ");
	            Serial.print(event.orientation.x, 4);
	            Serial.print("\tY: ");
	            Serial.print(event.orientation.y, 4);
	            Serial.print("\tZ: ");
	            Serial.print(event.orientation.z, 4);

	            /* Optional: Display calibration status */
	            displayCalStatus();

	            /* New line for the next sample */
	            Serial.println("");

	            /* Wait the specified delay before requesting new data */
	            delay(BNO055_SAMPLERATE_DELAY_MS);
	        }
	    }

	    Serial.println("\nFully calibrated!");
	    Serial.println("--------------------------------");
	    Serial.println("Calibration Results: ");
	    adafruit_bno055_offsets_t newCalib;
	    bno.getSensorOffsets(newCalib);
	    displaySensorOffsets(newCalib);

	    Serial.println("\n\nStoring calibration data to EEPROM...");

	    eeAddress = 0;
	    bno.getSensor(&sensor);
	    bnoID = sensor.sensor_id;

	    EEPROM.put(eeAddress, bnoID);

	    eeAddress += sizeof(long);
	    EEPROM.put(eeAddress, newCalib);
	    Serial.println("Data stored to EEPROM.");

	    Serial.println("\n--------------------------------\n");

	    // REMAP AXIS. IMPORTANT
	    /** Remap settings **/
		// typedef enum {
		// REMAP_CONFIG_P0 = 0x21,
		// REMAP_CONFIG_P1 = 0x24, // default
		// REMAP_CONFIG_P2 = 0x24,
		// REMAP_CONFIG_P3 = 0x21,
		// REMAP_CONFIG_P4 = 0x24,
		// REMAP_CONFIG_P5 = 0x21,
		// REMAP_CONFIG_P6 = 0x21,
		// REMAP_CONFIG_P7 = 0x24
		// } adafruit_bno055_axis_remap_config_t;

		// * Remap Signs *
		// typedef enum {
		// REMAP_SIGN_P0 = 0x04,
		// REMAP_SIGN_P1 = 0x00, // default
		// REMAP_SIGN_P2 = 0x06,
		// REMAP_SIGN_P3 = 0x02,
		// REMAP_SIGN_P4 = 0x03,
		// REMAP_SIGN_P5 = 0x01,
		// REMAP_SIGN_P6 = 0x07,
		// REMAP_SIGN_P7 = 0x05
		// } adafruit_bno055_axis_remap_sign_t;
		Adafruit_BNO055::adafruit_bno055_axis_remap_config_t r;
		// NOTE: with this we just swap roll and pitch from eul.x/z
		r = Adafruit_BNO055::REMAP_CONFIG_P7;
	    bno.setAxisRemap(r);
	    // bno.setAxisSign(0x00);
	    delay(500);
	}

	// delay(1000);

	bno.setExtCrystalUse(true);
}

bool IMU::available()
{
	return ok;
}

// NOTE: not really useful, use GetQuat()
imu::Vector<3> IMU::GetEuler()
{
	// return bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	imu::Quaternion quat = GetQuat();
    // Normalize
    quat.normalize();
    // convert quat to eul
    imu::Vector<3> eul = quat.toEuler();

    return eul;
}

imu::Quaternion IMU::GetQuat()
{
	return bno.getQuat();
}

imu::Vector<3> IMU::GetAcc()
{
	return bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}

imu::Vector<3> IMU::GetGyro()
{
	return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}


