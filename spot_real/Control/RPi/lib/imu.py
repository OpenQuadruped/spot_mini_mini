import time
import board
import busio
import adafruit_lsm9ds1
import numpy as np
import math

# i2c permission: sudo usermod -a -G i2c <user>
# https://www.raspberrypi.org/forums/viewtopic.php?t=58782

# use ROS with python3: https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674

class IMU:
    def __init__(self, rp_flip=True, r_neg=False, p_neg=True, y_neg=True):
        # I2C connection:
        # SPI connection:
        # from digitalio import DigitalInOut, Direction
        # spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        # csag = DigitalInOut(board.D5)
        # csag.direction = Direction.OUTPUT
        # csag.value = True
        # csm = DigitalInOut(board.D6)
        # csm.direction = Direction.OUTPUT
        # csm.value = True
        # sensor = adafruit_lsm9ds1.LSM9DS1_SPI(spi, csag, csm)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_lsm9ds1.LSM9DS1_I2C(self.i2c)
        # Calibration Parameters
        self.x_gyro_calibration = 0
        self.y_gyro_calibration = 0
        self.z_gyro_calibration = 0
        self.roll_calibration = 0
        self.pitch_calibration = 0
        self.yaw_calibration = 0
        # IMU Parameters: acc (x,y,z), gyro(x,y,z)
        self.imu_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Time in seconds
        self.prev_time = time.time()
        # IMU timer
        self.imu_diff = 0
        # Gyroscope integrals for filtering
        self.roll_int = 0
        self.pitch_int = 0
        self.yaw_int = 0
        # Complementary Filter Coefficient
        self.comp_filter = 0.02
        # Filtered RPY
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Used to turn the IMU into right-hand coordinate system
        self.rp_flip = rp_flip
        self.r_neg = r_neg
        self.p_neg = p_neg
        self.y_neg = y_neg

        # Magnemometer Calibration Values
        self.scale_x = 1
        self.scale_y = 1
        self.scale_z = 1
        self.mag_x_bias = 0
        self.mag_y_bias = 0
        self.mag_z_bias = 0
        self.yaw_bias = 0

        # CALIBRATE
        self.calibrate_imu()

        print("IMU Calibrated!\n")

    def calibrate_imu(self):
        """
        """
        # Reset calibration params
        self.x_gyro_calibration = 0
        self.y_gyro_calibration = 0
        self.z_gyro_calibration = 0
        self.roll_calibration = 0
        self.pitch_calibration = 0
        self.yaw_calibration = 0

        sum_xg = 0
        sum_yg = 0
        sum_zg = 0
        sum_xa = 0
        sum_ya = 0
        sum_za = 0
        sum_roll = 0
        sum_pitch = 0
        sum_yaw = 0

        num_calibrations = 1000

        print("Calibrating Gyroscope and Accelerometer...\n")

        for i in range(num_calibrations):
            """ GYRO/ACC CALIBRATION
            """

            self.read_imu()

            sum_xg += self.imu_data[0]
            sum_yg += self.imu_data[1]
            sum_zg += self.imu_data[2]

            sum_xa += self.imu_data[3]
            sum_ya += self.imu_data[4]
            sum_za += self.imu_data[5]

            # Y,Z accelerations make up roll
            sum_roll += (math.atan2(self.imu_data[3],
                                    self.imu_data[5])) * 180.0 / np.pi
            # X,Z accelerations make up pitch
            sum_pitch += (math.atan2(self.imu_data[4],
                                     self.imu_data[5])) * 180.0 / np.pi
            # # Y, X accelerations make up yaw
            # sum_yaw += (math.atan2(self.imu_data[7], self.imu_data[6]) *
            #             180.0 / np.pi)

        # Average values for calibration
        self.x_gyro_calibration = sum_xg / float(num_calibrations)
        self.y_gyro_calibration = sum_yg / float(num_calibrations)
        self.z_gyro_calibration = sum_zg / float(num_calibrations)
        self.roll_calibration = sum_roll / float(num_calibrations)
        self.pitch_calibration = sum_pitch / float(num_calibrations)
        # self.yaw_calibration = sum_yaw / float(num_calibrations)

        print("Gyroscope and Accelerometer calibrated!\n")

        # magne_cal = input(
        #     "Calibrate Magnemometer [c] or Load Existing Calibration [l] ?")

        # if magne_cal == "c":
        #     print("Calibrating Magnetometer...")
        #     self.calibrate_magnemometer()
        # else:
        #     print("Loading Magnetometer Calibration...")
        #     self.load_magnemometer_calibration()

        # input(
        #     "Put the robot at its zero-yaw position and press Enter to calibrate Yaw"
        # )
        # self.read_imu()
        # self.yaw_bias = (math.atan2(self.imu_data[7], self.imu_data[6]) *
        #                  180.0 / np.pi)
        # print("Recorded Bias: {}".format(self.yaw_bias))
        # input("Enter To Start")

    def load_magnemometer_calibration(self):
        return True

    def calibrate_magnemometer(self):
        # Get 10 seconds of magnemometer data
        collection_time = 10.0
        # Hard Iron Offset
        mag_x_max = -32767
        mag_x_min = 32767
        mag_y_max = -32767
        mag_y_min = 32767
        mag_z_max = -32767
        mag_z_min = 32767
        input("Press Enter to start data collection for " +
              str(collection_time) + " seconds:")
        start_time = time.time()
        elapsed_time = time.time() - start_time
        while elapsed_time < collection_time:
            elapsed_time = time.time() - start_time
            self.read_imu()
            # Set Max/Min for x
            if self.imu_data[6] > mag_x_max:
                mag_x_max = self.imu_data[6]
            if self.imu_data[6] < mag_x_min:
                mag_x_min = self.imu_data[6]
            # Set Max/Min for y
            if self.imu_data[7] > mag_y_max:
                mag_y_max = self.imu_data[7]
            if self.imu_data[7] < mag_y_min:
                mag_y_min = self.imu_data[6]
            # Set Max/Min for z
            if self.imu_data[8] > mag_z_max:
                mag_z_max = self.imu_data[8]
            if self.imu_data[8] < mag_z_min:
                mag_z_min = self.imu_data[8]
        # Get Hard Iron Correction
        self.mag_x_bias = (mag_x_max + mag_x_min) / 2.0
        self.mag_y_bias = (mag_y_max + mag_y_min) / 2.0
        self.mag_z_bias = (mag_z_max + mag_z_min) / 2.0

        # Soft Iron Distortion - SCALE BIASES METHOD
        # https://appelsiini.net/2018/calibrate-magnetometer/
        # NOTE: Can also do Matrix Method
        mag_x_delta = (mag_x_max - mag_x_min) / 2.0
        mag_y_delta = (mag_y_max - mag_y_min) / 2.0
        mag_z_delta = (mag_z_max - mag_z_min) / 2.0

        avg_delta = (mag_x_delta + mag_y_delta + mag_z_delta) / 3.0

        self.scale_x = avg_delta / mag_x_delta
        self.scale_y = avg_delta / mag_y_delta
        self.scale_y = avg_delta / mag_z_delta

        # NOW, FOR EACH AXIS: corrected_reading = (reading - bias) * scale

        input("Press Enter to save results")

    def read_imu(self):
        """
        """
        accel_x, accel_y, accel_z = self.sensor.acceleration
        mag_x, mag_y, mag_z = self.sensor.magnetic
        gyro_x, gyro_y, gyro_z = self.sensor.gyro
        # temp = self.sensor.temperature

        # Populate imu data list
        # Gyroscope Values (Degrees/sec)
        self.imu_data[0] = gyro_x - self.x_gyro_calibration
        self.imu_data[1] = gyro_y - self.y_gyro_calibration
        self.imu_data[2] = gyro_z - self.z_gyro_calibration
        # Accelerometer Values (m/s^2)
        self.imu_data[3] = accel_x
        self.imu_data[4] = accel_y
        self.imu_data[5] = accel_z
        # Magnemometer Values
        self.imu_data[6] = (mag_x - self.mag_x_bias) * self.scale_x
        self.imu_data[7] = (mag_y - self.mag_y_bias) * self.scale_y
        self.imu_data[8] = (mag_z - self.mag_z_bias) * self.scale_z

    def filter_rpy(self):
        """
        """
        # Get Readings
        self.read_imu()
        # Get Current Time in seconds
        current_time = time.time()
        self.imu_diff = current_time - self.prev_time
        # Set new previous time
        self.prev_time = current_time
        # Catch rollovers
        if self.imu_diff < 0:
            self.imu_diff = 0

        # Complementary filter for RPY
        # TODO: DOUBLE CHECK THIS!!!!!!!
        roll_gyro_delta = self.imu_data[1] * self.imu_diff
        pitch_gyro_delta = self.imu_data[0] * self.imu_diff
        yaw_gyro_delta = self.imu_data[2] * self.imu_diff

        self.roll_int += roll_gyro_delta
        self.pitch_int += pitch_gyro_delta
        self.yaw_int += yaw_gyro_delta

        # RPY from Accelerometer
        # Y,Z accelerations make up roll
        roll_a = (math.atan2(self.imu_data[3], self.imu_data[5])
                  ) * 180.0 / np.pi - self.roll_calibration
        # X,Z accelerations make up pitch
        pitch_a = (math.atan2(self.imu_data[4], self.imu_data[5])
                   ) * 180.0 / np.pi - self.pitch_calibration
        # Y, X Magnetometer data makes up yaw
        yaw_m = (math.atan2(self.imu_data[7], self.imu_data[6]) * 180.0 /
                 np.pi)

        # Calculate Filtered RPY
        self.roll = roll_a * self.comp_filter + (1 - self.comp_filter) * (
            roll_gyro_delta + self.roll)
        self.pitch = pitch_a * self.comp_filter + (1 - self.comp_filter) * (
            pitch_gyro_delta + self.pitch)
        self.yaw = math.atan2(self.imu_data[7],
                              self.imu_data[6]) * 180.0 / np.pi
        self.yaw = yaw_m - self.yaw_bias
        # Wrap from -PI to PI
        self.yaw -= 360.0 * math.floor((self.yaw + 180) / 360.0)

        # self.yaw = yaw_m * self.comp_filter + (1 - self.comp_filter) * (
        #     pitch_gyro_delta + self.yaw)
        # self.roll = roll_a
        # self.pitch = pitch_a
        # self.yaw = yaw_a

        # Recenter Roll and Pitch for Right Handed Frame
        self.recenter_rp()

    def recenter_rp(self):
        """ LSM9DS1 IMU does not adhere to right hand rule.
            this function allows the user to specify which coordinate
            frame standards to use by deciding when to flip and/or negate
            Roll and Pitch in the class constructor.
        """
        if self.rp_flip:
            if self.r_neg:
                self.true_roll = -self.pitch
            else:
                self.true_roll = self.pitch

            if self.p_neg:
                self.true_pitch = -self.roll
            else:
                self.true_pitch = self.roll
        else:
            if self.r_neg:
                self.true_roll = -self.roll
            else:
                self.true_roll = self.roll

            if self.p_neg:
                self.true_pitch = -self.pitch
            else:
                self.true_pitch = self.pitch

        if self.y_neg:
            self.yaw = -self.yaw


if __name__ == "__main__":
    imu = IMU()

    while True:
        imu.filter_rpy()

        print("ROLL: {} \t PICH: {} \t YAW: {} \n".format(
            imu.true_roll, imu.true_pitch, imu.yaw))