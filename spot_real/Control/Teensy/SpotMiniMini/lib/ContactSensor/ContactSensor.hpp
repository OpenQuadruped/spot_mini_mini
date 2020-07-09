#ifndef CONTACTSENSOR_INCLUDE_GUARD_HPP
#define CONTACTSENSOR_INCLUDE_GUARD_HPP
/// \file
/// \brief Contact Sensor Library. Adapted from https://github.com/adham-elarabawy/OpenQuadruped
class ContactSensor {
  private:
  int led_pin;
  int sensor_pin;
  double center = 0;
  double prev_val = 0;
  double val = 0;
  int numSamples = 200;
  double thresh = 2.5;
  double alpha = 0.999;

  public:
  void init(int in_sensor_pin, int in_led_pin);
  bool isTriggered();
  void update_clk();
};

#endif
