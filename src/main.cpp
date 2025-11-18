#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"

constexpr int pole_pairs = 15;
float phase_resistance = 0.6;
float phase_inductance = 0.0003;
float motor_KV = NOT_SET;
float maxCurrent = 5;
float alignStrength = 1;
float downsample = 50;

// Motor and driver objects
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, motor_KV, phase_inductance);
BLDCDriver3PWM driver = BLDCDriver3PWM( );
LowsideCurrentSense current_sense = LowsideCurrentSense(0.008, 10, _NC, PC_4, PC_5);

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise magnetic sensor hardware
  sensor.init();

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  sensor.update();
  magneticsensor = sensor.getAngle();
  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}