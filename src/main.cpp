#include "config.h"

#if defined(STM32G4)
uint16_t BRAKE_RESISTANCE = 5 * 100;
float phase_resistance = 0.9f;
// SimpleFOC expects inductance in henries, not millihenries.
float L_d = 0.00116f;
float L_q = 0.00131f;
float motor_KV = 12.5f;
float maxCurrent = 5.0f;
float alignStrength = 4.0f;
#if defined(PWM_INPUT)
STM32PWMInput pwmInput = STM32PWMInput(PB_15_ALT2);
#endif
#endif

#if defined(STM32F4)
uint16_t BRAKE_RESISTANCE = 2 * 100;
float phase_resistance = 5.0f;
float motor_KV = 6.25f;
float maxCurrent = 2;
float alignStrength = 3.0f;
#if defined(PWM_INPUT)
STM32PWMInput pwmInput = STM32PWMInput(PE5);
#endif
#endif
#if defined(ENCODER_DEBUG_TELEMETRY)
float sfoc_electrical_rads;
float spi_mechanical_degrees;
float electrical_degrees;
float sfoc_electrical_degrees;
float radians;
float monitor_target_value = 0.0f;
float monitor_voltage_q_value = 0.0f;
float monitor_voltage_d_value = 0.0f;
float monitor_current_q_ma = 0.0f;
float monitor_current_d_ma = 0.0f;
float abz_mechanical_degrees = 0.0f;
float shaft_mechanical_degrees = 0.0f;
float shaft_vs_spi_degrees = 0.0f;
float foc_zero_electric_degrees = 0.0f;
float foc_expected_electrical_degrees = 0.0f;
float foc_internal_electrical_degrees = 0.0f;
float foc_sensor_electrical_error_degrees = 0.0f;
float foc_stator_vector_degrees = 0.0f;
float foc_stator_vector_magnitude = 0.0f;
float foc_stator_vs_rotor_degrees = 0.0f;
float foc_stator_vs_internal_electrical_degrees = 0.0f;
float foc_sensor_offset_degrees = 0.0f;
float foc_voltage_q = 0.0f;
float foc_voltage_d = 0.0f;
float foc_ualpha = 0.0f;
float foc_ubeta = 0.0f;
float foc_target_current_amps = 0.0f;
float foc_current_q_amps = 0.0f;
float foc_current_d_amps = 0.0f;
float foc_sensor_direction_sign = 0.0f;
float abz_raw_rads = 0.0f;
float abz_effective_rads = 0.0f;
float abz_spi_offset_rads = 0.0f;
float openloop_mechanical_rads = 0.0f;
float openloop_electrical_rads = 0.0f;
float openloop_mechanical_from_electrical_rads = 0.0f;
float openloop_vs_encoder_error_rads = 0.0f;
float spi_mechanical_rads = 0.0f;
float spi_vs_encoder_error_rads = 0.0f;
float spi_vs_openloop_error_rads = 0.0f;
#endif
uint8_t encoder_source_id = ENCODER_SOURCE_DEFAULT;
float phase_inductance = L_q;
float current_bandwidth = 100.0f;
float a = 0.0f, b = 0.0f, c = 0.0f;
uint16_t pwmPeriodCounts = 0;
uint32_t period_ticks = 0;
uint32_t duty_ticks = 0;
uint16_t dutyPercent = 0;
int16_t target_current = 0;
int16_t I_Bus = 0;
bool brake_active = false;
bool break_active = false;
bool simplefoc_init = true;
bool v_error = false;
bool overvoltage_active = false;
uint32_t current_time = 0;
uint32_t t_pwm = 0;
uint16_t MAX_REGEN_CURRENT = 0;
uint16_t BRKRESACT_SENS = 1;
int16_t regenCur = 0;
float v_bus = 0.0f;
bool vbus_adc2_ready = false;
float target = 0.0f;
bool estop_motor_disabled = false;
#if defined(PWM_INPUT)
bool pwm_input_control_enabled = false;
#endif

SimpleFOCDebug debug;
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, motor_KV, phase_inductance);
BLDCDriver3PWM driver = BLDCDriver3PWM((int)PH_A, (int)PH_B, (int)PH_C, (int)BTS_ENABLE_PIN);
#if defined(STM32F4)
LowsideCurrentSense currentsense = LowsideCurrentSense(0.035f, 50.0f, currentPHA, currentPHB, currentPHC);
#elif defined(STM32G4)
LowsideCurrentSense currentsense = LowsideCurrentSense(66.0f, currentPHA, currentPHB, currentPHC);
#endif

#if defined(ENCODER_ABZ_REVERSED)
STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, ENCODER_PIN_B, ENCODER_PIN_A, _NC);
#else
STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, ENCODER_PIN_A, ENCODER_PIN_B, _NC);
#endif

SPIClass SPI_3(MT6835_SPI_MOSI, MT6835_SPI_MISO, MT6835_SPI_SCK);
SPISettings mt6835_spi_settings(1000000, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 encoder2 = MagneticSensorMT6835(MT6835_SPI_CS, mt6835_spi_settings);
#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
Commander commander = Commander(Serial);
#endif

#if defined(ESTOP_ENABLE)
volatile uint8_t estop_flag = 0U;
volatile uint8_t estop_latch_flag = 0U;
#endif

#if defined(ENCODER_DEBUG_TELEMETRY)
static float wrap_pm_pi(float angle) {
	float a = fmodf(angle + _PI, _2PI);
	if (a < 0.0f) {
		a += _2PI;
	}
	return a - _PI;
}

static float normalized_degrees(float angle) {
	return _normalizeAngle(angle) * RAD_2_DEG;
}

static float wrapped_degrees(float angle) {
	return wrap_pm_pi(angle) * RAD_2_DEG;
}

static void update_encoder_debug_telemetry() {
	DQCurrent_s monitor_currents = motor.current;
	if (motor.current_sense && motor.torque_controller != TorqueControlType::foc_current) {
		monitor_currents = motor.current_sense->getFOCCurrents(motor.electrical_angle);
		monitor_currents.q = motor.LPF_current_q(monitor_currents.q);
		monitor_currents.d = motor.LPF_current_d(monitor_currents.d);
	}
	monitor_target_value = motor.target;
	monitor_voltage_q_value = motor.voltage.q;
	monitor_voltage_d_value = motor.voltage.d;

	abz_raw_rads = encoder.getMechanicalAngle();
	abz_effective_rads = abz_raw_rads;
	radians = abz_effective_rads;
	abz_mechanical_degrees = normalized_degrees(abz_raw_rads);

	spi_mechanical_rads = encoder2.getMechanicalAngle();
	spi_mechanical_degrees = normalized_degrees(spi_mechanical_rads);
	abz_spi_offset_rads = wrap_pm_pi(spi_mechanical_rads - abz_raw_rads);

	const float shaft_mechanical_rads = _normalizeAngle(motor.shaftAngle());
	const float internal_electrical_rads = _normalizeAngle(motor.electrical_angle);
	const float sensor_electrical_rads = motor.electricalAngle();
	const float zero_electric_rads = _isset(motor.zero_electric_angle) ? _normalizeAngle(motor.zero_electric_angle) : 0.0f;
	const float expected_electrical_rads = _normalizeAngle(
		static_cast<float>(motor.sensor_direction * pole_pairs) * spi_mechanical_rads - zero_electric_rads);
	const float stator_vector_magnitude = sqrtf(motor.Ualpha * motor.Ualpha + motor.Ubeta * motor.Ubeta);
	const float stator_vector_rads = (stator_vector_magnitude > 0.001f) ? _normalizeAngle(atan2f(motor.Ubeta, motor.Ualpha)) : 0.0f;

	openloop_mechanical_rads = _normalizeAngle(motor.shaft_angle);
	openloop_electrical_rads = internal_electrical_rads;
	electrical_degrees = normalized_degrees(internal_electrical_rads);
	openloop_mechanical_from_electrical_rads = _normalizeAngle(openloop_electrical_rads / (float)pole_pairs);
	openloop_vs_encoder_error_rads = wrap_pm_pi(openloop_mechanical_rads - radians);
	spi_vs_encoder_error_rads = wrap_pm_pi(spi_mechanical_rads - radians);
	spi_vs_openloop_error_rads = wrap_pm_pi(spi_mechanical_rads - openloop_mechanical_rads);
	sfoc_electrical_rads = sensor_electrical_rads;
	sfoc_electrical_degrees = normalized_degrees(sensor_electrical_rads);

	shaft_mechanical_degrees = normalized_degrees(shaft_mechanical_rads);
	shaft_vs_spi_degrees = wrapped_degrees(shaft_mechanical_rads - spi_mechanical_rads);
	foc_zero_electric_degrees = normalized_degrees(zero_electric_rads);
	foc_expected_electrical_degrees = normalized_degrees(expected_electrical_rads);
	foc_internal_electrical_degrees = normalized_degrees(internal_electrical_rads);
	foc_sensor_electrical_error_degrees = wrapped_degrees(expected_electrical_rads - internal_electrical_rads);
	foc_stator_vector_magnitude = stator_vector_magnitude;
	foc_stator_vector_degrees = normalized_degrees(stator_vector_rads);
	foc_stator_vs_rotor_degrees = (stator_vector_magnitude > 0.001f) ? wrapped_degrees(stator_vector_rads - sensor_electrical_rads) : 0.0f;
	foc_stator_vs_internal_electrical_degrees = (stator_vector_magnitude > 0.001f) ? wrapped_degrees(stator_vector_rads - internal_electrical_rads) : 0.0f;
	foc_sensor_offset_degrees = normalized_degrees(motor.sensor_offset);
	foc_voltage_q = motor.voltage.q;
	foc_voltage_d = motor.voltage.d;
	foc_ualpha = motor.Ualpha;
	foc_ubeta = motor.Ubeta;
	foc_target_current_amps = motor.target;
	foc_current_q_amps = motor.current.q;
	foc_current_d_amps = motor.current.d;
	foc_sensor_direction_sign = (motor.sensor_direction == Direction::CW) ? 1.0f : ((motor.sensor_direction == Direction::CCW) ? -1.0f : 0.0f);
}
#endif

void setup() {
	Serial.begin(921600);
	motor.useMonitoring(Serial);
	//debug.enable();
	delay(2000);
	Serial.println("Starting setup...");
#if defined(HAL_CORDIC_MODULE_ENABLED)
	SimpleFOC_CORDIC_Config();
#endif

	SPI_3.begin();
Serial.println("SPI_3.begin() complete...");

	encoder2.init(&SPI_3);
Serial.println("encoder2.init(&SPI_3) setup...");
	#if defined(MT6835_SET_SENSOR_OFFSET_FROM_SPI)
	encoder2.update();
	motor.sensor_offset = encoder2.getMechanicalAngle();
	Serial.printf("MT6835 sensor_offset set: %.4f rad\n", motor.sensor_offset);
	#endif
	#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
	commander.add('B', setBandwidth, "Set current control bandwidth (Hz)");
	commander.add('E', onSetABZResolution, nullptr);
	commander.add('C', onPWMInputControl, nullptr);
	commander.add('M', onMotor, "my motor motion");
	#endif

	pinMode(CALIBRATION_GPIO, OUTPUT);
	digitalWrite(CALIBRATION_GPIO, LOW);

    #if defined(ESTOP_ENABLE)
    estop_init();
	estop_update();
	while (estop_active()) {
		motor.target = 0;
		motor.disable();
		estop_update();
		delay(5);
	}
#endif
Serial.println("Step 3 setup...");
#if defined(VOLTAGE_SENSING)
	vbus_adc2_ready = init_vbus_adc2_dma();
	if (!vbus_adc2_ready) {
		Serial.println("VBUS ADC2 init failed");
	}
#else
	vbus_adc2_ready = false;
	v_bus = (float)supply_voltage_V;
#endif
Serial.println("Step 4 setup...");

#if defined(PWM_INPUT)
	if (pwmInput.initialize() != 0) {
		Serial.println("PWM input init failed");
	}
#endif

	
	driver.voltage_power_supply = supply_voltage_V;
	driver.voltage_limit = driver.voltage_power_supply * 0.9f;
	//motor.voltage_limit = driver.voltage_limit * 0.5f;
	motor.voltage_limit = 2;
	driver.pwm_frequency = PWM_FREQ;
	driver.enable_active_high = true;
Serial.println("Step 6 setup...");

	if (!driver.init()) {
		simplefoc_init = false;
		Serial.println("Driver init failed!");
		return;
	}
Serial.println("Step 7 setup...");

	encoder.init();
	Serial.printf("Encoder init status: %d\n", encoder.initialized);
Serial.println("Step 8 setup...");
#if defined(BTS_BREAK)
	configureBtsBreak();
#elif defined(BTS_OC_MONITOR)
	bts_oc_input_init();
#endif
#if defined(BRAKE_CONTROL_ENABLED)
	if (!configureBrakePwm()) {
		Serial.println("Brake PWM init failed");
	}
#endif

#if defined(VOLTAGE_SENSING)
	v_bus = vbus_adc2_ready ? vbus_from_dma_counts() : 0.0f;
	while (vbus_adc2_ready && (v_bus < supply_voltage_V - 10.0f || v_bus > supply_voltage_V + 1.0f)) {
		Serial.printf("PSU UNDER/OVER VOLTAGE: %.2f V\n", v_bus);
		delay(500);
		v_bus = vbus_adc2_ready ? vbus_from_dma_counts() : 0.0f;
	}
#else
	v_bus = (float)supply_voltage_V;
#endif

	Serial.printf("PSU NOMINAL: %.2f V\n", v_bus);

	
	motor.linkSensor(&encoder2);
	motor.linkDriver(&driver);
	//currentsense.gain_a *= -1;
	//currentsense.gain_c *= -1;
	//currentsense.skip_align = false;
	currentsense.linkDriver(&driver);
	int cs_init = currentsense.init();
	Serial.printf("Current sense init status: %d\n", cs_init); 
	motor.linkCurrentSense(&currentsense);

    motor.axis_inductance.d = L_d;
    motor.axis_inductance.q = L_q;

	motor.PID_current_d.P = phase_inductance * current_bandwidth * _2PI;
	motor.PID_current_d.I = motor.PID_current_d.P * phase_resistance / phase_inductance;
	motor.PID_current_d.D = 0;
	motor.PID_current_d.output_ramp = 0;
	motor.LPF_current_d.Tf = 1 / (_2PI * 3.0f * current_bandwidth);

	motor.PID_current_q.P = phase_inductance * current_bandwidth * _2PI;
	motor.PID_current_q.I = motor.PID_current_q.P * phase_resistance / phase_inductance;
	motor.PID_current_q.D = 0;
	motor.PID_current_q.output_ramp = 0;
	motor.LPF_current_q.Tf = 1 / (_2PI * 3.0f * current_bandwidth);

	motor.current_limit = maxCurrent;
	motor.phase_resistance = phase_resistance;
	motor.voltage_sensor_align = alignStrength;
	motor.monitor_downsample = 10;
	motor.monitor_variables = _MON_CURR_Q | _MON_TARGET | _MON_CURR_D;
	motor.modulation_centered = 1;

    motor.controller = MotionControlType::torque;
	motor.torque_controller = TorqueControlType::voltage;  //estimated_current
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
	
	int m_init = motor.init();
	Serial.printf("Current skip_align: %d\n", m_init);

	#if defined(MT6835_CALIB_OPENLOOP)
	mt6835_autocal_sequence();
	#endif

	#if defined(MOTOR_CHAR)
	Serial.println("Hold The Wheel");
	delay(3000);
	motor_characterisation();
	delay(4000);
	#endif

	int foc_init = motor.initFOC();
	Serial.printf("FOC init status: %d\n", foc_init);
}

void loop() {
	current_time = HAL_GetTick();
	encoder2.update();
	motor.monitor();
	if ((current_time - t_pwm) >= 1) {
		t_pwm = current_time;
		check_vbus();
#if defined(BRAKE_CONTROL_ENABLED)
		#if defined(BRAKE_PWM_TEST_MODE)
		const uint32_t testDuty = (static_cast<uint32_t>(pwmPeriodCounts) * BRAKE_PWM_TEST_DUTY_PERCENT) / 100u;
		__HAL_TIM_SET_COMPARE(&htim_brake, BRAKE_PWM_CHANNEL, testDuty);
		#else
		brake_control();
		#endif
	#endif
#if defined(ENCODER_DEBUG_TELEMETRY)
	update_encoder_debug_telemetry();
	#endif
#if defined(ESTOP_ENABLE)
		estop_update();
		if (estop_active()) {
			if (!estop_motor_disabled) {
				motor.target = 0;
				motor.disable();
				estop_motor_disabled = true;
			}
		} else {
			if (estop_motor_disabled) {
				motor.enable();
				estop_motor_disabled = false;
			}
		}
#endif
#if defined(BTS_OC_MONITOR)
		bts_oc_input_update();
#endif
#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
		commander.run();
#endif
	}

	if (overvoltage_active) {
		driver.setPwm(0.0f, 0.0f, 0.0f);
	} else {
		motor.loopFOC();
#if defined(PWM_INPUT)
		calc_hw_pwm();
		if (pwm_input_control_enabled) {
			target = -target_current_to_amps(target_current);
			motor.move(target);
		} else {
			motor.move();
		}
#else
		motor.move();
#endif
	}
}