#pragma once

#include <Arduino.h>
#include "SimpleFOC.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "encoders/mt6835/MagneticSensorMT6835.h"
#include "encoders/calibrated/CalibratedSensor.h"
#include "drivers/hardware_specific/stm32/stm32_mcu.h"

//#define BTS_BREAK
//#define BTS_OC_MONITOR
#define PWM_INPUT
//#define BRAKE_CONTROL_ENABLED
//#define BRAKE_PWM_TEST_MODE
#define VOLTAGE_SENSING
//#define BRAKE_VOLTAGE_RAMP_ENABLED
#define CHECK_VBUS
//#define ESTOP_REQUIRE_HOLD
#define ESTOP_ENABLE
//#define MOTOR_CHAR
#define USE_CALIBRATED_SENSOR
// Enable MT6835 absolute angle offset capture at startup
#define MT6835_SET_SENSOR_OFFSET_FROM_SPI
// Uncomment to run MT6835 user autocalibration routine at startup.
//#define MT6835_CALIB_OPENLOOP
// Uncomment to run calibrated sensor LUT calibration at startup.
//#define CALIBRATE_SENSOR_ON_STARTUP
// Ramp time to reach calibration speed
#define MT6835_CALIB_RAMP_MS 2000U
// Acceptable speed error in RPM when checking ramp completion
#define MT6835_CALIB_SPEED_TOL_RPM 5.0f
// MT6835 autocalibration frequency (0-7) - drives target RPM and min rotations
#define MT6835_AUTOCAL_FREQ 0U
// Auto-save MT6835 setting changes (e.g. ABZ resolution via Commander E) to EEPROM
#define MT6835_AUTO_EEPROM_WRITE_ON_SETTING
// Encoder telemetry/debug calculations in loop() (ABZ/SPI/openloop comparison values)
//#define ENCODER_DEBUG_TELEMETRY

#define ENCODER_SOURCE_ABZ 0U
#define ENCODER_SOURCE_SPI 1U
// Startup encoder source. Keep ABZ support available via set_encoder_source().
#define ENCODER_SOURCE_DEFAULT ENCODER_SOURCE_SPI

// Flip ABZ direction by swapping A/B channels in software.
// Comment this out if ABZ and SPI already move in the same direction.
#define ENCODER_ABZ_REVERSED

#if defined(PWM_INPUT)
#include "utilities/stm32pwm/STM32PWMInput.h"
#endif
#if defined(STM32G4)
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#endif

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#if defined(STM32G4)
#define BTS_ENABLE_PIN PC4
#define BTS_ENABLE BTS_ENABLE_PIN
#define PH_B PA9
#define PH_C PA8
#define PH_A PA10
#define BTS_OC PB12
#define BTS_OC_GPIO_PORT GPIOB
#define BTS_OC_GPIO_PIN GPIO_PIN_12
#define BTS_OC_AF GPIO_AF6_TIM1
#define BTS_OC_ACTIVE_LOW false
#define CALIBRATION_GPIO PC6
#define A_VBUS PA0
#define currentPHA PA2
#define currentPHB _NC
#define currentPHC PA1
#define ENCODER_PIN_A PB6
#define ENCODER_PIN_B PB7_ALT1
#define ENCODER_PIN_I _NC
#define MT6835_SPI_MOSI PB5_ALT1
#define MT6835_SPI_MISO PC11
#define MT6835_SPI_SCK  PC10
#define MT6835_SPI_CS   PA15_ALT1
#define ESTOP_PORT GPIOC
#define ESTOP_PIN GPIO_PIN_13
#define ESTOP_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define ESTOP_ACTIVE_STATE GPIO_PIN_SET
constexpr int pole_pairs = 15;
constexpr int supply_voltage_V = 25;
constexpr float ADC_REF_V = 3.30f;
#endif

#if defined(STM32F4)
#define BRAKE_RESISTOR PB14
#define BTS_ENABLE_PIN PE8
#define BTS_ENABLE BTS_ENABLE_PIN
#define PH_B PE13
#define PH_C PE11
#define PH_A PE9
#define BTS_OC PE15
#define BTS_OC_GPIO_PORT GPIOE
#define BTS_OC_GPIO_PIN GPIO_PIN_15
#define BTS_OC_AF GPIO_AF1_TIM1
#define BTS_OC_ACTIVE_LOW false
#define CALIBRATION_GPIO LED_BUILTIN
#define A_VBUS PA1
#define currentPHA PA2
#define currentPHB _NC
#define currentPHC PA3
#define ENCODER_PIN_A PC6
#define ENCODER_PIN_B PB5
#define ENCODER_PIN_I _NC
#define MT6835_SPI_MOSI PB15
#define MT6835_SPI_MISO PC2
#define MT6835_SPI_SCK  PB13
#define MT6835_SPI_CS   PB12
#define ESTOP_PORT GPIOA
#define ESTOP_PIN GPIO_PIN_0
#define ESTOP_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define ESTOP_ACTIVE_STATE GPIO_PIN_SET
constexpr int pole_pairs = 6;
constexpr int supply_voltage_V = 24;
constexpr float ADC_REF_V = 3.0f;
#endif



#define ENCODER_PPR 16384
#define RAD_2_DEG 57.2957795131f
#define PWM_FREQ 16000
#define BRAKE_PWM_FREQ 20000
#define BRAKE_PWM_TEST_DUTY_PERCENT 25U
#define ESTOP_DEBOUNCE_MS 30U

// ############################### EMERGENCY STOP INPUT ###############################
// #define ESTOP_ENABLE                 // Enable discrete e-stop input on PA3 (shared with EXTBRK_USE_CH4). Comment out to disable.
// #define ESTOP_BUTTON_NO              // Normally Open (active low). Press once to latch until the next press.
// #define ESTOP_BUTTON_NC              // Normally Closed (active high). Do not define together with ESTOP_BUTTON_NO.
// #define ESTOP_REQUIRE_HOLD           // Require the button to stay pressed for the estop to remain active.
#ifdef ESTOP_ENABLE
  #if defined(ESTOP_BUTTON_NO) && defined(ESTOP_BUTTON_NC)
    #error "Define only one of ESTOP_BUTTON_NO or ESTOP_BUTTON_NC"
  #endif
  #if !defined(ESTOP_BUTTON_NO) && !defined(ESTOP_BUTTON_NC)
    #define ESTOP_BUTTON_NO
  #endif
  #ifndef ESTOP_DEBOUNCE_MS
    #define ESTOP_DEBOUNCE_MS   30U     // Debounce window (ms) for the estop input
  #endif
#endif

constexpr uint32_t CONTROL_LOOP_PERIOD_US = 500;
constexpr float VBUS_RESISTOR_TOP_OHMS = 10000.0f;
constexpr float VBUS_RESISTOR_BOTTOM_OHMS = 1000.0f;
constexpr float v_bus_scale = (VBUS_RESISTOR_TOP_OHMS + VBUS_RESISTOR_BOTTOM_OHMS) / VBUS_RESISTOR_BOTTOM_OHMS;
constexpr float ADC_MAX_COUNTS = 4095.0f;
constexpr float VBUS_ADC_SCALE = ADC_REF_V / ADC_MAX_COUNTS;


constexpr float BRAKE_OVERVOLTAGE_RAMP_START_V = supply_voltage_V + 1.0f;
constexpr float BRAKE_OVERVOLTAGE_RAMP_END_V = supply_voltage_V + 4.0f;


extern uint16_t BRAKE_RESISTANCE;
extern float phase_resistance;
extern float motor_KV;
extern float maxCurrent;
extern float alignStrength;

#if defined(PWM_INPUT)
extern STM32PWMInput pwmInput;
extern bool pwm_input_control_enabled;
#endif

#if defined(ENCODER_DEBUG_TELEMETRY)
extern float degrees;
extern float abz_raw_rads;
extern float abz_effective_rads;
extern float abz_spi_offset_rads;
extern float openloop_mechanical_rads;
extern float openloop_electrical_rads;
extern float openloop_mechanical_from_electrical_rads;
extern float openloop_vs_encoder_error_rads;
extern float spi_mechanical_rads;
extern float spi_vs_encoder_error_rads;
extern float spi_vs_openloop_error_rads;
#endif
extern uint8_t encoder_source_id;
extern float phase_inductance;
extern float current_bandwidth;
extern float a, b, c;
extern uint16_t pwmPeriodCounts;
extern uint32_t period_ticks;
extern uint32_t duty_ticks;
extern uint16_t dutyPercent;
extern int16_t target_current;
extern int16_t I_Bus;
extern bool brake_active;
extern bool break_active;
extern bool simplefoc_init;
extern bool v_error;
extern bool overvoltage_active;
extern uint32_t current_time;
extern uint32_t t_pwm;
extern uint16_t MAX_REGEN_CURRENT;
extern uint16_t BRKRESACT_SENS;
extern int16_t regenCur;
extern float v_bus;
extern bool vbus_adc2_ready;
extern float target;
extern SimpleFOCDebug debug;
extern BLDCMotor motor;
extern BLDCDriver3PWM driver;
extern LowsideCurrentSense currentsense;
extern STM32HWEncoder encoder;
#if defined(USE_CALIBRATED_SENSOR)
extern CalibratedSensor calibrated_encoder;
#endif
extern SPIClass SPI_3;
extern SPISettings mt6835_spi_settings;
extern MagneticSensorMT6835 encoder2;
#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
extern Commander commander;
#endif

bool wait_for_yes_no(const char* prompt, uint32_t timeout_ms = 0);
uint8_t mt6835_cal_state(void);
void ramp_velocity(BLDCMotor& m, float start_vel, float end_vel, uint32_t duration_ms);
void mt6835_autocal_sequence(void);
void calibrated_sensor_lut_sequence(void);
float mt6835_target_rpm_from_freq(uint8_t freq);
uint8_t mt6835_autocal_freq_from_rpm(float rpm);
float mt6835_min_rotations_from_freq(uint8_t freq);
bool set_encoder_source(uint8_t source, bool reinit_foc = true);
uint8_t get_encoder_source(void);

void setup();
void loop();

void calc_hw_pwm(void);
#if defined(CHECK_VBUS)
void check_vbus();
#else
static inline void check_vbus() {}
#endif

#if defined(BRAKE_CONTROL_ENABLED)
void brake_control(void);
bool configureBrakePwm(void);
#endif

#if defined(BTS_BREAK)
void configureBtsBreak(void);
#endif
#if defined(BTS_OC_MONITOR)
void bts_oc_input_init(void);
void bts_oc_input_update(void);
#endif

bool init_vbus_adc2_dma(void);
float vbus_from_dma_counts(void);
volatile uint32_t* vbus_adc2_dma_address(void);
float target_current_to_amps(int16_t target);

#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
void setBandwidth(char* cmd);
void onSetABZResolution(char* cmd);
void onPWMInputControl(char* cmd);
void onMotor(char* cmd);
#endif

void estop_init(void);
void estop_update(void);
#if defined(ESTOP_ENABLE)
extern volatile uint8_t estop_flag;
extern volatile uint8_t estop_latch_flag;
static inline uint8_t estop_active(void) { return estop_flag; }
static inline uint8_t estop_latched(void) { return estop_latch_flag; }
#else
static inline uint8_t estop_active(void) { return 0U; }
static inline uint8_t estop_latched(void) { return 0U; }
#endif

void motor_characterisation(void);
