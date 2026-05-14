#include "config.h"

#include <math.h>

bool wait_for_yes_no(const char* prompt, uint32_t timeout_ms) {
  Serial.print(prompt);
  Serial.print(" [y/n]: ");
  const uint32_t start = HAL_GetTick();
  while (true) {
    if (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == 'y' || c == 'Y' || c == '1') {
        Serial.println("y");
        return true;
      }
      if (c == 'n' || c == 'N' || c == '0') {
        Serial.println("n");
        return false;
      }
    }
    if (timeout_ms > 0U && (HAL_GetTick() - start) >= timeout_ms) {
      Serial.println("(timeout)");
      return false;
    }
    delay(10);
  }
}

uint8_t mt6835_cal_state(void) {
  const uint8_t raw = encoder2.getCalibrationStatus();
  // Some driver versions already return 2-bit state; others return full register byte.
  return (raw <= 0x03u) ? raw : ((raw >> 6) & 0x03u);
}

float mt6835_target_rpm_from_freq(uint8_t freq) {
  static const float midpoints[8] = {
    4800.0f, // 3200-6400
    2400.0f, // 1600-3200
    1200.0f, // 800-1600
    600.0f,  // 400-800 (factory default)
    300.0f,  // 200-400
    150.0f,  // 100-200
    75.0f,   // 50-100
    37.5f    // 25-50
  };
  const uint8_t idx = (freq < 8U) ? freq : 3U;
  return midpoints[idx];
}

uint8_t mt6835_autocal_freq_from_rpm(float rpm) {
  if (rpm >= 3200.0f) return 0u;
  if (rpm >= 1600.0f) return 1u;
  if (rpm >= 800.0f)  return 2u;
  if (rpm >= 400.0f)  return 3u;
  if (rpm >= 200.0f)  return 4u;
  if (rpm >= 100.0f)  return 5u;
  if (rpm >= 50.0f)   return 6u;
  return 7u;
}

float mt6835_min_rotations_from_freq(uint8_t freq) {
  (void)freq;
  return 64.0f; // datasheet guidance (>=64 rotations)
}

void ramp_velocity(BLDCMotor& m, float start_vel, float end_vel, uint32_t duration_ms) {
  const uint32_t start = HAL_GetTick();
  if (duration_ms == 0U) {
    m.target = end_vel;
    m.loopFOC();
    m.move();
    return;
  }

  while (true) {
    const uint32_t now = HAL_GetTick();
    float progress = static_cast<float>(now - start) / static_cast<float>(duration_ms);
    progress = CLAMP(progress, 0.0f, 1.0f);
    m.target = start_vel + (end_vel - start_vel) * progress;
    m.loopFOC();
    m.move();
    if ((now - start) >= duration_ms) {
      break;
    }
  }
  m.target = end_vel;
}

#if defined(MT6835_CALIB_OPENLOOP) && defined(SIMPLEFOC_STM32_DEBUG)
void mt6835_autocal_sequence(void) {
  if (!wait_for_yes_no("Start MT6835 User-AutoCalibration?", 5000U)) {
    return;
  }

  bool retry_calib = true;
  while (retry_calib) {
    retry_calib = false;

    MT6835Options4 opts4 = encoder2.getOptions4();
    if (opts4.autocal_freq != MT6835_AUTOCAL_FREQ) {
      opts4.autocal_freq = MT6835_AUTOCAL_FREQ;
      encoder2.setOptions4(opts4);
    }

    float target_rpm = mt6835_target_rpm_from_freq(opts4.autocal_freq);
    const float requested_velocity = (target_rpm * _2PI) / 60.0f;
    float effective_velocity = requested_velocity;
    const float vel_tol = (MT6835_CALIB_SPEED_TOL_RPM * _2PI) / 60.0f;
    const float min_rotations = mt6835_min_rotations_from_freq(opts4.autocal_freq);
    uint32_t min_calib_ms = static_cast<uint32_t>((min_rotations / target_rpm) * 60.0f * 1000.0f);

    const MotionControlType prev_controller = motor.controller;
    const TorqueControlType prev_torque_controller = motor.torque_controller;
    const float prev_voltage_limit = motor.voltage_limit;
    const float prev_target = motor.target;

    motor.controller = MotionControlType::velocity_openloop;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = alignStrength;
    motor.target = 0.0f;

    const float stall_floor_rad_s = 0.5f;
    const uint32_t plateau_timeout_ms = 400U;
    bool aborted = false;
    bool plateau_detected = false;
    float best_velocity = -1000.0f;
    uint32_t last_improve_tick = HAL_GetTick();
    const uint32_t ramp_start = last_improve_tick;

    while ((HAL_GetTick() - ramp_start) < MT6835_CALIB_RAMP_MS) {
      const uint32_t now = HAL_GetTick();
      const float progress = (float)(now - ramp_start) / (float)MT6835_CALIB_RAMP_MS;
      motor.target = effective_velocity * CLAMP(progress, 0.0f, 1.0f);
      motor.loopFOC();
      motor.move();
      const float shaft = motor.shaftVelocity();

      if ((now - ramp_start) > 200U && fabsf(shaft) < stall_floor_rad_s) {
        Serial.println("MT6835 calibration aborted: motor stalled during ramp. Lower calibration RPM.");
        aborted = true;
        break;
      }

      if (shaft > best_velocity + (vel_tol * 0.5f)) {
        best_velocity = shaft;
        last_improve_tick = now;
      }

      if ((now - last_improve_tick) > plateau_timeout_ms && (motor.target - shaft) > vel_tol) {
        plateau_detected = true;
        effective_velocity = CLAMP(0.8f * shaft, stall_floor_rad_s, requested_velocity);
        const float new_rpm = effective_velocity * 60.0f / _2PI;
        uint8_t new_freq = mt6835_autocal_freq_from_rpm(new_rpm);
        if (new_freq != opts4.autocal_freq) {
          opts4.autocal_freq = new_freq;
          encoder2.setOptions4(opts4);
          target_rpm = mt6835_target_rpm_from_freq(opts4.autocal_freq);
          min_calib_ms = static_cast<uint32_t>((min_rotations / target_rpm) * 60.0f * 1000.0f);
        }
        Serial.printf("Velocity plateau at %.2f rad/s (%.1f RPM), reducing target to %.2f rad/s and setting AUTOCAL_FREQ=%u\n", shaft, new_rpm, effective_velocity, opts4.autocal_freq);
        break;
      }
    }

    if (aborted) {
      digitalWrite(CALIBRATION_GPIO, LOW);
      ramp_velocity(motor, motor.target, 0.0f, 1000U);
      motor.controller = prev_controller;
      motor.torque_controller = prev_torque_controller;
      motor.voltage_limit = prev_voltage_limit;
      motor.target = prev_target;
      if (wait_for_yes_no("Retry MT6835 calibration?", 5000U)) {
        retry_calib = true;
      }
      continue;
    }

    if (plateau_detected && motor.target > effective_velocity) {
      ramp_velocity(motor, motor.target, effective_velocity, 700U);
    }

    bool speed_ready = false;
    const uint32_t speed_wait_start = HAL_GetTick();
    while ((HAL_GetTick() - speed_wait_start) < 3000U) {
      motor.loopFOC();
      motor.move();
      const float shaft = motor.shaftVelocity();
      if (fabsf(shaft - effective_velocity) <= vel_tol) {
        speed_ready = true;
        break;
      }
      if (fabsf(shaft) < stall_floor_rad_s) {
        Serial.println("MT6835 calibration aborted: motor stalled while holding speed.");
        aborted = true;
        break;
      }
    }

    if (!speed_ready || aborted) {
      digitalWrite(CALIBRATION_GPIO, LOW);
      ramp_velocity(motor, motor.target, 0.0f, 1000U);
      motor.controller = prev_controller;
      motor.torque_controller = prev_torque_controller;
      motor.voltage_limit = prev_voltage_limit;
      motor.target = prev_target;
      if (!aborted) {
        Serial.println("MT6835 calibration skipped: speed not reached.");
      }
      if (wait_for_yes_no("Retry MT6835 calibration?", 5000U)) {
        retry_calib = true;
      }
      continue;
    }

    digitalWrite(CALIBRATION_GPIO, HIGH);
    delay(100);
    uint8_t state = mt6835_cal_state();
    bool use_time_fallback = false;
    if (state == 0x00u) {
      use_time_fallback = true;
      Serial.println("MT6835 status unavailable/not-running, using time fallback (>=64 rotations).");
    } else if (state != 0x01u) {
      Serial.printf("MT6835 autocal not running (state=%u)\n", state);
      digitalWrite(CALIBRATION_GPIO, LOW);
      ramp_velocity(motor, motor.target, 0.0f, 1000U);
      motor.controller = prev_controller;
      motor.torque_controller = prev_torque_controller;
      motor.voltage_limit = prev_voltage_limit;
      motor.target = prev_target;
      continue;
    }

    const uint32_t calib_start = HAL_GetTick();
    bool calibration_done = false;
    bool calibration_success = false;
    bool status_warned = false;

    while (!calibration_done) {
      motor.loopFOC();
      motor.move();
      state = mt6835_cal_state();
      const uint32_t elapsed = HAL_GetTick() - calib_start;

      if (use_time_fallback) {
        if (!status_warned) {
          Serial.println("Using time-based completion only (no SPI status feedback).");
          status_warned = true;
        }
        if (fabsf(motor.shaftVelocity()) < stall_floor_rad_s) {
          Serial.println("MT6835 calibration aborted: stall during fallback calibration.");
          aborted = true;
          calibration_done = true;
          continue;
        }
        if (elapsed >= min_calib_ms) {
          calibration_success = true;
          calibration_done = true;
        }
        continue;
      }

      if (state == 0x01u) {
        if (fabsf(motor.shaftVelocity()) < stall_floor_rad_s) {
          Serial.println("MT6835 calibration aborted: stall during calibration.");
          aborted = true;
          calibration_done = true;
        }
        continue;
      }

      if (state == 0x03u) {
        calibration_success = true;
        calibration_done = true;
      } else if (state == 0x02u) {
        calibration_done = true;
      }
    }

    digitalWrite(CALIBRATION_GPIO, LOW);
    ramp_velocity(motor, motor.target, 0.0f, 3000U);

    if (calibration_success && !aborted) {
      Serial.println("MT6835 calibration successful.");
      for (int i = 10; i >= 1; i--) {
        Serial.printf("Wait %d s\n", i);
        delay(1000);
      }
      for (int i = 0; i < 5; i++) {
        Serial.println("Power Off the system fully");
        delay(1000);
      }
    } else {
      Serial.println("MT6835 calibration failed");
      if (wait_for_yes_no("Retry MT6835 calibration?", 5000U)) {
        retry_calib = true;
      }
    }

    motor.controller = prev_controller;
    motor.torque_controller = prev_torque_controller;
    motor.voltage_limit = prev_voltage_limit;
    motor.target = prev_target;
  }
}
#endif

#if defined(USE_CALIBRATED_SENSOR) && defined(CALIBRATE_SENSOR_ON_STARTUP) && defined(SIMPLEFOC_STM32_DEBUG)
void calibrated_sensor_lut_sequence(void) {
  if (!wait_for_yes_no("Start calibrated sensor LUT calibration?", 5000U)) {
    return;
  }
  calibrated_encoder.voltage_calibration = alignStrength;
  calibrated_encoder.calibrate(motor);
}
#endif

#if defined(VOLTAGE_SENSING)
static ADC_HandleTypeDef hadc2 = {0};
static DMA_HandleTypeDef hdma_adc2 = {0};
static volatile uint32_t vbus_adc2_dma_raw = 0;
#endif

#if defined(ESTOP_ENABLE)
static uint8_t estop_state = 0U;
static uint8_t estop_sample_prev = 0U;
static uint32_t estop_change_tick = 0U;
#endif

float target_current_to_amps(int16_t target) {
  constexpr float TARGET_INPUT_MAX = 16000.0f;
  float normalized = static_cast<float>(target) / TARGET_INPUT_MAX;
  normalized = CLAMP(normalized, -1.0f, 1.0f);
  return normalized * maxCurrent;
}

#if defined(VOLTAGE_SENSING)
bool init_vbus_adc2_dma(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();

#if defined(STM32F4)
  __HAL_RCC_ADC2_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
#elif defined(STM32G4)
  __HAL_RCC_ADC12_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
#ifdef __HAL_RCC_DMAMUX1_CLK_ENABLE
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
#endif
#else
  return false;
#endif

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0;
#if defined(STM32F4)
  gpio.Pin = GPIO_PIN_1;
#endif
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  hadc2.Instance = ADC2;
#if defined(STM32F4)
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.NbrOfDiscConversion = 0;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
#elif defined(STM32G4)
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
#endif

#if defined(STM32F4)
  hdma_adc2.Instance = DMA2_Stream2;
  hdma_adc2.Init.Channel = DMA_CHANNEL_1;
  hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc2.Init.MemInc = DMA_MINC_DISABLE;
  hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc2.Init.Mode = DMA_CIRCULAR;
  hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#elif defined(STM32G4)
  hdma_adc2.Instance = DMA1_Channel1;
  hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
  hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc2.Init.MemInc = DMA_MINC_DISABLE;
  hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc2.Init.Mode = DMA_CIRCULAR;
  hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
#endif

  if (HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
    return false;
  }
  __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2);

  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    return false;
  }

  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_1;
#if defined(STM32G4)
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
#elif defined(STM32F4)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
#endif

  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    return false;
  }

#if defined(STM32G4)
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    return false;
  }
#endif

  if (HAL_ADC_Start_DMA(&hadc2, const_cast<uint32_t*>(&vbus_adc2_dma_raw), 1) != HAL_OK) {
    return false;
  }

  return true;
}

float vbus_from_dma_counts(void) {
  const float pin_voltage = static_cast<float>(vbus_adc2_dma_raw & 0x0FFFu) * VBUS_ADC_SCALE;
  return pin_voltage * v_bus_scale;
}

volatile uint32_t* vbus_adc2_dma_address(void) {
  return &vbus_adc2_dma_raw;
}
#else
bool init_vbus_adc2_dma(void) { return false; }
float vbus_from_dma_counts(void) { return static_cast<float>(supply_voltage_V); }
volatile uint32_t* vbus_adc2_dma_address(void) { return nullptr; }
#endif

void estop_init(void) {
#if defined(ESTOP_ENABLE)
  ESTOP_GPIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = ESTOP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ESTOP_PORT, &GPIO_InitStruct);

  estop_flag = 0U;
  estop_latch_flag = 0U;
  estop_change_tick = HAL_GetTick();
  estop_sample_prev = (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == ESTOP_ACTIVE_STATE);
  estop_state = estop_sample_prev;
#endif
}

void estop_update(void) {
#if defined(ESTOP_ENABLE)
  const uint32_t now = HAL_GetTick();
  const uint8_t sample = (HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == ESTOP_ACTIVE_STATE);

  if (sample != estop_sample_prev) {
    estop_sample_prev = sample;
    estop_change_tick = now;
  }

  if ((now - estop_change_tick) >= ESTOP_DEBOUNCE_MS && estop_state != estop_sample_prev) {
    estop_state = estop_sample_prev;
    if (estop_state) {
#if defined(ESTOP_REQUIRE_HOLD)
      estop_flag = 1U;
#else
      if (estop_latch_flag) {
        estop_latch_flag = 0U;
        estop_flag = 0U;
      } else {
        estop_flag = 1U;
#if defined(ESTOP_BUTTON_NO)
        estop_latch_flag = 1U;
#endif
      }
#endif
    } else {
#if defined(ESTOP_REQUIRE_HOLD)
      estop_flag = 0U;
#else
#if defined(ESTOP_BUTTON_NO)
      if (!estop_latch_flag) {
        estop_flag = 0U;
      }
#else
      estop_flag = 0U;
#endif
#endif
    }
  }
#endif
}

#if defined(PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF)
static bool mt6835_commit_eeprom_with_wait(const char* what) {
  const char* item = (what != nullptr) ? what : "setting";
  Serial.printf("Saving MT6835 %s to EEPROM...\n", item);
  const bool ack = encoder2.writeEEPROM();
  if (!ack) {
    Serial.println("MT6835 EEPROM write command failed (no ACK).");
    return false;
  }

  // MT6835 requires settle time after program command.
  for (int s = 6; s >= 1; --s) {
    Serial.printf("MT6835 EEPROM programming: wait %d s\n", s);
    delay(1000);
  }
  Serial.println("MT6835 EEPROM save complete.");
  return true;
}

void setBandwidth(char* cmd) {
  float new_bandwidth = current_bandwidth;
  commander.scalar(&new_bandwidth, cmd);

  if (new_bandwidth > 0 && new_bandwidth <= 1000) {
    current_bandwidth = new_bandwidth;

    motor.PID_current_d.P = phase_inductance * current_bandwidth * _2PI;
    motor.PID_current_d.I = motor.PID_current_d.P * phase_resistance / phase_inductance;
    motor.LPF_current_d.Tf = 1 / (_2PI * 3.0f * current_bandwidth);

    motor.PID_current_q.P = phase_inductance * current_bandwidth * _2PI;
    motor.PID_current_q.I = motor.PID_current_q.P * phase_resistance / phase_inductance;
    motor.LPF_current_q.Tf = 1 / (_2PI * 3.0f * current_bandwidth);

    Serial.printf("Current bandwidth set to: %.1f Hz\n", current_bandwidth);
    Serial.printf("Updated PID_P: %.6f, PID_I: %.6f, LPF_Tf: %.6f\n",
                  motor.PID_current_d.P, motor.PID_current_d.I, motor.LPF_current_d.Tf);
  } else {
    Serial.printf("Invalid bandwidth: %.1f Hz (must be 0-1000)\n", new_bandwidth);
  }
}

void onSetABZResolution(char* cmd) {
  float requestedPprFloat = 16384.0f;
  commander.scalar(&requestedPprFloat, cmd);
  long requestedPpr = (long)requestedPprFloat;

  if (requestedPpr < 1) {
    requestedPpr = 1;
  } else if (requestedPpr > 16384) {
    requestedPpr = 16384;
  }

  const uint16_t rawAbz = static_cast<uint16_t>(requestedPpr - 1L);
  encoder2.setABZResolution(rawAbz);

#if defined(MT6835_AUTO_EEPROM_WRITE_ON_SETTING)
  const bool eeprom_saved = mt6835_commit_eeprom_with_wait("ABZ resolution");
#else
  const bool eeprom_saved = false;
#endif

  const uint16_t readRawAbz = encoder2.getABZResolution();
  Serial.printf("ABZ rb ppr=%u, eeprom=%s\n", (unsigned)(readRawAbz + 1u), eeprom_saved ? "ok" : "not-saved");
}

void onSetEncoderSource(char* cmd) {
  if (cmd == nullptr || cmd[0] == '\0') {
    Serial.printf("SE<0|1> (0=ABZ,1=SPI), current=%u\n", (unsigned)get_encoder_source());
    return;
  }

  const char* p = cmd;
  while (*p == ' ' || *p == '\t') {
    ++p;
  }

  if (*p == 'E' || *p == 'e') {
    ++p;
  }

  while (*p == ' ' || *p == '\t' || *p == '=' || *p == ':') {
    ++p;
  }

  uint8_t requested = 0xFFu;
  if (*p == '0' || *p == 'A' || *p == 'a') {
    requested = ENCODER_SOURCE_ABZ;
  } else if (*p == '1' || *p == 'S' || *p == 's') {
    requested = ENCODER_SOURCE_SPI;
  }

  if (requested == 0xFFu) {
    Serial.println("Invalid source. Use SE0 (ABZ) or SE1 (SPI).");
    return;
  }

  if (set_encoder_source(requested, true)) {
    Serial.printf("Encoder source now %u\n", (unsigned)get_encoder_source());
  } else {
    Serial.println("Failed to switch encoder source.");
  }
}

void onPWMInputControl(char* cmd) {
#if defined(PWM_INPUT)
  if (cmd == nullptr || cmd[0] == '\0') {
    Serial.printf("C=%u\n", pwm_input_control_enabled ? 1u : 0u);
    return;
  }

  if (cmd[0] == '0') {
    pwm_input_control_enabled = false;
    Serial.println("C0");
  } else if (cmd[0] == '1') {
    pwm_input_control_enabled = true;
    Serial.println("C1");
  }
#else
  (void)cmd;
  Serial.println("PWM_INPUT off");
#endif
}

void onMotor(char* cmd) {
  commander.motor(&motor, cmd);
}
#endif

void motor_characterisation(void) {
motor.characteriseMotor(alignStrength);
  float R = motor.phase_resistance;
  float L_d = motor.axis_inductance.d;
  float L_q = motor.axis_inductance.q;

  Serial.print("Resistance: ");
  Serial.print(R);
  Serial.println(" Ohms");

  Serial.print("D-axis inductance: ");
  Serial.print(L_d * 1000);
  Serial.println(" mH");

  Serial.print("Q-axis inductance: ");
  Serial.print(L_q * 1000);
  Serial.println(" mH");
}

