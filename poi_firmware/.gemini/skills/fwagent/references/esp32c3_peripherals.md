# ESP32-C3 Peripheral Configuration

Detailed configuration for core peripherals used in POI firmware.

## 1. I2C (Internal IMU / Display)
- **Controller**: 1x I2C master/slave.
- **Max Frequency**: 1 MHz (400 kHz recommended for stability).
- **Configuration Example**:
```c
i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPIO_NUM_X,
    .scl_io_num = GPIO_NUM_Y,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,
};
i2c_param_config(I2C_NUM_0, &conf);
i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
```

## 2. LEDC (PWM for LED strip / Dimming)
- **Timer**: 1x LEDC timer.
- **Channels**: 6x LEDC channels.
- **Frequency**: Up to 40 MHz.
- **Resolution**: Up to 14-bit.
- **Note**: ESP32-C3 only supports "Low Speed Mode".

## 3. RMT (NeoPixel / WS2812 Control)
- **Channels**: 2x Transmit, 2x Receive.
- **Usage**: Use the `led_strip` component or `rmt_tx` driver for timing-critical pulses.
- **Timing**: WS2812 requires precise ~800kHz bitstreams.

## 4. SPI (High-speed LED strips / Display)
- **SPI2**: The general-purpose SPI (often called "GPSPI2").
- **Pins**: Can be mapped to any GPIO via GPIO Matrix, but IOMUX is faster for 40+ MHz.
- **Typical Pins (IOMUX)**:
    - CS: GPIO7
    - SCLK: GPIO6
    - MOSI: GPIO5
    - MISO: GPIO4

## 5. ADC (Battery Sensing)
- **ADC1**: 5 channels (GPIO0 - GPIO4).
- **Resolution**: 12-bit.
- **Attenuation**: Use `ADC_ATTEN_DB_12` for full-range (0V - 3.1V).
- **Calibration**: Always use `esp_adc_cal_characterize` for accurate voltage readings.
