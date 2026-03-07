---
name: fwagent
description: ESP-IDF firmware development, review, and modification for ESP32-C3. Use when working on the poi_firmware project, adding features, debugging, or optimizing for the ESP32-C3 platform.
---

# FWAgent: ESP32-C3 Firmware Expert

Specialized guidance for developing high-performance, reliable firmware using ESP-IDF v5.x on the RISC-V ESP32-C3 microcontroller.

## Core Capabilities

### 1. Project Management & Build
- **Environment**: Always export the ESP-IDF environment before building: `. $HOME/esp/v5.5.1/export.sh`.
- **Build**: Compiles the project into firmware binaries. Use `idf.py build` or fallback `bash -c ". $HOME/esp/v5.5.1/export.sh && idf.py build"`.
- **Flash**: Uploads the compiled binaries to the ESP32-C3. Use `idf.py flash` or fallback `bash -c ". $HOME/esp/v5.5.1/export.sh && idf.py flash"`.
- **Monitor**: Displays real-time serial output and handles backtrace decoding. Use `idf.py monitor` or fallback `bash -c ". $HOME/esp/v5.5.1/export.sh && idf.py monitor"`.
- **Clean**: Removes all build artifacts. Use `idf.py fullclean` or fallback `bash -c ". $HOME/esp/v5.5.1/export.sh && idf.py fullclean"`.

### 2. ESP32-C3 Hardware Mapping
- **GPIOs**: 22 programmable GPIOs.
- **USB Serial/JTAG**: Pins 18 (D-) and 19 (D+). Prefer using these for debugging.
- **Strapping Pins**: GPIO2, GPIO8, GPIO9. Be careful when using these as outputs/inputs.
- **Peripherals**: 2x UART, 3x SPI, 1x I2C, 1x I2S, 2x ADC, 1x RMT, 6x LEDC (PWM).

### 3. Firmware Review Checklist
- **Interrupts**: Ensure ISRs are short and use `IRAM_ATTR` if they need to run while flash is busy.
- **Memory**: ESP32-C3 has 400KB SRAM. Monitor heap usage with `esp_get_free_heap_size()`.
- **NVS**: Use Non-Volatile Storage for persistent settings.
- **Power Management**: Use Light-sleep or Deep-sleep for battery-powered applications.

### 4. Common Workflows

#### Adding a New Component
1. Create a directory in `components/`.
2. Add `CMakeLists.txt`: `idf_component_register(SRCS "file.c" INCLUDE_DIRS "include")`.
3. Update main `CMakeLists.txt` if necessary.

#### Debugging Crashes
1. Capture the backtrace from `idf.py monitor`.
2. Use `idf.py addr2line` to map addresses to code lines if needed manually, but `monitor` usually does this automatically.
3. Check for stack overflows; increase task stack size if `Stack canary watchdog triggered`.

## Resources

- **References**: See [esp32c3_peripherals.md](references/esp32c3_peripherals.md) for detailed peripheral configuration.
- **Assets**: See [assets/boilerplate/](assets/boilerplate/) for starting a new ESP-IDF component.

## Optimization for POI
- **LED Control**: Use the **RMT** or **SPI** peripheral for WS2812/APA102 LEDs to ensure timing accuracy.
- **IMU Integration**: Use I2C at 400kHz for fast sensor polling.
- **Task Priority**: Keep LED rendering and IMU polling at high priority to avoid stuttering.
