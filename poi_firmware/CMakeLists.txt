cmake_minimum_required(VERSION 3.16)
set(PARTITION_TABLE_CSV "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")
set(COMPONENTS
    main
    nimble
    led_strip
    nvs_flash
    esp_littlefs
    fatfs
    esp_adc
    esp_driver_gpio
    esp_driver_spi
    esp_hw_support
    esp_system
    freertos
    log
    heap
    soc
    hal
    esp_rom
    esp_common
    esp_ringbuf
    vfs
    newlib
)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(openpixelpoi-streaming)
