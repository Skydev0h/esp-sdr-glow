idf_build_get_property(streaming ESP_SDR_STREAMING)
if(streaming)
    idf_component_register(REQUIRES streaming)
    return()
endif()
set(esp_sdr_supported_targets esp32 esp32c2 esp32c3 esp32c5 esp32c6 esp32c61 esp32h2 esp32s2 esp32s3 esp32s31)
if(NOT IDF_TARGET IN_LIST esp_sdr_supported_targets)
    message(FATAL_ERROR "No ESP-SDR RF backend for ${IDF_TARGET}. Supported targets: ${esp_sdr_supported_targets}")
endif()

option(SAMPLE_RATE_PROBE "Build volatile hardware sample-rate diagnostics (never package)" OFF)

set(target_dir "targets/${IDF_TARGET}")
set(target_includes "${target_dir}")
if(IDF_TARGET STREQUAL "esp32s31")
    set(target_dir "${target_dir}/burst")
    list(APPEND target_includes "${target_dir}")
endif()
set(receiver "${target_dir}/receiver.c")
set(sources "common/burst_serial.c" "common/burst_gpio.c" "common/burst_version.c" "common/spectrum_stats.c" "common/rx_recalibration.c" "common/rx_calibration_state.c")
set(dependencies esp_driver_gpio esp_hw_support esp_phy esp_wifi nvs_flash esp_timer esp_driver_uart esp-dsp)
option(RING_PROBE "Build bounded experimental ring diagnostics (never package)" OFF)
if(IDF_TARGET STREQUAL "esp32c61" OR IDF_TARGET STREQUAL "esp32c6")
    list(APPEND sources "common/ring_capture.c")
endif()
if(IDF_TARGET STREQUAL "esp32c2")
    list(APPEND sources "${target_dir}/heap_guard.c")
endif()
set(probes RING_PROBE)
set(fragments)
if(NOT IDF_TARGET STREQUAL "esp32s3")
    list(APPEND sources "common/spectrum.c")
endif()

if(IDF_TARGET STREQUAL "esp32s2")
    list(APPEND dependencies esp_usb_cdc_rom_console)
    list(APPEND probes S2_RF_PROBE)
elseif(NOT IDF_TARGET STREQUAL "esp32" AND NOT IDF_TARGET STREQUAL "esp32c2")
    list(APPEND dependencies esp_driver_usb_serial_jtag)
endif()

if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND sources "common/ring_capture.c" "${target_dir}/s3_fft_rnd.S" "${target_dir}/s3_unpack.S" "${target_dir}/s3_core1.S" "${target_dir}/s3_firdot.S")
    # SPEC-mode FFT; pinned submodule, the component manager stays disabled.
    list(APPEND dependencies esp-dsp)
    list(APPEND fragments "${target_dir}/linker.lf")
    list(APPEND probes SAMPLE_RATE_PROBE FILTER_REGISTER_PROBE S3_RF_PROBE)
elseif(IDF_TARGET STREQUAL "esp32c5" OR IDF_TARGET STREQUAL "esp32c6" OR IDF_TARGET STREQUAL "esp32c61")
    set(receiver "families/c5_c6_c61/receiver.c")
    if(IDF_TARGET STREQUAL "esp32c6" AND FILTER_REGISTER_PROBE)
        message(FATAL_ERROR "Filter register diagnostics are not characterized for C6")
    endif()
    list(APPEND probes SAMPLE_RATE_PROBE FILTER_REGISTER_PROBE C5_TUNE_PROBE)
endif()

if(IDF_TARGET STREQUAL "esp32s31")
    list(APPEND sources "${target_dir}/s31_fft_rnd.S" "${target_dir}/s31_unpack.S")
    list(APPEND fragments "${target_dir}/linker.lf")
endif()

if(IDF_TARGET STREQUAL "esp32c61" OR IDF_TARGET STREQUAL "esp32s31")
    list(APPEND sources "common/burst_gain_table.c")
endif()

set(board_includes)
if(IDF_TARGET STREQUAL "esp32")
    # Board UI drivers (ODROID-GO). Requirements are resolved before Kconfig
    # values exist, so list them for every ESP32 build; unused code is dropped.
    list(APPEND dependencies esp_lcd esp_driver_spi esp_driver_ledc esp_driver_dac esp_adc esp_hal_ana_conv)
endif()
if(CONFIG_ESP_SDR_LOCAL_UI)
    list(APPEND sources "boards/common/sdr_view.c")
    list(APPEND board_includes "boards/common")
endif()
if(CONFIG_ESP_SDR_ODROID_GO)
    list(APPEND sources "boards/odroidgo/ui.c" "boards/odroidgo/lcd.c" "boards/odroidgo/io.c"
                        "boards/odroidgo/tone.c" "boards/odroidgo/zb.c")
    list(APPEND board_includes "boards/odroidgo")
endif()

idf_component_register(SRCS "${receiver}" ${sources}
    LDFRAGMENTS ${fragments}
    PRIV_INCLUDE_DIRS "common" ${target_includes} "diagnostics" ${board_includes}
    REQUIRES ${dependencies})

# Regenerate at build time (not only CMake configure time). The embedded record
# is also the artifact exporter's source of revision and UTC build time.
idf_build_get_property(version_python PYTHON)
set(version_header "${CMAKE_CURRENT_BINARY_DIR}/generated/build_version.h")
set(version_profile "${IDF_TARGET}")
if(CONFIG_ESP_SDR_ODROID_GO)
    set(version_profile "odroid-go")
endif()
add_custom_target(esp_sdr_build_version
    COMMAND "${version_python}" "${PROJECT_DIR}/tools/generate_version.py"
        --root "${PROJECT_DIR}" --profile "${version_profile}" --output "${version_header}"
    BYPRODUCTS "${version_header}"
    VERBATIM)
add_dependencies(${COMPONENT_LIB} esp_sdr_build_version)
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Keep the linker guard beside the backend whose SRAM aperture it protects.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${CMAKE_CURRENT_LIST_DIR}/${target_dir}/sram_guard.ld")
if(IDF_TARGET STREQUAL "esp32c61" OR IDF_TARGET STREQUAL "esp32s31")
    target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=phy_write_gain_mem")
endif()
if(IDF_TARGET STREQUAL "esp32c2")
    target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=soc_get_available_memory_regions")
endif()
if(CONFIG_ESP_SDR_ODROID_GO)
    # 320 px wide plot: 512-point FFTs, 480 kept bins, still 4096 samples.
    # Gain index per sample: SPEC levels referred to gain 50 even under AGC.
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SV_FFT_N=512 SV_SEGMENTS=8 SV_BINS=480 SV_GAIN_COMP=1)
endif()
foreach(probe IN LISTS probes)
    if(${probe})
        target_compile_definitions(${COMPONENT_LIB} PRIVATE "${probe}=1")
    endif()
endforeach()

if(IDF_TARGET STREQUAL "esp32c5")
    target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=phy_chip_set_chan_ana")
elseif(IDF_TARGET STREQUAL "esp32c61" OR IDF_TARGET STREQUAL "esp32s31")
    target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=phy_set_channel_rfpll_freq")
else()
    target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=chip_v7_set_chan_ana")
endif()

include("common/rx_calibration_link.cmake")

if(IDF_TARGET STREQUAL "esp32h2")
    target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=write_gain_mem")
endif()
