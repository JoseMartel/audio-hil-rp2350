cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2)
include(pico_sdk_import.cmake)
project(audio_hil_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

# Configure CMSIS-DSP dependencies
if(NOT DEFINED CMSIS_DSP_PATH)
    if(DEFINED ENV{CMSIS_DSP_PATH})
        set(CMSIS_DSP_PATH "$ENV{CMSIS_DSP_PATH}" CACHE PATH "Ruta local a la carpeta raíz de CMSIS-DSP")
    else()
        message(FATAL_ERROR "La variable de entorno CMSIS_DSP_PATH no está definida. Defínela en tu terminal (e.g. en ~/.bashrc) apuntando al directorio raíz de CMSIS-DSP clonado localmente.")
    endif()
endif()

# We use the CMSIS Core headers bundled with the Pico SDK.
set(CMSISCORE "${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core" CACHE PATH "CMSIS Core Path")

# Disable float16 functions to avoid compile issues with older compilers and keep build lightweight
set(DISABLEFLOAT16 ON CACHE BOOL "Disable float16 kernels in CMSIS-DSP")

# Include CMSIS-DSP sources from the external path
add_subdirectory(${CMSIS_DSP_PATH}/Source bin_dsp)

# Main executable with layered HIL components
add_executable(audio_hil
    main.c
    components/src/hil_hal.c
    components/src/dsp_processor.c
    components/src/frequency_estimator.c
)

# Setup include directories
target_include_directories(audio_hil PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/components/include
)

# Enable USB stdio printing and binary CDC transfer
pico_enable_stdio_usb(audio_hil 1)
pico_enable_stdio_uart(audio_hil 0)
pico_add_extra_outputs(audio_hil)

# Link standard SDK libraries, hardware peripherals, CMSIS-DSP, and math library
target_link_libraries(audio_hil 
    pico_stdlib 
    pico_stdio_usb
    CMSISDSP
    m
)

add_custom_target(
    create_symlink ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
)
