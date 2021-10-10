cmake_minimum_required(VERSION 3.12.0)

project(bootloader)

enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(COMMON ${CMAKE_CURRENT_SOURCE_DIR}/../../common)
set(CMSIS Drivers/CMSIS)
set(DEVICE ${CMSIS}/Device/ST/STM32F3xx)
set(HAL Drivers/STM32F3xx_HAL_Driver)
set(LIB Lib)
set(TLSF ${LIB}/tlsf)

set(FPU hard)
set(COPTS)

# C++ specific options here (added to USE_OPT).
set(CPPOPTS -fno-rtti)

set(CWARN -Wall -Wextra -Wundef -Wstrict-prototypes)

# Define C++ warning options here.
set(CPPWARN -Wall -Wextra -Wundef)

set(LINKERSCRIPT STM32F303CCTx_FLASH.ld)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/${LINKERSCRIPT})

FILE(GLOB STM32CUBE ${HAL}/Src/*.c)
FILE(GLOB STM32CUBE_HAL ${HAL}/Src/stm32f3xx_hal*.c
${HAL}/Src/stm32f3xx_ll_fmc.c
${HAL}/Src/stm32f3xx_ll_usb.c)
list(REMOVE_ITEM STM32CUBE 
${STM32CUBE_HAL}
)


FILE(GLOB CORE_SRC
startup_stm32f303xc.s
)

FILE(GLOB LIB_SRC
${TLSF}/tlsf.c
Drivers/platform_specific_components/stm32/libcanard/bxcan/src/bxcan.c
)

FILE(GLOB APP_SRC 
App/Src/*.cpp
App/Src/*.c
)

set(EXECUTABLE ${PROJECT_NAME}.elf)

add_executable(${EXECUTABLE} 
${STM32CUBE} 
${CORE_SRC} 
${APP_SRC}
${LIB_SRC}
)

target_include_directories( ${EXECUTABLE} PRIVATE
App/Inc
${TLSF}
${CMSIS}/Include
${DEVICE}/Include
${HAL}/Inc 
Drivers/platform_specific_components/stm32/libcanard/bxcan/src
${LIB}/kocherga/kocherga
)

target_compile_definitions(
${EXECUTABLE} PRIVATE
NODENAME=${NODENAME}
USE_FULL_LL_DRIVER
HSE_VALUE=8000000 
HSE_STARTUP_TIMEOUT=100
LSE_STARTUP_TIMEOUT=5000
LSE_VALUE=32768
EXTERNAL_CLOCK_VALUE=8000000
HSI_VALUE=8000000
LSI_VALUE=40000
VDD_VALUE=3300
PREFETCH_ENABLE=1
STM32F303xC
$<$<CONFIG:Debug>:DEBUG_BUILD=1>
_DEBUG=0
BXCAN_MAX_IFACE_INDEX=0
BXCAN_BUSYWAIT_DELAY_SYSTEM_CORE_CLOCK=72000000
)

if(${FPU} MATCHES no)
set(IS_FPU FALSE)
set(FPU_CFG soft)
else()
set(IS_FPU TRUE)
set(FPU_CFG ${FPU})
endif()
set(MCU 
-mcpu=cortex-m4
-mthumb
-mfpu=fpv4-sp-d16
-mfloat-abi=${FPU_CFG}
)

target_compile_options(${EXECUTABLE} PRIVATE
${MCU}
-fdata-sections
-ffunction-sections
-fno-common
-fomit-frame-pointer 
-falign-functions=16
${UDEF}
$<$<COMPILE_LANGUAGE:CXX>:${CPPWARN} ${CPPOPTS}> 
$<$<COMPILE_LANGUAGE:C>:${CWARN} ${COPTS}>
$<$<CONFIG:Debug>:-g -Os>
$<$<CONFIG:Release>: -Os>
$<$<CONFIG:MinSizeRel>: -Os>
$<$<CONFIG:RelWithDebInfo>:-ggdb -Os>
)

target_link_options(${EXECUTABLE} PRIVATE
${MCU}
-specs=nano.specs
-lstdc++ 
-lsupc++
-lc 
-lm 
-lnosys
-Wl,-Map=${PROJECT_NAME}.map,--cref
-Wl,--no-warn-mismatch
-Wl,--library-path=${LINKER_SRC}
-Wl,--script=${LINKER_SCRIPT},--gc-sections
-Wl,--print-memory-usage
)

SET_TARGET_PROPERTIES(${EXECUTABLE} PROPERTIES LINKER_LANGUAGE CXX)

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE} ${PROJECT_NAME}.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin)