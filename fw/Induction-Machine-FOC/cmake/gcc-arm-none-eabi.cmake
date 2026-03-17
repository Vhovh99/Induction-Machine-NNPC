set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Locate the arm-none-eabi toolchain from the STM32CubeIDE bundle (or PATH).
if(DEFINED ENV{STM32_TOOLCHAIN_PATH})
    set(TOOLCHAIN_BIN_DIR "$ENV{STM32_TOOLCHAIN_PATH}/bin/")
elseif(EXISTS "$ENV{HOME}/.local/share/stm32cube/bundles/gnu-tools-for-stm32")
    file(GLOB _GCC_BUNDLES "$ENV{HOME}/.local/share/stm32cube/bundles/gnu-tools-for-stm32/*")
    list(SORT _GCC_BUNDLES ORDER DESCENDING)
    list(GET _GCC_BUNDLES 0 _GCC_BUNDLE_DIR)
    set(TOOLCHAIN_BIN_DIR "${_GCC_BUNDLE_DIR}/bin/")
else()
    set(TOOLCHAIN_BIN_DIR "")
endif()

set(TOOLCHAIN_PREFIX                arm-none-eabi-)
set(TOOLCHAIN_FULL_PREFIX           ${TOOLCHAIN_BIN_DIR}${TOOLCHAIN_PREFIX})

set(CMAKE_C_COMPILER                ${TOOLCHAIN_FULL_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_FULL_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_FULL_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_FULL_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_FULL_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32G474XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs -u _printf_float")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
