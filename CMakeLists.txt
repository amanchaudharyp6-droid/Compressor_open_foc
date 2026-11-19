#SETUP PROJECT:
    cmake_minimum_required(VERSION 4.0.0)

#Project Name
    project(Pegasus_v4
            VERSION 1.0.0
            DESCRIPTION "bruh"
    )


    set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY") #Statc library means it won't try to create an executable during tests

#Setting up the toolchain
    enable_language(C ASM)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CMAKE_C_EXTENSIONS OFF)
    
    set(MCU_OPTIONS
        -mcpu=cortex-m7
        -mthumb
    )

    execute_process (
        COMMAND git rev-parse --short HEAD
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_SHA
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

# ============================================================
#  Input Paths (Includes and Sources)
# ============================================================
#Include Directories
    set(COMMON_INCLUDE
        Core/Inc
        Core/Bootloader
        Core/Utilities
        Core/Libs/Include
        
        Drivers/CMSIS/Device/ST/STM32H7xx/Include
        Drivers/CMSIS/Include
        Drivers/STM32H7xx_HAL_Driver/Inc
        Drivers/STM32H7xx_HAL_Driver/Inc/Legacy

        Core/Utilities/can-tp-module-threadx
        Core/Utilities/can-tp-module-threadx/isotp-c/src
        # Core/Utilities/can-tp-module-threadx/isotp-c/src/isotp
        Core/Utilities/can-tp-module-threadx/isotp-c/deps/bitfield-c/src

    )

#Common Source Files
    set(COMMON_FILES

        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
        Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c

        Core/Src/stm32h7xx_hal_msp.c
        Core/Src/system_stm32h7xx.c
        Core/Src/stm32h7xx_it.c

        Core/Src/syscalls.c
        Core/Src/sysmem.c

        Core/Src/adc.c
        Core/Src/dma.c
        Core/Src/fdcan.c
        Core/Src/gpio.c
        Core/Src/tim.c
        Core/Src/i2c.c
        Core/Src/spi.c

        Core/Utilities/Flash.c

        Core/Startup/startup_stm32h743zitx.s
    )

    set(BOOTLOADER_FILES
        Core/Bootloader/BootloaderMain.c
        Core/Bootloader/BootloaderFunctions.c
    )

    set(FIRMWARE_MAIN
        Core/Src/main.c
    )

    set(FIRMWARE_FILES
        Core/Src/Ad2s1210.c            # AD2S1210 resolver-to-digital converter (SPI1)
        Core/Src/Resolver_Position.c   # Resolver position manager (SPI angle -> TIM2 encoder)
        Core/Src/Eeprom.c
        Core/Src/fdcan_AL.c   # firmware-only CAN app layer (FOC_Loop Send_* + queue); fdcan.c stays hardware-only in COMMON for the bootloader
        # --- FOC_Model_Based (FOC_Loop) architecture, merged from dev/PEG_v4/FOC_Model_Based ---
        Core/Src/FOC_Loop.c
        Core/Src/Current_Ref_Estimator.c
        Core/Src/can_rx_handlers.c
        Core/Src/isr_handlers.c
        Core/Src/tim17_handlers.c
        Core/Src/HelperFuncs.c
        Core/Src/Helper_Functions.c
        Core/Src/Limit_Errors.c
        Core/Src/LPF.c
        Core/Src/MCU_Protections.c
        Core/Src/MCU_Protections_data.c
        Core/Src/NTC_Temp_Reading.c
        Core/Src/Open_FOC0.c
        Core/Src/Peripheral_Init.c
        Core/Src/Rate_Limiter.c
        Core/Src/rt_nonfinite.c
        Core/Src/rtGetNaN.c
        Core/Src/Sensor_Disconnection_Checks.c
        Core/Src/Speed_Sensing.c
        Core/Src/throttle_map.c
        Core/Src/Torque_Management.c
        Core/Src/Hill_Hold.c

        Core/Utilities/can-tp-module-threadx/isotp-c/deps/bitfield-c/src/bitfield/8byte.c
        Core/Utilities/can-tp-module-threadx/isotp-c/deps/bitfield-c/src/bitfield/bitarray.c
        Core/Utilities/can-tp-module-threadx/isotp-c/deps/bitfield-c/src/bitfield/bitfield.c
        Core/Utilities/can-tp-module-threadx/isotp-c/src/isotp/isotp.c
        Core/Utilities/can-tp-module-threadx/isotp-c/src/isotp/receive.c
        Core/Utilities/can-tp-module-threadx/isotp-c/src/isotp/send.c
        Core/Utilities/can-tp-module-threadx/can_tp_app.c

        Core/Utilities/firmware_upgrade_app.c

        Core/Utilities/crc32.c
    )

    
# ============================================================
#  Setting Outputs (Executables)
# ============================================================

    set(EXECUTABLE_BOOTLOADER ${PROJECT_NAME}-Bootloader.elf)
    set(EXECUTABLE_APP1 ${PROJECT_NAME}-App1.elf)
    set(EXECUTABLE_APP2 ${PROJECT_NAME}-App2.elf)

#Assigning Executable Targets:
    add_executable(${EXECUTABLE_BOOTLOADER}
                    ${COMMON_FILES}
                    ${BOOTLOADER_FILES}
                    )
    add_executable(${EXECUTABLE_APP1}
                    ${COMMON_FILES}
                    ${FIRMWARE_MAIN}
                    ${FIRMWARE_FILES})
    add_executable(${EXECUTABLE_APP2}
                    ${COMMON_FILES}
                    ${FIRMWARE_MAIN}
                    ${FIRMWARE_FILES})

# ============================================================
#  Compiler & Preprocessor Definitions
# ============================================================
#

    target_compile_definitions(${EXECUTABLE_BOOTLOADER} PRIVATE
        -DUSE_HAL_DRIVER
        -DSTM32H743xx
        -DGIT_SHA=0x${GIT_SHA}U
        -DENABLE_DEBUG=${ENABLE_DEBUG}
    )
    target_compile_definitions(${EXECUTABLE_APP1} PRIVATE
        -DUSE_HAL_DRIVER
        -DSTM32H743xx
        -DGIT_SHA=0x${GIT_SHA}U
        -DCURR_APP1
        -DENABLE_DEBUG=${ENABLE_DEBUG}
    )
    target_compile_definitions(${EXECUTABLE_APP2} PRIVATE
        -DUSE_HAL_DRIVER
        -DSTM32H743xx
        -DGIT_SHA=0x${GIT_SHA}U
        -DCURR_APP2
        -DENABLE_DEBUG=${ENABLE_DEBUG}  
    )

# ============================================================
#  Linkin the Static Library
# ============================================================
    target_link_libraries(${EXECUTABLE_APP1} PRIVATE ${CMAKE_SOURCE_DIR}/Core/Libs/libAngle_Offset_101.a)
    target_link_libraries(${EXECUTABLE_APP2} PRIVATE ${CMAKE_SOURCE_DIR}/Core/Libs/libAngle_Offset_101.a)

# ============================================================
#  Compiler & Preprocessor Definitions
# ============================================================
#Here we add the include directories to the target and 

    target_include_directories(${EXECUTABLE_BOOTLOADER} PUBLIC ${COMMON_INCLUDE})
    target_include_directories(${EXECUTABLE_APP1} PUBLIC ${COMMON_INCLUDE})
    target_include_directories(${EXECUTABLE_APP2} PUBLIC ${COMMON_INCLUDE})

        target_compile_options(${EXECUTABLE_BOOTLOADER} PRIVATE
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard

        -flto
        -fstack-usage
        -fdata-sections
        -ffunction-sections
        # -fsingle-precision-constant

        # -std=gnu11
        -Wall
        -Wextra
        -Wpedantic
        # -Wundef
        # -Wshadow
        # -Wdouble-promotion

        $<$<CONFIG:Debug>:-O1>
        )

        target_compile_options(${EXECUTABLE_APP1} PRIVATE
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard

        -fstack-usage
        -fdata-sections
        -ffunction-sections
        -fsingle-precision-constant

        # -std=gnu11
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-unused-variable
        # -Wpedantic
        # -Wundef
        # -Wshadow
        -Wdouble-promotion
        # -Wno-discarded-qualifiers

        $<$<CONFIG:Debug>:-Og>
        )

        target_compile_options(${EXECUTABLE_APP2} PRIVATE
        -mcpu=cortex-m7
        -mthumb
        -mfpu=fpv5-d16
        -mfloat-abi=hard

        -fstack-usage
        -fdata-sections
        -ffunction-sections
        -fsingle-precision-constant
        # -fno-builtin

        # -std=gnu11
        -Wall
        -Wextra
        -Wno-unused-parameter
        -Wno-unused-variable
        # -Wpedantic
        # -Wundef
        # -Wshadow
        -Wdouble-promotion
        # -Wno-discarded-qualifiers

        $<$<CONFIG:Debug>:-Og>
    )

# ============================================================
#  Linker Definitions
# ============================================================ 
    target_link_options(${EXECUTABLE_BOOTLOADER} PRIVATE
        # -T${CMAKE_SOURCE_DIR}/STM32H743ZITX_FLASH.ld
        -T${CMAKE_SOURCE_DIR}/LinkerScripts/MemoryMap.ld
        -T${CMAKE_SOURCE_DIR}/LinkerScripts/Bootloader.ld
        -mcpu=cortex-m7 
        -mthumb 
        -mfpu=fpv5-d16 
        -mfloat-abi=hard 

        -flto
        --specs=nano.specs 
        -Wl,--Map=${PROJECT_NAME}-bootloader.map,--cref 
        -Wl,--gc-sections 
        # -static  

        # --specs=nosys.specs 
        -Wl,--print-memory-usage

        -Wl,--start-group 
        -lc -lm #-lnosys
        -Wl,--end-group 
    )

    target_link_options(${EXECUTABLE_APP1} PRIVATE
        # -T${CMAKE_SOURCE_DIR}/STM32H743ZITX_FLASH.ld
        -T${CMAKE_SOURCE_DIR}/LinkerScripts/MemoryMap.ld
        -T${CMAKE_SOURCE_DIR}/LinkerScripts/App1.ld
        -mcpu=cortex-m7 
        -mthumb 
        -mfpu=fpv5-d16 
        -mfloat-abi=hard 

        --specs=nano.specs 
        -Wl,--Map=${PROJECT_NAME}-App1.map,--cref 
        -Wl,--gc-sections 
        # -static  

        # --specs=nosys.specs 
        -Wl,--print-memory-usage

        -Wl,--start-group 
        -lc -lm #-lnosys
        -Wl,--end-group 
    )

    target_link_options(${EXECUTABLE_APP2} PRIVATE
        # -T${CMAKE_SOURCE_DIR}/STM32H743ZITX_FLASH.ld
        -T${CMAKE_SOURCE_DIR}/LinkerScripts/MemoryMap.ld
        -T${CMAKE_SOURCE_DIR}/LinkerScripts/App2.ld
        -mcpu=cortex-m7 
        -mthumb 
        -mfpu=fpv5-d16 
        -mfloat-abi=hard 

        --specs=nano.specs 
        -Wl,--Map=${PROJECT_NAME}-App2.map,--cref 
        -Wl,--gc-sections 
        # -static  

        # --specs=nosys.specs 
        -Wl,--print-memory-usage

        -Wl,--start-group 
        -lc -lm #-lnosys
        -Wl,--end-group 
    )  

# ============================================================
#  Post-Build Commands (Size + HEX/BIN generation)
# ============================================================

    add_custom_command(TARGET ${EXECUTABLE_BOOTLOADER}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE_BOOTLOADER}
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE_BOOTLOADER} ${PROJECT_NAME}-Bootloader.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE_BOOTLOADER} ${PROJECT_NAME}-Bootloader.bin
    )

    add_custom_command(TARGET ${EXECUTABLE_APP1}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE_APP1}
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE_APP1} ${PROJECT_NAME}-App1.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE_APP1} ${PROJECT_NAME}-App1.bin
    )

    add_custom_command(TARGET ${EXECUTABLE_APP2}
        POST_BUILD
        COMMAND arm-none-eabi-size ${EXECUTABLE_APP2}
        COMMAND arm-none-eabi-objcopy -O ihex ${EXECUTABLE_APP2} ${PROJECT_NAME}-App2.hex
        COMMAND arm-none-eabi-objcopy -O binary ${EXECUTABLE_APP2} ${PROJECT_NAME}-App2.bin
    )