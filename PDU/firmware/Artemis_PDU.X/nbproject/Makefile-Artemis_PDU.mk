#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-Artemis_PDU.mk)" "nbproject/Makefile-local-Artemis_PDU.mk"
include nbproject/Makefile-local-Artemis_PDU.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=Artemis_PDU
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/Artemis_PDU.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/Artemis_PDU.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi.c ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_file_system.c ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_driver_interface.c ../src/config/Artemis_PDU/driver/spi/src/drv_spi.c ../src/config/Artemis_PDU/osal/osal_freertos.c ../src/config/Artemis_PDU/peripheral/clock/plib_clock.c ../src/config/Artemis_PDU/peripheral/cmcc/plib_cmcc.c ../src/config/Artemis_PDU/peripheral/evsys/plib_evsys.c ../src/config/Artemis_PDU/peripheral/nvic/plib_nvic.c ../src/config/Artemis_PDU/peripheral/nvmctrl/plib_nvmctrl.c ../src/config/Artemis_PDU/peripheral/port/plib_port.c ../src/config/Artemis_PDU/peripheral/rtc/plib_rtc_timer.c ../src/config/Artemis_PDU/peripheral/sercom/i2c_slave/plib_sercom4_i2c_slave.c ../src/config/Artemis_PDU/peripheral/sercom/spi_master/plib_sercom2_spi_master.c ../src/config/Artemis_PDU/peripheral/sercom/usart/plib_sercom3_usart.c ../src/config/Artemis_PDU/stdio/xc32_monitor.c ../src/config/Artemis_PDU/system/cache/sys_cache.c ../src/config/Artemis_PDU/system/dma/sys_dma.c ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ff.c ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ffunicode.c ../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access/diskio.c ../src/config/Artemis_PDU/system/fs/src/sys_fs.c ../src/config/Artemis_PDU/system/fs/src/sys_fs_media_manager.c ../src/config/Artemis_PDU/system/fs/src/sys_fs_fat_interface.c ../src/config/Artemis_PDU/system/int/src/sys_int.c ../src/config/Artemis_PDU/system/time/src/sys_time.c ../src/config/Artemis_PDU/initialization.c ../src/config/Artemis_PDU/interrupts.c ../src/config/Artemis_PDU/exceptions.c ../src/config/Artemis_PDU/startup_xc32.c ../src/config/Artemis_PDU/libc_syscalls.c ../src/config/Artemis_PDU/freertos_hooks.c ../src/config/Artemis_PDU/tasks.c ../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F/port.c ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../src/third_party/rtos/FreeRTOS/Source/croutine.c ../src/third_party/rtos/FreeRTOS/Source/list.c ../src/third_party/rtos/FreeRTOS/Source/queue.c ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c ../src/third_party/rtos/FreeRTOS/Source/timers.c ../src/third_party/rtos/FreeRTOS/Source/event_groups.c ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../src/main.c ../src/app.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1760076532/drv_sdspi.o ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o ${OBJECTDIR}/_ext/1471363227/drv_spi.o ${OBJECTDIR}/_ext/1624425958/osal_freertos.o ${OBJECTDIR}/_ext/1650922938/plib_clock.o ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o ${OBJECTDIR}/_ext/1648773452/plib_evsys.o ${OBJECTDIR}/_ext/1022749782/plib_nvic.o ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o ${OBJECTDIR}/_ext/1022696631/plib_port.o ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o ${OBJECTDIR}/_ext/177662547/sys_cache.o ${OBJECTDIR}/_ext/1148418531/sys_dma.o ${OBJECTDIR}/_ext/401857885/ff.o ${OBJECTDIR}/_ext/401857885/ffunicode.o ${OBJECTDIR}/_ext/867183764/diskio.o ${OBJECTDIR}/_ext/1111598889/sys_fs.o ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o ${OBJECTDIR}/_ext/1813730513/sys_int.o ${OBJECTDIR}/_ext/1304864151/sys_time.o ${OBJECTDIR}/_ext/974488028/initialization.o ${OBJECTDIR}/_ext/974488028/interrupts.o ${OBJECTDIR}/_ext/974488028/exceptions.o ${OBJECTDIR}/_ext/974488028/startup_xc32.o ${OBJECTDIR}/_ext/974488028/libc_syscalls.o ${OBJECTDIR}/_ext/974488028/freertos_hooks.o ${OBJECTDIR}/_ext/974488028/tasks.o ${OBJECTDIR}/_ext/246609638/port.o ${OBJECTDIR}/_ext/1665200909/heap_1.o ${OBJECTDIR}/_ext/404212886/croutine.o ${OBJECTDIR}/_ext/404212886/list.o ${OBJECTDIR}/_ext/404212886/queue.o ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o ${OBJECTDIR}/_ext/404212886/timers.o ${OBJECTDIR}/_ext/404212886/event_groups.o ${OBJECTDIR}/_ext/404212886/stream_buffer.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1760076532/drv_sdspi.o.d ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o.d ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o.d ${OBJECTDIR}/_ext/1471363227/drv_spi.o.d ${OBJECTDIR}/_ext/1624425958/osal_freertos.o.d ${OBJECTDIR}/_ext/1650922938/plib_clock.o.d ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o.d ${OBJECTDIR}/_ext/1648773452/plib_evsys.o.d ${OBJECTDIR}/_ext/1022749782/plib_nvic.o.d ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o.d ${OBJECTDIR}/_ext/1022696631/plib_port.o.d ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o.d ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o.d ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o.d ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o.d ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o.d ${OBJECTDIR}/_ext/177662547/sys_cache.o.d ${OBJECTDIR}/_ext/1148418531/sys_dma.o.d ${OBJECTDIR}/_ext/401857885/ff.o.d ${OBJECTDIR}/_ext/401857885/ffunicode.o.d ${OBJECTDIR}/_ext/867183764/diskio.o.d ${OBJECTDIR}/_ext/1111598889/sys_fs.o.d ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o.d ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o.d ${OBJECTDIR}/_ext/1813730513/sys_int.o.d ${OBJECTDIR}/_ext/1304864151/sys_time.o.d ${OBJECTDIR}/_ext/974488028/initialization.o.d ${OBJECTDIR}/_ext/974488028/interrupts.o.d ${OBJECTDIR}/_ext/974488028/exceptions.o.d ${OBJECTDIR}/_ext/974488028/startup_xc32.o.d ${OBJECTDIR}/_ext/974488028/libc_syscalls.o.d ${OBJECTDIR}/_ext/974488028/freertos_hooks.o.d ${OBJECTDIR}/_ext/974488028/tasks.o.d ${OBJECTDIR}/_ext/246609638/port.o.d ${OBJECTDIR}/_ext/1665200909/heap_1.o.d ${OBJECTDIR}/_ext/404212886/croutine.o.d ${OBJECTDIR}/_ext/404212886/list.o.d ${OBJECTDIR}/_ext/404212886/queue.o.d ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d ${OBJECTDIR}/_ext/404212886/timers.o.d ${OBJECTDIR}/_ext/404212886/event_groups.o.d ${OBJECTDIR}/_ext/404212886/stream_buffer.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1760076532/drv_sdspi.o ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o ${OBJECTDIR}/_ext/1471363227/drv_spi.o ${OBJECTDIR}/_ext/1624425958/osal_freertos.o ${OBJECTDIR}/_ext/1650922938/plib_clock.o ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o ${OBJECTDIR}/_ext/1648773452/plib_evsys.o ${OBJECTDIR}/_ext/1022749782/plib_nvic.o ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o ${OBJECTDIR}/_ext/1022696631/plib_port.o ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o ${OBJECTDIR}/_ext/177662547/sys_cache.o ${OBJECTDIR}/_ext/1148418531/sys_dma.o ${OBJECTDIR}/_ext/401857885/ff.o ${OBJECTDIR}/_ext/401857885/ffunicode.o ${OBJECTDIR}/_ext/867183764/diskio.o ${OBJECTDIR}/_ext/1111598889/sys_fs.o ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o ${OBJECTDIR}/_ext/1813730513/sys_int.o ${OBJECTDIR}/_ext/1304864151/sys_time.o ${OBJECTDIR}/_ext/974488028/initialization.o ${OBJECTDIR}/_ext/974488028/interrupts.o ${OBJECTDIR}/_ext/974488028/exceptions.o ${OBJECTDIR}/_ext/974488028/startup_xc32.o ${OBJECTDIR}/_ext/974488028/libc_syscalls.o ${OBJECTDIR}/_ext/974488028/freertos_hooks.o ${OBJECTDIR}/_ext/974488028/tasks.o ${OBJECTDIR}/_ext/246609638/port.o ${OBJECTDIR}/_ext/1665200909/heap_1.o ${OBJECTDIR}/_ext/404212886/croutine.o ${OBJECTDIR}/_ext/404212886/list.o ${OBJECTDIR}/_ext/404212886/queue.o ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o ${OBJECTDIR}/_ext/404212886/timers.o ${OBJECTDIR}/_ext/404212886/event_groups.o ${OBJECTDIR}/_ext/404212886/stream_buffer.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app.o

# Source Files
SOURCEFILES=../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi.c ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_file_system.c ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_driver_interface.c ../src/config/Artemis_PDU/driver/spi/src/drv_spi.c ../src/config/Artemis_PDU/osal/osal_freertos.c ../src/config/Artemis_PDU/peripheral/clock/plib_clock.c ../src/config/Artemis_PDU/peripheral/cmcc/plib_cmcc.c ../src/config/Artemis_PDU/peripheral/evsys/plib_evsys.c ../src/config/Artemis_PDU/peripheral/nvic/plib_nvic.c ../src/config/Artemis_PDU/peripheral/nvmctrl/plib_nvmctrl.c ../src/config/Artemis_PDU/peripheral/port/plib_port.c ../src/config/Artemis_PDU/peripheral/rtc/plib_rtc_timer.c ../src/config/Artemis_PDU/peripheral/sercom/i2c_slave/plib_sercom4_i2c_slave.c ../src/config/Artemis_PDU/peripheral/sercom/spi_master/plib_sercom2_spi_master.c ../src/config/Artemis_PDU/peripheral/sercom/usart/plib_sercom3_usart.c ../src/config/Artemis_PDU/stdio/xc32_monitor.c ../src/config/Artemis_PDU/system/cache/sys_cache.c ../src/config/Artemis_PDU/system/dma/sys_dma.c ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ff.c ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ffunicode.c ../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access/diskio.c ../src/config/Artemis_PDU/system/fs/src/sys_fs.c ../src/config/Artemis_PDU/system/fs/src/sys_fs_media_manager.c ../src/config/Artemis_PDU/system/fs/src/sys_fs_fat_interface.c ../src/config/Artemis_PDU/system/int/src/sys_int.c ../src/config/Artemis_PDU/system/time/src/sys_time.c ../src/config/Artemis_PDU/initialization.c ../src/config/Artemis_PDU/interrupts.c ../src/config/Artemis_PDU/exceptions.c ../src/config/Artemis_PDU/startup_xc32.c ../src/config/Artemis_PDU/libc_syscalls.c ../src/config/Artemis_PDU/freertos_hooks.c ../src/config/Artemis_PDU/tasks.c ../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F/port.c ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c ../src/third_party/rtos/FreeRTOS/Source/croutine.c ../src/third_party/rtos/FreeRTOS/Source/list.c ../src/third_party/rtos/FreeRTOS/Source/queue.c ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c ../src/third_party/rtos/FreeRTOS/Source/timers.c ../src/third_party/rtos/FreeRTOS/Source/event_groups.c ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c ../src/main.c ../src/app.c

# Pack Options 
PACK_COMMON_OPTIONS=-I "${CMSIS_DIR}/CMSIS/Core/Include"



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-Artemis_PDU.mk ${DISTDIR}/Artemis_PDU.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=ATSAME51N19A
MP_LINKER_FILE_OPTION=,--script="..\src\config\Artemis_PDU\ATSAME51N19A.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1760076532/drv_sdspi.o: ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi.c  .generated_files/flags/Artemis_PDU/3fe31c9c2ef49435b069d049075e2d90ad25c498 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1760076532" 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1760076532/drv_sdspi.o.d" -o ${OBJECTDIR}/_ext/1760076532/drv_sdspi.o ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o: ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_file_system.c  .generated_files/flags/Artemis_PDU/6aab9808c72a6508d44fe46a669eaaf28fcce2f8 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1760076532" 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o.d 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o.d" -o ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_file_system.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o: ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_driver_interface.c  .generated_files/flags/Artemis_PDU/ecba32f483a42dcfdc09f4eeea36175414472ba1 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1760076532" 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o.d 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o.d" -o ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_driver_interface.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1471363227/drv_spi.o: ../src/config/Artemis_PDU/driver/spi/src/drv_spi.c  .generated_files/flags/Artemis_PDU/d97c607798a59de69346ce481c662126a90fe1ec .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1471363227" 
	@${RM} ${OBJECTDIR}/_ext/1471363227/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1471363227/drv_spi.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1471363227/drv_spi.o.d" -o ${OBJECTDIR}/_ext/1471363227/drv_spi.o ../src/config/Artemis_PDU/driver/spi/src/drv_spi.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1624425958/osal_freertos.o: ../src/config/Artemis_PDU/osal/osal_freertos.c  .generated_files/flags/Artemis_PDU/603deb4e09691a4bf1f45224d6c222773ac04f22 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1624425958" 
	@${RM} ${OBJECTDIR}/_ext/1624425958/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1624425958/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1624425958/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1624425958/osal_freertos.o ../src/config/Artemis_PDU/osal/osal_freertos.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1650922938/plib_clock.o: ../src/config/Artemis_PDU/peripheral/clock/plib_clock.c  .generated_files/flags/Artemis_PDU/8152f0c2636aed54ad0c873c79d5059a01d4f320 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1650922938" 
	@${RM} ${OBJECTDIR}/_ext/1650922938/plib_clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650922938/plib_clock.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1650922938/plib_clock.o.d" -o ${OBJECTDIR}/_ext/1650922938/plib_clock.o ../src/config/Artemis_PDU/peripheral/clock/plib_clock.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1023086318/plib_cmcc.o: ../src/config/Artemis_PDU/peripheral/cmcc/plib_cmcc.c  .generated_files/flags/Artemis_PDU/61feeba69eba54b0dd0b276e67bad2243d5b677d .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1023086318" 
	@${RM} ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1023086318/plib_cmcc.o.d" -o ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o ../src/config/Artemis_PDU/peripheral/cmcc/plib_cmcc.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1648773452/plib_evsys.o: ../src/config/Artemis_PDU/peripheral/evsys/plib_evsys.c  .generated_files/flags/Artemis_PDU/b4df4d70e0a75bf110f76b139584e6514d2eb3a9 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1648773452" 
	@${RM} ${OBJECTDIR}/_ext/1648773452/plib_evsys.o.d 
	@${RM} ${OBJECTDIR}/_ext/1648773452/plib_evsys.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1648773452/plib_evsys.o.d" -o ${OBJECTDIR}/_ext/1648773452/plib_evsys.o ../src/config/Artemis_PDU/peripheral/evsys/plib_evsys.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1022749782/plib_nvic.o: ../src/config/Artemis_PDU/peripheral/nvic/plib_nvic.c  .generated_files/flags/Artemis_PDU/1c6bdfc93e50eb9e3bc15563da36b53e59208ba0 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1022749782" 
	@${RM} ${OBJECTDIR}/_ext/1022749782/plib_nvic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1022749782/plib_nvic.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1022749782/plib_nvic.o.d" -o ${OBJECTDIR}/_ext/1022749782/plib_nvic.o ../src/config/Artemis_PDU/peripheral/nvic/plib_nvic.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o: ../src/config/Artemis_PDU/peripheral/nvmctrl/plib_nvmctrl.c  .generated_files/flags/Artemis_PDU/f09152c3c3f223546e268a884f3fffee0b0fa9b3 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/236948536" 
	@${RM} ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o.d 
	@${RM} ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o.d" -o ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o ../src/config/Artemis_PDU/peripheral/nvmctrl/plib_nvmctrl.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1022696631/plib_port.o: ../src/config/Artemis_PDU/peripheral/port/plib_port.c  .generated_files/flags/Artemis_PDU/8a8d50094d7bfe413ac2a6074cf9c42d79126da8 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1022696631" 
	@${RM} ${OBJECTDIR}/_ext/1022696631/plib_port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1022696631/plib_port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1022696631/plib_port.o.d" -o ${OBJECTDIR}/_ext/1022696631/plib_port.o ../src/config/Artemis_PDU/peripheral/port/plib_port.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o: ../src/config/Artemis_PDU/peripheral/rtc/plib_rtc_timer.c  .generated_files/flags/Artemis_PDU/3c8c5835626547efc2fdec0ee884ea9dd034273 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/659748505" 
	@${RM} ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o.d" -o ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o ../src/config/Artemis_PDU/peripheral/rtc/plib_rtc_timer.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o: ../src/config/Artemis_PDU/peripheral/sercom/i2c_slave/plib_sercom4_i2c_slave.c  .generated_files/flags/Artemis_PDU/c2dbf42df20eff5fc30289d18e5a39ac149599f4 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/835284660" 
	@${RM} ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o.d" -o ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o ../src/config/Artemis_PDU/peripheral/sercom/i2c_slave/plib_sercom4_i2c_slave.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o: ../src/config/Artemis_PDU/peripheral/sercom/spi_master/plib_sercom2_spi_master.c  .generated_files/flags/Artemis_PDU/7c4c0739837b236ba4a3815480e86e58efd3dacf .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/796654533" 
	@${RM} ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o.d" -o ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o ../src/config/Artemis_PDU/peripheral/sercom/spi_master/plib_sercom2_spi_master.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o: ../src/config/Artemis_PDU/peripheral/sercom/usart/plib_sercom3_usart.c  .generated_files/flags/Artemis_PDU/501c25a6c298186dfa96e2a1ca59fb8fd3836485 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1679430495" 
	@${RM} ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o.d" -o ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o ../src/config/Artemis_PDU/peripheral/sercom/usart/plib_sercom3_usart.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1186129630/xc32_monitor.o: ../src/config/Artemis_PDU/stdio/xc32_monitor.c  .generated_files/flags/Artemis_PDU/e1432223b264b8efb088a3c46d471290446a4c7e .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1186129630" 
	@${RM} ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1186129630/xc32_monitor.o.d" -o ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o ../src/config/Artemis_PDU/stdio/xc32_monitor.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/177662547/sys_cache.o: ../src/config/Artemis_PDU/system/cache/sys_cache.c  .generated_files/flags/Artemis_PDU/ca9ab698b82855c9e6d17aba87ae40b2014889b0 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/177662547" 
	@${RM} ${OBJECTDIR}/_ext/177662547/sys_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/177662547/sys_cache.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/177662547/sys_cache.o.d" -o ${OBJECTDIR}/_ext/177662547/sys_cache.o ../src/config/Artemis_PDU/system/cache/sys_cache.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1148418531/sys_dma.o: ../src/config/Artemis_PDU/system/dma/sys_dma.c  .generated_files/flags/Artemis_PDU/b2c4b2d0fa91a5131554da551d002df9a7af40fc .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1148418531" 
	@${RM} ${OBJECTDIR}/_ext/1148418531/sys_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/1148418531/sys_dma.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1148418531/sys_dma.o.d" -o ${OBJECTDIR}/_ext/1148418531/sys_dma.o ../src/config/Artemis_PDU/system/dma/sys_dma.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/401857885/ff.o: ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ff.c  .generated_files/flags/Artemis_PDU/e20636f5d3d8876a0616f9ded51d5fda62c784e1 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/401857885" 
	@${RM} ${OBJECTDIR}/_ext/401857885/ff.o.d 
	@${RM} ${OBJECTDIR}/_ext/401857885/ff.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/401857885/ff.o.d" -o ${OBJECTDIR}/_ext/401857885/ff.o ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ff.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/401857885/ffunicode.o: ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ffunicode.c  .generated_files/flags/Artemis_PDU/2e459e0a576a19c599fc7c6a94eff951e42ea415 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/401857885" 
	@${RM} ${OBJECTDIR}/_ext/401857885/ffunicode.o.d 
	@${RM} ${OBJECTDIR}/_ext/401857885/ffunicode.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/401857885/ffunicode.o.d" -o ${OBJECTDIR}/_ext/401857885/ffunicode.o ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ffunicode.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/867183764/diskio.o: ../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access/diskio.c  .generated_files/flags/Artemis_PDU/8982876e5743150db53558c808b98043ed9d08e4 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/867183764" 
	@${RM} ${OBJECTDIR}/_ext/867183764/diskio.o.d 
	@${RM} ${OBJECTDIR}/_ext/867183764/diskio.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/867183764/diskio.o.d" -o ${OBJECTDIR}/_ext/867183764/diskio.o ../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access/diskio.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1111598889/sys_fs.o: ../src/config/Artemis_PDU/system/fs/src/sys_fs.c  .generated_files/flags/Artemis_PDU/31a1233597d76354be24f6a2ae5e48b6fd37b8ae .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1111598889" 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1111598889/sys_fs.o.d" -o ${OBJECTDIR}/_ext/1111598889/sys_fs.o ../src/config/Artemis_PDU/system/fs/src/sys_fs.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o: ../src/config/Artemis_PDU/system/fs/src/sys_fs_media_manager.c  .generated_files/flags/Artemis_PDU/306163b998d30a5c6e9772fd754097a0b156d24c .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1111598889" 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o.d" -o ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o ../src/config/Artemis_PDU/system/fs/src/sys_fs_media_manager.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o: ../src/config/Artemis_PDU/system/fs/src/sys_fs_fat_interface.c  .generated_files/flags/Artemis_PDU/3e437dafeed9bf82ffa67bb433d2cacaa211d698 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1111598889" 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o.d 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o.d" -o ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o ../src/config/Artemis_PDU/system/fs/src/sys_fs_fat_interface.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1813730513/sys_int.o: ../src/config/Artemis_PDU/system/int/src/sys_int.c  .generated_files/flags/Artemis_PDU/2bae4d7bb3b2d28129d63ef42e3d1710d3f6d59f .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1813730513" 
	@${RM} ${OBJECTDIR}/_ext/1813730513/sys_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/1813730513/sys_int.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1813730513/sys_int.o.d" -o ${OBJECTDIR}/_ext/1813730513/sys_int.o ../src/config/Artemis_PDU/system/int/src/sys_int.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1304864151/sys_time.o: ../src/config/Artemis_PDU/system/time/src/sys_time.c  .generated_files/flags/Artemis_PDU/57accab6a39a97199afc4f454c23a1723f8ec73c .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1304864151" 
	@${RM} ${OBJECTDIR}/_ext/1304864151/sys_time.o.d 
	@${RM} ${OBJECTDIR}/_ext/1304864151/sys_time.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1304864151/sys_time.o.d" -o ${OBJECTDIR}/_ext/1304864151/sys_time.o ../src/config/Artemis_PDU/system/time/src/sys_time.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/initialization.o: ../src/config/Artemis_PDU/initialization.c  .generated_files/flags/Artemis_PDU/dfbeec536ac9bbfc01cab744c7e6f39d17607016 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/initialization.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/initialization.o.d" -o ${OBJECTDIR}/_ext/974488028/initialization.o ../src/config/Artemis_PDU/initialization.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/interrupts.o: ../src/config/Artemis_PDU/interrupts.c  .generated_files/flags/Artemis_PDU/309c6128f0cfd5b53db567aa3d756cb888fef67d .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/interrupts.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/interrupts.o.d" -o ${OBJECTDIR}/_ext/974488028/interrupts.o ../src/config/Artemis_PDU/interrupts.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/exceptions.o: ../src/config/Artemis_PDU/exceptions.c  .generated_files/flags/Artemis_PDU/a9c57755c0b7cfc25d2cb403a6c1bc7df8fd94e1 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/exceptions.o.d" -o ${OBJECTDIR}/_ext/974488028/exceptions.o ../src/config/Artemis_PDU/exceptions.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/startup_xc32.o: ../src/config/Artemis_PDU/startup_xc32.c  .generated_files/flags/Artemis_PDU/d580ea588a2018bbd78b6f8c5a8f5c223433474c .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/startup_xc32.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/startup_xc32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/startup_xc32.o.d" -o ${OBJECTDIR}/_ext/974488028/startup_xc32.o ../src/config/Artemis_PDU/startup_xc32.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/libc_syscalls.o: ../src/config/Artemis_PDU/libc_syscalls.c  .generated_files/flags/Artemis_PDU/fb22d2798a7055aec60366b023005457fb512e7b .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/libc_syscalls.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/libc_syscalls.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/libc_syscalls.o.d" -o ${OBJECTDIR}/_ext/974488028/libc_syscalls.o ../src/config/Artemis_PDU/libc_syscalls.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/freertos_hooks.o: ../src/config/Artemis_PDU/freertos_hooks.c  .generated_files/flags/Artemis_PDU/5199fe4f8da196aeb8f7b076680eb9c8a362dbf .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/freertos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/freertos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/freertos_hooks.o.d" -o ${OBJECTDIR}/_ext/974488028/freertos_hooks.o ../src/config/Artemis_PDU/freertos_hooks.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/tasks.o: ../src/config/Artemis_PDU/tasks.c  .generated_files/flags/Artemis_PDU/6a813c5ef8a66ebafa9f22f0d7e6c9fc142adb4a .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/tasks.o.d" -o ${OBJECTDIR}/_ext/974488028/tasks.o ../src/config/Artemis_PDU/tasks.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/246609638/port.o: ../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F/port.c  .generated_files/flags/Artemis_PDU/56116046d959c5c2347ee0075652bffa690cf6b8 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/246609638" 
	@${RM} ${OBJECTDIR}/_ext/246609638/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/246609638/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/246609638/port.o.d" -o ${OBJECTDIR}/_ext/246609638/port.o ../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F/port.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1665200909/heap_1.o: ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  .generated_files/flags/Artemis_PDU/2456b232494fda7a39e03301d094731d51891199 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1665200909" 
	@${RM} ${OBJECTDIR}/_ext/1665200909/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1665200909/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1665200909/heap_1.o.d" -o ${OBJECTDIR}/_ext/1665200909/heap_1.o ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/croutine.o: ../src/third_party/rtos/FreeRTOS/Source/croutine.c  .generated_files/flags/Artemis_PDU/e36ded615adce6af79ae0a27e0a36d9e1688ae37 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/croutine.o.d" -o ${OBJECTDIR}/_ext/404212886/croutine.o ../src/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/list.o: ../src/third_party/rtos/FreeRTOS/Source/list.c  .generated_files/flags/Artemis_PDU/cbfe7d31807e6ee5d379db3af73c35b0e4ed1aa6 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/list.o.d" -o ${OBJECTDIR}/_ext/404212886/list.o ../src/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/queue.o: ../src/third_party/rtos/FreeRTOS/Source/queue.c  .generated_files/flags/Artemis_PDU/38f265844a158c36a37f14864368275bd186362a .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/queue.o.d" -o ${OBJECTDIR}/_ext/404212886/queue.o ../src/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o: ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c  .generated_files/flags/Artemis_PDU/fd11b7c3f1b20a3f558e4ef623435fe35c3bbaed .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d" -o ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/timers.o: ../src/third_party/rtos/FreeRTOS/Source/timers.c  .generated_files/flags/Artemis_PDU/919451798410aee36218b531a65f86da655fc006 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/timers.o.d" -o ${OBJECTDIR}/_ext/404212886/timers.o ../src/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/event_groups.o: ../src/third_party/rtos/FreeRTOS/Source/event_groups.c  .generated_files/flags/Artemis_PDU/cde1d017042666b4613dfa9c506eca78c951d1cd .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/event_groups.o.d" -o ${OBJECTDIR}/_ext/404212886/event_groups.o ../src/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/stream_buffer.o: ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c  .generated_files/flags/Artemis_PDU/3dc317779aaa1c267932c9aec9f37df9608e9d96 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/404212886/stream_buffer.o ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/flags/Artemis_PDU/559a1c22f60346dd091a7584bccb66e124f399a4 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  .generated_files/flags/Artemis_PDU/3a155479c9da51cebdbdc6358ca24826efa8b456 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
else
${OBJECTDIR}/_ext/1760076532/drv_sdspi.o: ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi.c  .generated_files/flags/Artemis_PDU/526ec9958eca1c1641cb9db636fa6d9c6bcfde7f .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1760076532" 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1760076532/drv_sdspi.o.d" -o ${OBJECTDIR}/_ext/1760076532/drv_sdspi.o ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o: ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_file_system.c  .generated_files/flags/Artemis_PDU/8fd0eb1697b31f067e50f02dcd708d66a55e8535 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1760076532" 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o.d 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o.d" -o ${OBJECTDIR}/_ext/1760076532/drv_sdspi_file_system.o ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_file_system.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o: ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_driver_interface.c  .generated_files/flags/Artemis_PDU/c16f041889436d4f973332055df53a130519b87d .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1760076532" 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o.d 
	@${RM} ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o.d" -o ${OBJECTDIR}/_ext/1760076532/drv_sdspi_driver_interface.o ../src/config/Artemis_PDU/driver/sdspi/src/drv_sdspi_driver_interface.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1471363227/drv_spi.o: ../src/config/Artemis_PDU/driver/spi/src/drv_spi.c  .generated_files/flags/Artemis_PDU/ff124a0c2ea4fb8e5f0e1211651b0267ebf2914f .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1471363227" 
	@${RM} ${OBJECTDIR}/_ext/1471363227/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1471363227/drv_spi.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1471363227/drv_spi.o.d" -o ${OBJECTDIR}/_ext/1471363227/drv_spi.o ../src/config/Artemis_PDU/driver/spi/src/drv_spi.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1624425958/osal_freertos.o: ../src/config/Artemis_PDU/osal/osal_freertos.c  .generated_files/flags/Artemis_PDU/729216002d51a0425f7cc4762dfd6511905901b .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1624425958" 
	@${RM} ${OBJECTDIR}/_ext/1624425958/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/1624425958/osal_freertos.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1624425958/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/1624425958/osal_freertos.o ../src/config/Artemis_PDU/osal/osal_freertos.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1650922938/plib_clock.o: ../src/config/Artemis_PDU/peripheral/clock/plib_clock.c  .generated_files/flags/Artemis_PDU/5f128241f6e3ca79c0506b69c0ee2e8c9108b61b .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1650922938" 
	@${RM} ${OBJECTDIR}/_ext/1650922938/plib_clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1650922938/plib_clock.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1650922938/plib_clock.o.d" -o ${OBJECTDIR}/_ext/1650922938/plib_clock.o ../src/config/Artemis_PDU/peripheral/clock/plib_clock.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1023086318/plib_cmcc.o: ../src/config/Artemis_PDU/peripheral/cmcc/plib_cmcc.c  .generated_files/flags/Artemis_PDU/6a585fafeeac27b3f30610bc13a1b081f9ff10 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1023086318" 
	@${RM} ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1023086318/plib_cmcc.o.d" -o ${OBJECTDIR}/_ext/1023086318/plib_cmcc.o ../src/config/Artemis_PDU/peripheral/cmcc/plib_cmcc.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1648773452/plib_evsys.o: ../src/config/Artemis_PDU/peripheral/evsys/plib_evsys.c  .generated_files/flags/Artemis_PDU/1997cf6ba76db8d29bc7a92b217fd8f0bcd7e2a2 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1648773452" 
	@${RM} ${OBJECTDIR}/_ext/1648773452/plib_evsys.o.d 
	@${RM} ${OBJECTDIR}/_ext/1648773452/plib_evsys.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1648773452/plib_evsys.o.d" -o ${OBJECTDIR}/_ext/1648773452/plib_evsys.o ../src/config/Artemis_PDU/peripheral/evsys/plib_evsys.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1022749782/plib_nvic.o: ../src/config/Artemis_PDU/peripheral/nvic/plib_nvic.c  .generated_files/flags/Artemis_PDU/de628f4bcff67c738c3bef3fbf81a757faf4a784 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1022749782" 
	@${RM} ${OBJECTDIR}/_ext/1022749782/plib_nvic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1022749782/plib_nvic.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1022749782/plib_nvic.o.d" -o ${OBJECTDIR}/_ext/1022749782/plib_nvic.o ../src/config/Artemis_PDU/peripheral/nvic/plib_nvic.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o: ../src/config/Artemis_PDU/peripheral/nvmctrl/plib_nvmctrl.c  .generated_files/flags/Artemis_PDU/4d475ffbaab9cd6b16a617ae2bee64ab1b494ff9 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/236948536" 
	@${RM} ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o.d 
	@${RM} ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o.d" -o ${OBJECTDIR}/_ext/236948536/plib_nvmctrl.o ../src/config/Artemis_PDU/peripheral/nvmctrl/plib_nvmctrl.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1022696631/plib_port.o: ../src/config/Artemis_PDU/peripheral/port/plib_port.c  .generated_files/flags/Artemis_PDU/3c523b0487c9e5cd8df485c33458b8b55d857a1 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1022696631" 
	@${RM} ${OBJECTDIR}/_ext/1022696631/plib_port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1022696631/plib_port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1022696631/plib_port.o.d" -o ${OBJECTDIR}/_ext/1022696631/plib_port.o ../src/config/Artemis_PDU/peripheral/port/plib_port.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o: ../src/config/Artemis_PDU/peripheral/rtc/plib_rtc_timer.c  .generated_files/flags/Artemis_PDU/e6613f02d124f9719ca75ecdd49e200b82a66d50 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/659748505" 
	@${RM} ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o.d" -o ${OBJECTDIR}/_ext/659748505/plib_rtc_timer.o ../src/config/Artemis_PDU/peripheral/rtc/plib_rtc_timer.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o: ../src/config/Artemis_PDU/peripheral/sercom/i2c_slave/plib_sercom4_i2c_slave.c  .generated_files/flags/Artemis_PDU/1cb2b8964d326f45dcaf44f9d0e642c0eb77af20 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/835284660" 
	@${RM} ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o.d 
	@${RM} ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o.d" -o ${OBJECTDIR}/_ext/835284660/plib_sercom4_i2c_slave.o ../src/config/Artemis_PDU/peripheral/sercom/i2c_slave/plib_sercom4_i2c_slave.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o: ../src/config/Artemis_PDU/peripheral/sercom/spi_master/plib_sercom2_spi_master.c  .generated_files/flags/Artemis_PDU/2ffc425d4a9a3265df352e708fab79e689c254ad .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/796654533" 
	@${RM} ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o.d 
	@${RM} ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o.d" -o ${OBJECTDIR}/_ext/796654533/plib_sercom2_spi_master.o ../src/config/Artemis_PDU/peripheral/sercom/spi_master/plib_sercom2_spi_master.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o: ../src/config/Artemis_PDU/peripheral/sercom/usart/plib_sercom3_usart.c  .generated_files/flags/Artemis_PDU/90f2f2648bbd8f220ba23215a9f543d711f81673 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1679430495" 
	@${RM} ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o.d" -o ${OBJECTDIR}/_ext/1679430495/plib_sercom3_usart.o ../src/config/Artemis_PDU/peripheral/sercom/usart/plib_sercom3_usart.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1186129630/xc32_monitor.o: ../src/config/Artemis_PDU/stdio/xc32_monitor.c  .generated_files/flags/Artemis_PDU/435288ae837a1f82a2b332fd715a98f3c1f063d8 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1186129630" 
	@${RM} ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o.d 
	@${RM} ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1186129630/xc32_monitor.o.d" -o ${OBJECTDIR}/_ext/1186129630/xc32_monitor.o ../src/config/Artemis_PDU/stdio/xc32_monitor.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/177662547/sys_cache.o: ../src/config/Artemis_PDU/system/cache/sys_cache.c  .generated_files/flags/Artemis_PDU/33b7892afc311b26ee195f7dbe7a2e6f3f15c84a .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/177662547" 
	@${RM} ${OBJECTDIR}/_ext/177662547/sys_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/177662547/sys_cache.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/177662547/sys_cache.o.d" -o ${OBJECTDIR}/_ext/177662547/sys_cache.o ../src/config/Artemis_PDU/system/cache/sys_cache.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1148418531/sys_dma.o: ../src/config/Artemis_PDU/system/dma/sys_dma.c  .generated_files/flags/Artemis_PDU/a44d260032ea0d2eed5649acbd84b3ae0e47dfd5 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1148418531" 
	@${RM} ${OBJECTDIR}/_ext/1148418531/sys_dma.o.d 
	@${RM} ${OBJECTDIR}/_ext/1148418531/sys_dma.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1148418531/sys_dma.o.d" -o ${OBJECTDIR}/_ext/1148418531/sys_dma.o ../src/config/Artemis_PDU/system/dma/sys_dma.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/401857885/ff.o: ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ff.c  .generated_files/flags/Artemis_PDU/2106f8ffe55e74d5cbac6ff335c2c2eac4ea9b1f .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/401857885" 
	@${RM} ${OBJECTDIR}/_ext/401857885/ff.o.d 
	@${RM} ${OBJECTDIR}/_ext/401857885/ff.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/401857885/ff.o.d" -o ${OBJECTDIR}/_ext/401857885/ff.o ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ff.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/401857885/ffunicode.o: ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ffunicode.c  .generated_files/flags/Artemis_PDU/845d0852a55d46b06d869e9e0b04b9d636b062a3 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/401857885" 
	@${RM} ${OBJECTDIR}/_ext/401857885/ffunicode.o.d 
	@${RM} ${OBJECTDIR}/_ext/401857885/ffunicode.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/401857885/ffunicode.o.d" -o ${OBJECTDIR}/_ext/401857885/ffunicode.o ../src/config/Artemis_PDU/system/fs/fat_fs/file_system/ffunicode.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/867183764/diskio.o: ../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access/diskio.c  .generated_files/flags/Artemis_PDU/9d79de341da7b60c62903c93906af2f9590e251f .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/867183764" 
	@${RM} ${OBJECTDIR}/_ext/867183764/diskio.o.d 
	@${RM} ${OBJECTDIR}/_ext/867183764/diskio.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/867183764/diskio.o.d" -o ${OBJECTDIR}/_ext/867183764/diskio.o ../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access/diskio.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1111598889/sys_fs.o: ../src/config/Artemis_PDU/system/fs/src/sys_fs.c  .generated_files/flags/Artemis_PDU/6e9de1b60d4a1d461bf0999932a6fbdd3af21437 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1111598889" 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1111598889/sys_fs.o.d" -o ${OBJECTDIR}/_ext/1111598889/sys_fs.o ../src/config/Artemis_PDU/system/fs/src/sys_fs.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o: ../src/config/Artemis_PDU/system/fs/src/sys_fs_media_manager.c  .generated_files/flags/Artemis_PDU/cf110a5ba5af2ea59045cd386d078069251123b6 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1111598889" 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o.d" -o ${OBJECTDIR}/_ext/1111598889/sys_fs_media_manager.o ../src/config/Artemis_PDU/system/fs/src/sys_fs_media_manager.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o: ../src/config/Artemis_PDU/system/fs/src/sys_fs_fat_interface.c  .generated_files/flags/Artemis_PDU/2ac1272bd753cf6283a8e4f43d52575c56c122b6 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1111598889" 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o.d 
	@${RM} ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o.d" -o ${OBJECTDIR}/_ext/1111598889/sys_fs_fat_interface.o ../src/config/Artemis_PDU/system/fs/src/sys_fs_fat_interface.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1813730513/sys_int.o: ../src/config/Artemis_PDU/system/int/src/sys_int.c  .generated_files/flags/Artemis_PDU/473b4f6fac1023db9acc5655071e9c11dd7faac1 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1813730513" 
	@${RM} ${OBJECTDIR}/_ext/1813730513/sys_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/1813730513/sys_int.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1813730513/sys_int.o.d" -o ${OBJECTDIR}/_ext/1813730513/sys_int.o ../src/config/Artemis_PDU/system/int/src/sys_int.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1304864151/sys_time.o: ../src/config/Artemis_PDU/system/time/src/sys_time.c  .generated_files/flags/Artemis_PDU/df2e2681a61e3f783af280db16e61188b698abe4 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1304864151" 
	@${RM} ${OBJECTDIR}/_ext/1304864151/sys_time.o.d 
	@${RM} ${OBJECTDIR}/_ext/1304864151/sys_time.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1304864151/sys_time.o.d" -o ${OBJECTDIR}/_ext/1304864151/sys_time.o ../src/config/Artemis_PDU/system/time/src/sys_time.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/initialization.o: ../src/config/Artemis_PDU/initialization.c  .generated_files/flags/Artemis_PDU/9db276115f46595eab97dcd39bbeccf2bd0d32d4 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/initialization.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/initialization.o.d" -o ${OBJECTDIR}/_ext/974488028/initialization.o ../src/config/Artemis_PDU/initialization.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/interrupts.o: ../src/config/Artemis_PDU/interrupts.c  .generated_files/flags/Artemis_PDU/831b20281610638ca3cd29f0e0874eadcb3a9712 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/interrupts.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/interrupts.o.d" -o ${OBJECTDIR}/_ext/974488028/interrupts.o ../src/config/Artemis_PDU/interrupts.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/exceptions.o: ../src/config/Artemis_PDU/exceptions.c  .generated_files/flags/Artemis_PDU/c3c3e4c7943a30a01194415ca6cc6bcc223c6c2e .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/exceptions.o.d" -o ${OBJECTDIR}/_ext/974488028/exceptions.o ../src/config/Artemis_PDU/exceptions.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/startup_xc32.o: ../src/config/Artemis_PDU/startup_xc32.c  .generated_files/flags/Artemis_PDU/9d25cc5b355872ff6bb7c8f17dbb1f9ef96f9322 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/startup_xc32.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/startup_xc32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/startup_xc32.o.d" -o ${OBJECTDIR}/_ext/974488028/startup_xc32.o ../src/config/Artemis_PDU/startup_xc32.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/libc_syscalls.o: ../src/config/Artemis_PDU/libc_syscalls.c  .generated_files/flags/Artemis_PDU/caf49fa2f948ca9e21f08633c873bdf45b61695a .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/libc_syscalls.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/libc_syscalls.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/libc_syscalls.o.d" -o ${OBJECTDIR}/_ext/974488028/libc_syscalls.o ../src/config/Artemis_PDU/libc_syscalls.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/freertos_hooks.o: ../src/config/Artemis_PDU/freertos_hooks.c  .generated_files/flags/Artemis_PDU/45231226c48d83a4fc4ae9af42f7ac7a53e167bd .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/freertos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/freertos_hooks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/freertos_hooks.o.d" -o ${OBJECTDIR}/_ext/974488028/freertos_hooks.o ../src/config/Artemis_PDU/freertos_hooks.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/974488028/tasks.o: ../src/config/Artemis_PDU/tasks.c  .generated_files/flags/Artemis_PDU/f375875623ec29034c31ff42999c71bfaa5a1e8e .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/974488028" 
	@${RM} ${OBJECTDIR}/_ext/974488028/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/974488028/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/974488028/tasks.o.d" -o ${OBJECTDIR}/_ext/974488028/tasks.o ../src/config/Artemis_PDU/tasks.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/246609638/port.o: ../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F/port.c  .generated_files/flags/Artemis_PDU/a7bf097469fedbe029cf5765fe3608143a3f636f .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/246609638" 
	@${RM} ${OBJECTDIR}/_ext/246609638/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/246609638/port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/246609638/port.o.d" -o ${OBJECTDIR}/_ext/246609638/port.o ../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F/port.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1665200909/heap_1.o: ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c  .generated_files/flags/Artemis_PDU/b5a2880e51375358045b9d947dbdec4fa1deec26 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1665200909" 
	@${RM} ${OBJECTDIR}/_ext/1665200909/heap_1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1665200909/heap_1.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1665200909/heap_1.o.d" -o ${OBJECTDIR}/_ext/1665200909/heap_1.o ../src/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_1.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/croutine.o: ../src/third_party/rtos/FreeRTOS/Source/croutine.c  .generated_files/flags/Artemis_PDU/262e819ad43127af2ec482ae0508eb5f658d39fb .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/croutine.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/croutine.o.d" -o ${OBJECTDIR}/_ext/404212886/croutine.o ../src/third_party/rtos/FreeRTOS/Source/croutine.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/list.o: ../src/third_party/rtos/FreeRTOS/Source/list.c  .generated_files/flags/Artemis_PDU/cb0505e2a6c77500e1dc77792a969889e418305d .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/list.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/list.o.d" -o ${OBJECTDIR}/_ext/404212886/list.o ../src/third_party/rtos/FreeRTOS/Source/list.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/queue.o: ../src/third_party/rtos/FreeRTOS/Source/queue.c  .generated_files/flags/Artemis_PDU/c2295273eb91e138913f2b4f7b280c5ffb3dc262 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/queue.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/queue.o.d" -o ${OBJECTDIR}/_ext/404212886/queue.o ../src/third_party/rtos/FreeRTOS/Source/queue.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o: ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c  .generated_files/flags/Artemis_PDU/4448fcdffdea6d4eabebb8b81168f8db175ff81 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o.d" -o ${OBJECTDIR}/_ext/404212886/FreeRTOS_tasks.o ../src/third_party/rtos/FreeRTOS/Source/FreeRTOS_tasks.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/timers.o: ../src/third_party/rtos/FreeRTOS/Source/timers.c  .generated_files/flags/Artemis_PDU/f2279310b410039e96cb47d0f7354deae937213 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/timers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/timers.o.d" -o ${OBJECTDIR}/_ext/404212886/timers.o ../src/third_party/rtos/FreeRTOS/Source/timers.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/event_groups.o: ../src/third_party/rtos/FreeRTOS/Source/event_groups.c  .generated_files/flags/Artemis_PDU/bd5e9a03ad345fdd716d8441f61226381f755103 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/event_groups.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/event_groups.o.d" -o ${OBJECTDIR}/_ext/404212886/event_groups.o ../src/third_party/rtos/FreeRTOS/Source/event_groups.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/404212886/stream_buffer.o: ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c  .generated_files/flags/Artemis_PDU/507c0370faaf193fc5a4a0d747f1d7e7f632f65c .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/404212886" 
	@${RM} ${OBJECTDIR}/_ext/404212886/stream_buffer.o.d 
	@${RM} ${OBJECTDIR}/_ext/404212886/stream_buffer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/404212886/stream_buffer.o.d" -o ${OBJECTDIR}/_ext/404212886/stream_buffer.o ../src/third_party/rtos/FreeRTOS/Source/stream_buffer.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/flags/Artemis_PDU/7921800957cd4a55c21a915d34246e081c49eea4 .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  .generated_files/flags/Artemis_PDU/b36b44d5cc4d8950fe9aee5c57866b92abb0310d .generated_files/flags/Artemis_PDU/94cc8d5d174c81b19c972cbd48d6a449853dd346
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O1 -fno-common -I"../src" -I"../src/config/Artemis_PDU" -I"../src/config/Artemis_PDU/system/fs/fat_fs/file_system" -I"../src/config/Artemis_PDU/system/fs/fat_fs/hardware_access" -I"../src/packs/ATSAME51N19A_DFP" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/third_party/rtos/FreeRTOS/Source/include" -I"../src/third_party/rtos/FreeRTOS/Source/portable/GCC/SAM/ARM_CM4F" -Werror -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c    -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}" ${PACK_COMMON_OPTIONS} 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/Artemis_PDU.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ../src/config/Artemis_PDU/ATSAME51N19A.ld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g   -mprocessor=$(MP_PROCESSOR_OPTION) -mno-device-startup-code -o ${DISTDIR}/Artemis_PDU.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=_min_heap_size=512,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,${DISTDIR}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
${DISTDIR}/Artemis_PDU.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ../src/config/Artemis_PDU/ATSAME51N19A.ld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION) -mno-device-startup-code -o ${DISTDIR}/Artemis_PDU.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_Artemis_PDU=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=512,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,${DISTDIR}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex ${DISTDIR}/Artemis_PDU.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
