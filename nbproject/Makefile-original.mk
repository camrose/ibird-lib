#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-original.mk)" "nbproject/Makefile-local-original.mk"
include nbproject/Makefile-local-original.mk
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=original
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ibird-lib.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/ibird-lib.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/source/adc_pid.o ${OBJECTDIR}/source/clock_sync.o ${OBJECTDIR}/source/cv.o ${OBJECTDIR}/source/directory.o ${OBJECTDIR}/source/hall.o ${OBJECTDIR}/source/lstrobe.o ${OBJECTDIR}/source/motor_ctrl.o ${OBJECTDIR}/source/net.o ${OBJECTDIR}/source/pbuff.o ${OBJECTDIR}/source/ppbuff.o ${OBJECTDIR}/source/rate.o ${OBJECTDIR}/source/regulator.o ${OBJECTDIR}/source/slew.o ${OBJECTDIR}/source/sqrti.o ${OBJECTDIR}/source/sync_servo.o ${OBJECTDIR}/source/sys_clock.o ${OBJECTDIR}/source/telemetry.o ${OBJECTDIR}/_ext/921515994/attitude.o ${OBJECTDIR}/_ext/921515994/bams.o ${OBJECTDIR}/_ext/921515994/battery.o ${OBJECTDIR}/_ext/921515994/carray.o ${OBJECTDIR}/_ext/921515994/controller.o ${OBJECTDIR}/_ext/921515994/counter.o ${OBJECTDIR}/_ext/921515994/delay.o ${OBJECTDIR}/_ext/921515994/dfilter.o ${OBJECTDIR}/_ext/921515994/dfilter_avg.o ${OBJECTDIR}/_ext/921515994/dfmem.o ${OBJECTDIR}/_ext/921515994/gyro.o ${OBJECTDIR}/_ext/921515994/i2c_driver.o ${OBJECTDIR}/_ext/921515994/init_default.o ${OBJECTDIR}/_ext/921515994/ipspi1.o ${OBJECTDIR}/_ext/921515994/larray.o ${OBJECTDIR}/_ext/921515994/lcd.o ${OBJECTDIR}/_ext/921515994/ovcam.o ${OBJECTDIR}/_ext/921515994/ovcamHS.o ${OBJECTDIR}/_ext/921515994/packet_queue.o ${OBJECTDIR}/_ext/921515994/payload.o ${OBJECTDIR}/_ext/921515994/payload_queue.o ${OBJECTDIR}/_ext/921515994/pid.o ${OBJECTDIR}/_ext/921515994/ppool.o ${OBJECTDIR}/_ext/921515994/quat.o ${OBJECTDIR}/_ext/921515994/queue.o ${OBJECTDIR}/_ext/921515994/radio.o ${OBJECTDIR}/_ext/921515994/spi_controller.o ${OBJECTDIR}/_ext/921515994/stopwatch.o ${OBJECTDIR}/_ext/921515994/wii.o ${OBJECTDIR}/_ext/921515994/xl.o ${OBJECTDIR}/_ext/921515994/mac_packet.o ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o ${OBJECTDIR}/_ext/921515994/cam.o ${OBJECTDIR}/source/cmd.o ${OBJECTDIR}/source/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/source/adc_pid.o.d ${OBJECTDIR}/source/clock_sync.o.d ${OBJECTDIR}/source/cv.o.d ${OBJECTDIR}/source/directory.o.d ${OBJECTDIR}/source/hall.o.d ${OBJECTDIR}/source/lstrobe.o.d ${OBJECTDIR}/source/motor_ctrl.o.d ${OBJECTDIR}/source/net.o.d ${OBJECTDIR}/source/pbuff.o.d ${OBJECTDIR}/source/ppbuff.o.d ${OBJECTDIR}/source/rate.o.d ${OBJECTDIR}/source/regulator.o.d ${OBJECTDIR}/source/slew.o.d ${OBJECTDIR}/source/sqrti.o.d ${OBJECTDIR}/source/sync_servo.o.d ${OBJECTDIR}/source/sys_clock.o.d ${OBJECTDIR}/source/telemetry.o.d ${OBJECTDIR}/_ext/921515994/attitude.o.d ${OBJECTDIR}/_ext/921515994/bams.o.d ${OBJECTDIR}/_ext/921515994/battery.o.d ${OBJECTDIR}/_ext/921515994/carray.o.d ${OBJECTDIR}/_ext/921515994/controller.o.d ${OBJECTDIR}/_ext/921515994/counter.o.d ${OBJECTDIR}/_ext/921515994/delay.o.d ${OBJECTDIR}/_ext/921515994/dfilter.o.d ${OBJECTDIR}/_ext/921515994/dfilter_avg.o.d ${OBJECTDIR}/_ext/921515994/dfmem.o.d ${OBJECTDIR}/_ext/921515994/gyro.o.d ${OBJECTDIR}/_ext/921515994/i2c_driver.o.d ${OBJECTDIR}/_ext/921515994/init_default.o.d ${OBJECTDIR}/_ext/921515994/ipspi1.o.d ${OBJECTDIR}/_ext/921515994/larray.o.d ${OBJECTDIR}/_ext/921515994/lcd.o.d ${OBJECTDIR}/_ext/921515994/ovcam.o.d ${OBJECTDIR}/_ext/921515994/ovcamHS.o.d ${OBJECTDIR}/_ext/921515994/packet_queue.o.d ${OBJECTDIR}/_ext/921515994/payload.o.d ${OBJECTDIR}/_ext/921515994/payload_queue.o.d ${OBJECTDIR}/_ext/921515994/pid.o.d ${OBJECTDIR}/_ext/921515994/ppool.o.d ${OBJECTDIR}/_ext/921515994/quat.o.d ${OBJECTDIR}/_ext/921515994/queue.o.d ${OBJECTDIR}/_ext/921515994/radio.o.d ${OBJECTDIR}/_ext/921515994/spi_controller.o.d ${OBJECTDIR}/_ext/921515994/stopwatch.o.d ${OBJECTDIR}/_ext/921515994/wii.o.d ${OBJECTDIR}/_ext/921515994/xl.o.d ${OBJECTDIR}/_ext/921515994/mac_packet.o.d ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d ${OBJECTDIR}/_ext/921515994/cam.o.d ${OBJECTDIR}/source/cmd.o.d ${OBJECTDIR}/source/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/source/adc_pid.o ${OBJECTDIR}/source/clock_sync.o ${OBJECTDIR}/source/cv.o ${OBJECTDIR}/source/directory.o ${OBJECTDIR}/source/hall.o ${OBJECTDIR}/source/lstrobe.o ${OBJECTDIR}/source/motor_ctrl.o ${OBJECTDIR}/source/net.o ${OBJECTDIR}/source/pbuff.o ${OBJECTDIR}/source/ppbuff.o ${OBJECTDIR}/source/rate.o ${OBJECTDIR}/source/regulator.o ${OBJECTDIR}/source/slew.o ${OBJECTDIR}/source/sqrti.o ${OBJECTDIR}/source/sync_servo.o ${OBJECTDIR}/source/sys_clock.o ${OBJECTDIR}/source/telemetry.o ${OBJECTDIR}/_ext/921515994/attitude.o ${OBJECTDIR}/_ext/921515994/bams.o ${OBJECTDIR}/_ext/921515994/battery.o ${OBJECTDIR}/_ext/921515994/carray.o ${OBJECTDIR}/_ext/921515994/controller.o ${OBJECTDIR}/_ext/921515994/counter.o ${OBJECTDIR}/_ext/921515994/delay.o ${OBJECTDIR}/_ext/921515994/dfilter.o ${OBJECTDIR}/_ext/921515994/dfilter_avg.o ${OBJECTDIR}/_ext/921515994/dfmem.o ${OBJECTDIR}/_ext/921515994/gyro.o ${OBJECTDIR}/_ext/921515994/i2c_driver.o ${OBJECTDIR}/_ext/921515994/init_default.o ${OBJECTDIR}/_ext/921515994/ipspi1.o ${OBJECTDIR}/_ext/921515994/larray.o ${OBJECTDIR}/_ext/921515994/lcd.o ${OBJECTDIR}/_ext/921515994/ovcam.o ${OBJECTDIR}/_ext/921515994/ovcamHS.o ${OBJECTDIR}/_ext/921515994/packet_queue.o ${OBJECTDIR}/_ext/921515994/payload.o ${OBJECTDIR}/_ext/921515994/payload_queue.o ${OBJECTDIR}/_ext/921515994/pid.o ${OBJECTDIR}/_ext/921515994/ppool.o ${OBJECTDIR}/_ext/921515994/quat.o ${OBJECTDIR}/_ext/921515994/queue.o ${OBJECTDIR}/_ext/921515994/radio.o ${OBJECTDIR}/_ext/921515994/spi_controller.o ${OBJECTDIR}/_ext/921515994/stopwatch.o ${OBJECTDIR}/_ext/921515994/wii.o ${OBJECTDIR}/_ext/921515994/xl.o ${OBJECTDIR}/_ext/921515994/mac_packet.o ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o ${OBJECTDIR}/_ext/921515994/cam.o ${OBJECTDIR}/source/cmd.o ${OBJECTDIR}/source/main.o


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-original.mk dist/${CND_CONF}/${IMAGE_TYPE}/ibird-lib.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ128MC706A
MP_LINKER_FILE_OPTION=,--script="p33FJ128MC706A_Bootload.gld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/source/adc_pid.o: ./source/adc_pid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/adc_pid.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/adc_pid.c  -o ${OBJECTDIR}/source/adc_pid.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/adc_pid.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/adc_pid.o.d" $(SILENT) 
	
${OBJECTDIR}/source/clock_sync.o: ./source/clock_sync.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/clock_sync.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/clock_sync.c  -o ${OBJECTDIR}/source/clock_sync.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/clock_sync.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/clock_sync.o.d" $(SILENT) 
	
${OBJECTDIR}/source/cv.o: ./source/cv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/cv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/cv.c  -o ${OBJECTDIR}/source/cv.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/cv.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/cv.o.d" $(SILENT) 
	
${OBJECTDIR}/source/directory.o: ./source/directory.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/directory.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/directory.c  -o ${OBJECTDIR}/source/directory.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/directory.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/directory.o.d" $(SILENT) 
	
${OBJECTDIR}/source/hall.o: ./source/hall.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/hall.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/hall.c  -o ${OBJECTDIR}/source/hall.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/hall.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/hall.o.d" $(SILENT) 
	
${OBJECTDIR}/source/lstrobe.o: ./source/lstrobe.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/lstrobe.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/lstrobe.c  -o ${OBJECTDIR}/source/lstrobe.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/lstrobe.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/lstrobe.o.d" $(SILENT) 
	
${OBJECTDIR}/source/motor_ctrl.o: ./source/motor_ctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/motor_ctrl.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/motor_ctrl.c  -o ${OBJECTDIR}/source/motor_ctrl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/motor_ctrl.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/motor_ctrl.o.d" $(SILENT) 
	
${OBJECTDIR}/source/net.o: ./source/net.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/net.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/net.c  -o ${OBJECTDIR}/source/net.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/net.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/net.o.d" $(SILENT) 
	
${OBJECTDIR}/source/pbuff.o: ./source/pbuff.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/pbuff.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/pbuff.c  -o ${OBJECTDIR}/source/pbuff.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/pbuff.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/pbuff.o.d" $(SILENT) 
	
${OBJECTDIR}/source/ppbuff.o: ./source/ppbuff.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/ppbuff.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/ppbuff.c  -o ${OBJECTDIR}/source/ppbuff.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/ppbuff.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/ppbuff.o.d" $(SILENT) 
	
${OBJECTDIR}/source/rate.o: ./source/rate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/rate.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/rate.c  -o ${OBJECTDIR}/source/rate.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/rate.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/rate.o.d" $(SILENT) 
	
${OBJECTDIR}/source/regulator.o: ./source/regulator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/regulator.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/regulator.c  -o ${OBJECTDIR}/source/regulator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/regulator.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/regulator.o.d" $(SILENT) 
	
${OBJECTDIR}/source/slew.o: ./source/slew.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/slew.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/slew.c  -o ${OBJECTDIR}/source/slew.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/slew.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/slew.o.d" $(SILENT) 
	
${OBJECTDIR}/source/sqrti.o: ./source/sqrti.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/sqrti.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/sqrti.c  -o ${OBJECTDIR}/source/sqrti.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/sqrti.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/sqrti.o.d" $(SILENT) 
	
${OBJECTDIR}/source/sync_servo.o: ./source/sync_servo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/sync_servo.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/sync_servo.c  -o ${OBJECTDIR}/source/sync_servo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/sync_servo.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/sync_servo.o.d" $(SILENT) 
	
${OBJECTDIR}/source/sys_clock.o: ./source/sys_clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/sys_clock.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/sys_clock.c  -o ${OBJECTDIR}/source/sys_clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/sys_clock.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/sys_clock.o.d" $(SILENT) 
	
${OBJECTDIR}/source/telemetry.o: ./source/telemetry.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/telemetry.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/telemetry.c  -o ${OBJECTDIR}/source/telemetry.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/telemetry.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/telemetry.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/attitude.o: ../imageproc-lib/attitude.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/attitude.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/attitude.c  -o ${OBJECTDIR}/_ext/921515994/attitude.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/attitude.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/attitude.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/bams.o: ../imageproc-lib/bams.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/bams.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/bams.c  -o ${OBJECTDIR}/_ext/921515994/bams.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/bams.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/bams.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/battery.o: ../imageproc-lib/battery.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/battery.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/battery.c  -o ${OBJECTDIR}/_ext/921515994/battery.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/battery.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/battery.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/carray.o: ../imageproc-lib/carray.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/carray.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/carray.c  -o ${OBJECTDIR}/_ext/921515994/carray.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/carray.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/carray.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/controller.o: ../imageproc-lib/controller.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/controller.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/controller.c  -o ${OBJECTDIR}/_ext/921515994/controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/controller.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/controller.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/counter.o: ../imageproc-lib/counter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/counter.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/counter.c  -o ${OBJECTDIR}/_ext/921515994/counter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/counter.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/counter.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/dfilter.o: ../imageproc-lib/dfilter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/dfilter.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/dfilter.c  -o ${OBJECTDIR}/_ext/921515994/dfilter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/dfilter.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/dfilter.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/dfilter_avg.o: ../imageproc-lib/dfilter_avg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/dfilter_avg.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/dfilter_avg.c  -o ${OBJECTDIR}/_ext/921515994/dfilter_avg.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/dfilter_avg.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/dfilter_avg.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/dfmem.o: ../imageproc-lib/dfmem.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/dfmem.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/dfmem.c  -o ${OBJECTDIR}/_ext/921515994/dfmem.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/dfmem.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/dfmem.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/gyro.o: ../imageproc-lib/gyro.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/gyro.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/gyro.c  -o ${OBJECTDIR}/_ext/921515994/gyro.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/gyro.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/gyro.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/i2c_driver.o: ../imageproc-lib/i2c_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/i2c_driver.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/i2c_driver.c  -o ${OBJECTDIR}/_ext/921515994/i2c_driver.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/i2c_driver.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/i2c_driver.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/init_default.o: ../imageproc-lib/init_default.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/init_default.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/init_default.c  -o ${OBJECTDIR}/_ext/921515994/init_default.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/init_default.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/init_default.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/ipspi1.o: ../imageproc-lib/ipspi1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ipspi1.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ipspi1.c  -o ${OBJECTDIR}/_ext/921515994/ipspi1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ipspi1.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ipspi1.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/larray.o: ../imageproc-lib/larray.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/larray.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/larray.c  -o ${OBJECTDIR}/_ext/921515994/larray.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/larray.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/larray.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/lcd.o: ../imageproc-lib/lcd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/lcd.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/lcd.c  -o ${OBJECTDIR}/_ext/921515994/lcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/lcd.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/lcd.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/ovcam.o: ../imageproc-lib/ovcam.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ovcam.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ovcam.c  -o ${OBJECTDIR}/_ext/921515994/ovcam.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ovcam.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ovcam.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/packet_queue.o: ../imageproc-lib/packet_queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/packet_queue.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/packet_queue.c  -o ${OBJECTDIR}/_ext/921515994/packet_queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/packet_queue.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/packet_queue.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/payload.o: ../imageproc-lib/payload.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/payload.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/payload.c  -o ${OBJECTDIR}/_ext/921515994/payload.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/payload.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/payload.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/payload_queue.o: ../imageproc-lib/payload_queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/payload_queue.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/payload_queue.c  -o ${OBJECTDIR}/_ext/921515994/payload_queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/payload_queue.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/payload_queue.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/pid.o: ../imageproc-lib/pid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/pid.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/pid.c  -o ${OBJECTDIR}/_ext/921515994/pid.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/pid.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/pid.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/ppool.o: ../imageproc-lib/ppool.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ppool.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ppool.c  -o ${OBJECTDIR}/_ext/921515994/ppool.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ppool.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ppool.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/quat.o: ../imageproc-lib/quat.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/quat.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/quat.c  -o ${OBJECTDIR}/_ext/921515994/quat.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/quat.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/quat.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/queue.o: ../imageproc-lib/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/queue.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/queue.c  -o ${OBJECTDIR}/_ext/921515994/queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/queue.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/queue.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/radio.o: ../imageproc-lib/radio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/radio.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/radio.c  -o ${OBJECTDIR}/_ext/921515994/radio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/radio.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/radio.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/spi_controller.o: ../imageproc-lib/spi_controller.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/spi_controller.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/spi_controller.c  -o ${OBJECTDIR}/_ext/921515994/spi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/spi_controller.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/spi_controller.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/stopwatch.o: ../imageproc-lib/stopwatch.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/stopwatch.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/stopwatch.c  -o ${OBJECTDIR}/_ext/921515994/stopwatch.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/stopwatch.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/stopwatch.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/wii.o: ../imageproc-lib/wii.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/wii.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/wii.c  -o ${OBJECTDIR}/_ext/921515994/wii.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/wii.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/wii.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/xl.o: ../imageproc-lib/xl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/xl.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/xl.c  -o ${OBJECTDIR}/_ext/921515994/xl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/xl.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/xl.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/mac_packet.o: ../imageproc-lib/mac_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/mac_packet.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/mac_packet.c  -o ${OBJECTDIR}/_ext/921515994/mac_packet.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/mac_packet.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/mac_packet.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/at86rf231_driver.o: ../imageproc-lib/at86rf231_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/at86rf231_driver.c  -o ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/cam.o: ../imageproc-lib/cam.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/cam.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/cam.c  -o ${OBJECTDIR}/_ext/921515994/cam.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/cam.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/cam.o.d" $(SILENT) 
	
${OBJECTDIR}/source/cmd.o: ./source/cmd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/cmd.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/cmd.c  -o ${OBJECTDIR}/source/cmd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/cmd.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/cmd.o.d" $(SILENT) 
	
${OBJECTDIR}/source/main.o: ./source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/main.c  -o ${OBJECTDIR}/source/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/main.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/main.o.d" $(SILENT) 
	
else
${OBJECTDIR}/source/adc_pid.o: ./source/adc_pid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/adc_pid.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/adc_pid.c  -o ${OBJECTDIR}/source/adc_pid.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/adc_pid.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/adc_pid.o.d" $(SILENT) 
	
${OBJECTDIR}/source/clock_sync.o: ./source/clock_sync.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/clock_sync.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/clock_sync.c  -o ${OBJECTDIR}/source/clock_sync.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/clock_sync.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/clock_sync.o.d" $(SILENT) 
	
${OBJECTDIR}/source/cv.o: ./source/cv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/cv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/cv.c  -o ${OBJECTDIR}/source/cv.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/cv.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/cv.o.d" $(SILENT) 
	
${OBJECTDIR}/source/directory.o: ./source/directory.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/directory.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/directory.c  -o ${OBJECTDIR}/source/directory.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/directory.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/directory.o.d" $(SILENT) 
	
${OBJECTDIR}/source/hall.o: ./source/hall.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/hall.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/hall.c  -o ${OBJECTDIR}/source/hall.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/hall.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/hall.o.d" $(SILENT) 
	
${OBJECTDIR}/source/lstrobe.o: ./source/lstrobe.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/lstrobe.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/lstrobe.c  -o ${OBJECTDIR}/source/lstrobe.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/lstrobe.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/lstrobe.o.d" $(SILENT) 
	
${OBJECTDIR}/source/motor_ctrl.o: ./source/motor_ctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/motor_ctrl.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/motor_ctrl.c  -o ${OBJECTDIR}/source/motor_ctrl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/motor_ctrl.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/motor_ctrl.o.d" $(SILENT) 
	
${OBJECTDIR}/source/net.o: ./source/net.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/net.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/net.c  -o ${OBJECTDIR}/source/net.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/net.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/net.o.d" $(SILENT) 
	
${OBJECTDIR}/source/pbuff.o: ./source/pbuff.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/pbuff.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/pbuff.c  -o ${OBJECTDIR}/source/pbuff.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/pbuff.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/pbuff.o.d" $(SILENT) 
	
${OBJECTDIR}/source/ppbuff.o: ./source/ppbuff.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/ppbuff.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/ppbuff.c  -o ${OBJECTDIR}/source/ppbuff.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/ppbuff.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/ppbuff.o.d" $(SILENT) 
	
${OBJECTDIR}/source/rate.o: ./source/rate.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/rate.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/rate.c  -o ${OBJECTDIR}/source/rate.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/rate.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/rate.o.d" $(SILENT) 
	
${OBJECTDIR}/source/regulator.o: ./source/regulator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/regulator.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/regulator.c  -o ${OBJECTDIR}/source/regulator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/regulator.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/regulator.o.d" $(SILENT) 
	
${OBJECTDIR}/source/slew.o: ./source/slew.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/slew.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/slew.c  -o ${OBJECTDIR}/source/slew.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/slew.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/slew.o.d" $(SILENT) 
	
${OBJECTDIR}/source/sqrti.o: ./source/sqrti.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/sqrti.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/sqrti.c  -o ${OBJECTDIR}/source/sqrti.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/sqrti.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/sqrti.o.d" $(SILENT) 
	
${OBJECTDIR}/source/sync_servo.o: ./source/sync_servo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/sync_servo.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/sync_servo.c  -o ${OBJECTDIR}/source/sync_servo.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/sync_servo.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/sync_servo.o.d" $(SILENT) 
	
${OBJECTDIR}/source/sys_clock.o: ./source/sys_clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/sys_clock.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/sys_clock.c  -o ${OBJECTDIR}/source/sys_clock.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/sys_clock.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/sys_clock.o.d" $(SILENT) 
	
${OBJECTDIR}/source/telemetry.o: ./source/telemetry.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/telemetry.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/telemetry.c  -o ${OBJECTDIR}/source/telemetry.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/telemetry.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/telemetry.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/attitude.o: ../imageproc-lib/attitude.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/attitude.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/attitude.c  -o ${OBJECTDIR}/_ext/921515994/attitude.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/attitude.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/attitude.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/bams.o: ../imageproc-lib/bams.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/bams.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/bams.c  -o ${OBJECTDIR}/_ext/921515994/bams.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/bams.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/bams.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/battery.o: ../imageproc-lib/battery.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/battery.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/battery.c  -o ${OBJECTDIR}/_ext/921515994/battery.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/battery.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/battery.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/carray.o: ../imageproc-lib/carray.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/carray.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/carray.c  -o ${OBJECTDIR}/_ext/921515994/carray.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/carray.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/carray.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/controller.o: ../imageproc-lib/controller.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/controller.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/controller.c  -o ${OBJECTDIR}/_ext/921515994/controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/controller.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/controller.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/counter.o: ../imageproc-lib/counter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/counter.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/counter.c  -o ${OBJECTDIR}/_ext/921515994/counter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/counter.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/counter.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/dfilter.o: ../imageproc-lib/dfilter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/dfilter.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/dfilter.c  -o ${OBJECTDIR}/_ext/921515994/dfilter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/dfilter.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/dfilter.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/dfilter_avg.o: ../imageproc-lib/dfilter_avg.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/dfilter_avg.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/dfilter_avg.c  -o ${OBJECTDIR}/_ext/921515994/dfilter_avg.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/dfilter_avg.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/dfilter_avg.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/dfmem.o: ../imageproc-lib/dfmem.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/dfmem.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/dfmem.c  -o ${OBJECTDIR}/_ext/921515994/dfmem.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/dfmem.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/dfmem.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/gyro.o: ../imageproc-lib/gyro.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/gyro.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/gyro.c  -o ${OBJECTDIR}/_ext/921515994/gyro.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/gyro.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/gyro.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/i2c_driver.o: ../imageproc-lib/i2c_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/i2c_driver.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/i2c_driver.c  -o ${OBJECTDIR}/_ext/921515994/i2c_driver.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/i2c_driver.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/i2c_driver.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/init_default.o: ../imageproc-lib/init_default.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/init_default.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/init_default.c  -o ${OBJECTDIR}/_ext/921515994/init_default.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/init_default.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/init_default.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/ipspi1.o: ../imageproc-lib/ipspi1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ipspi1.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ipspi1.c  -o ${OBJECTDIR}/_ext/921515994/ipspi1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ipspi1.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ipspi1.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/larray.o: ../imageproc-lib/larray.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/larray.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/larray.c  -o ${OBJECTDIR}/_ext/921515994/larray.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/larray.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/larray.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/lcd.o: ../imageproc-lib/lcd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/lcd.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/lcd.c  -o ${OBJECTDIR}/_ext/921515994/lcd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/lcd.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/lcd.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/ovcam.o: ../imageproc-lib/ovcam.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ovcam.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ovcam.c  -o ${OBJECTDIR}/_ext/921515994/ovcam.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ovcam.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ovcam.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/packet_queue.o: ../imageproc-lib/packet_queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/packet_queue.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/packet_queue.c  -o ${OBJECTDIR}/_ext/921515994/packet_queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/packet_queue.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/packet_queue.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/payload.o: ../imageproc-lib/payload.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/payload.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/payload.c  -o ${OBJECTDIR}/_ext/921515994/payload.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/payload.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/payload.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/payload_queue.o: ../imageproc-lib/payload_queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/payload_queue.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/payload_queue.c  -o ${OBJECTDIR}/_ext/921515994/payload_queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/payload_queue.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/payload_queue.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/pid.o: ../imageproc-lib/pid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/pid.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/pid.c  -o ${OBJECTDIR}/_ext/921515994/pid.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/pid.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/pid.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/ppool.o: ../imageproc-lib/ppool.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ppool.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/ppool.c  -o ${OBJECTDIR}/_ext/921515994/ppool.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/ppool.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ppool.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/quat.o: ../imageproc-lib/quat.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/quat.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/quat.c  -o ${OBJECTDIR}/_ext/921515994/quat.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/quat.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/quat.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/queue.o: ../imageproc-lib/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/queue.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/queue.c  -o ${OBJECTDIR}/_ext/921515994/queue.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/queue.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/queue.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/radio.o: ../imageproc-lib/radio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/radio.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/radio.c  -o ${OBJECTDIR}/_ext/921515994/radio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/radio.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/radio.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/spi_controller.o: ../imageproc-lib/spi_controller.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/spi_controller.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/spi_controller.c  -o ${OBJECTDIR}/_ext/921515994/spi_controller.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/spi_controller.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/spi_controller.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/stopwatch.o: ../imageproc-lib/stopwatch.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/stopwatch.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/stopwatch.c  -o ${OBJECTDIR}/_ext/921515994/stopwatch.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/stopwatch.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/stopwatch.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/wii.o: ../imageproc-lib/wii.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/wii.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/wii.c  -o ${OBJECTDIR}/_ext/921515994/wii.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/wii.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/wii.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/xl.o: ../imageproc-lib/xl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/xl.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/xl.c  -o ${OBJECTDIR}/_ext/921515994/xl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/xl.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/xl.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/mac_packet.o: ../imageproc-lib/mac_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/mac_packet.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/mac_packet.c  -o ${OBJECTDIR}/_ext/921515994/mac_packet.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/mac_packet.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/mac_packet.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/at86rf231_driver.o: ../imageproc-lib/at86rf231_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/at86rf231_driver.c  -o ${OBJECTDIR}/_ext/921515994/at86rf231_driver.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/at86rf231_driver.o.d" $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/cam.o: ../imageproc-lib/cam.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/cam.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../imageproc-lib/cam.c  -o ${OBJECTDIR}/_ext/921515994/cam.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/921515994/cam.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/cam.o.d" $(SILENT) 
	
${OBJECTDIR}/source/cmd.o: ./source/cmd.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/cmd.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/cmd.c  -o ${OBJECTDIR}/source/cmd.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/cmd.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/cmd.o.d" $(SILENT) 
	
${OBJECTDIR}/source/main.o: ./source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ./source/main.c  -o ${OBJECTDIR}/source/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/main.o.d"        -g -omf=elf -fast-math -mlarge-code -mlarge-data -O0 -I"..\ibird-lib" -I"..\imageproc-lib" -D__IMAGEPROC2 -D__DFMEM_8MBIT -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/main.o.d" $(SILENT) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/921515994/delay.o: ../imageproc-lib/delay.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/delay.o.d 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../imageproc-lib/delay.s  -o ${OBJECTDIR}/_ext/921515994/delay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -I".." -Wa,-MD,"${OBJECTDIR}/_ext/921515994/delay.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/delay.o.d"  $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/ovcamHS.o: ../imageproc-lib/ovcamHS.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ovcamHS.o.d 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../imageproc-lib/ovcamHS.s  -o ${OBJECTDIR}/_ext/921515994/ovcamHS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -I".." -Wa,-MD,"${OBJECTDIR}/_ext/921515994/ovcamHS.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ovcamHS.o.d"  $(SILENT) 
	
else
${OBJECTDIR}/_ext/921515994/delay.o: ../imageproc-lib/delay.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/delay.o.d 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../imageproc-lib/delay.s  -o ${OBJECTDIR}/_ext/921515994/delay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -fast-math -I".." -Wa,-MD,"${OBJECTDIR}/_ext/921515994/delay.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/delay.o.d"  $(SILENT) 
	
${OBJECTDIR}/_ext/921515994/ovcamHS.o: ../imageproc-lib/ovcamHS.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/921515994 
	@${RM} ${OBJECTDIR}/_ext/921515994/ovcamHS.o.d 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../imageproc-lib/ovcamHS.s  -o ${OBJECTDIR}/_ext/921515994/ovcamHS.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=elf -fast-math -I".." -Wa,-MD,"${OBJECTDIR}/_ext/921515994/ovcamHS.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/921515994/ovcamHS.o.d"  $(SILENT) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/ibird-lib.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/ibird-lib.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -fast-math -Wl,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--heap=7288,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..\..\..\..\..\..\Program Files\Microchip\MPLAB C30\lib",--library-path="..",--no-force-link,--smart-io,-Map="${DISTDIR}\ImageProc2.X.${IMAGE_TYPE}.map",--report-mem,-ldsp-elf$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/ibird-lib.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/ibird-lib.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -fast-math -Wl,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--heap=7288,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..\..\..\..\..\..\Program Files\Microchip\MPLAB C30\lib",--library-path="..",--no-force-link,--smart-io,-Map="${DISTDIR}\ImageProc2.X.${IMAGE_TYPE}.map",--report-mem,-ldsp-elf$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/ibird-lib.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/original
	${RM} -r dist/original

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
