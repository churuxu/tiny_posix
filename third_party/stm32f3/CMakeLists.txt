cmake_minimum_required(VERSION 3.10)

include(${CMAKE_CURRENT_LIST_DIR}/../xpackage.cmake)


set(PACKAGE_NAME stm32f3)

set(PACKAGE_VERSION 170)

set(SRC_DIR STM32Cube_FW_F3_V1.7.0)

set(PACKAGE_URL http://sw-center.st.com/packs/resource/firmware/stm32cube_fw_f3_v${PACKAGE_VERSION}.zip)

set(EXCLUDES ".*Legacy/.*")

if(NOT ${PACKAGE_NAME} IN_LIST LIBS)
	return()
endif()


#301
if(${CPU_NAME} MATCHES ".*32F301..")
	set(MCU_DEFINE STM32F301x8)
#302
elseif(${CPU_NAME} MATCHES ".*32F302.[468]")
	set(MCU_DEFINE STM32F302x8)
elseif(${CPU_NAME} MATCHES ".*32F302.[BC]")
	set(MCU_DEFINE STM32F302xC)
elseif(${CPU_NAME} MATCHES ".*32F302.[DEG]")
	set(MCU_DEFINE STM32F302xE)
#303
elseif(${CPU_NAME} MATCHES ".*32F303.[468]")
	set(MCU_DEFINE STM32F303x8)
elseif(${CPU_NAME} MATCHES ".*32F303.[BC]")
	set(MCU_DEFINE STM32F303xC)
elseif(${CPU_NAME} MATCHES ".*32F303.[DEG]")
	set(MCU_DEFINE STM32F303xE)
#other
elseif(${CPU_NAME} MATCHES ".*32F373..")
	set(MCU_DEFINE STM32F373xC)
elseif(${CPU_NAME} MATCHES ".*32F334..")
	set(MCU_DEFINE STM32F334x8)
elseif(${CPU_NAME} MATCHES ".*32F318..")
	set(MCU_DEFINE STM32F318xx)	
elseif(${CPU_NAME} MATCHES ".*32F328..")
	set(MCU_DEFINE STM32F328xx)
elseif(${CPU_NAME} MATCHES ".*32F358..")
	set(MCU_DEFINE STM32F358xx)	
elseif(${CPU_NAME} MATCHES ".*32F378..")
	set(MCU_DEFINE STM32F378xx)		
elseif(${CPU_NAME} MATCHES ".*32F398..")
	set(MCU_DEFINE STM32F398xx)		
else()
	if(DEFINED STM32_DEFINE)
		set(MCU_DEFINE ${STM32_DEFINE})
	else()
		message(FATAL_ERROR "Must set valid 'CPU_NAME' variable")
	endif()
endif()

string(TOLOWER ${MCU_DEFINE} mcu_define_lcase)

if(NOT DEFINED STM32HAL_CONF_FILE)
	set(STM32HAL_CONF_FILE ${CMAKE_CURRENT_LIST_DIR}/stm32f3xx_hal_conf.h)
endif()

set(CONFIG_HASH "")
set(HEADER_HASH_STR "")

file(MD5 ${STM32HAL_CONF_FILE} HASH)
set(HEADER_HASH_STR "${HEADER_HASH_STR} ${HASH}")
string(MD5 HASH ${HEADER_HASH_STR})

set(FLAGS "-DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER -D${MCU_DEFINE} -DCONFIG_HASH=${HASH}")



xpackage_init(${PACKAGE_NAME} ${PACKAGE_VERSION} ${PACKAGE_URL} "${FLAGS}" SRC_ROOT BUILD_ROOT)

set(HAL_DRIVER_DIR ${SRC_ROOT}/${SRC_DIR}/Drivers/STM32F3xx_HAL_Driver)
set(CMSIS_DIR ${SRC_ROOT}/${SRC_DIR}/Drivers/CMSIS)
set(CMSIS_ST_DIR ${SRC_ROOT}/${SRC_DIR}/Drivers/CMSIS/Device/ST/STM32F3xx)


if(NOT EXISTS ${BUILD_ROOT}/stm32f3xx_hal_conf.h)
	configure_file(${STM32HAL_CONF_FILE} ${BUILD_ROOT}/stm32f3xx_hal_conf.h COPYONLY)
endif()

set(EXPORT_INCS ${BUILD_ROOT} ${HAL_DRIVER_DIR}/Inc ${CMSIS_ST_DIR}/Include ${CMSIS_DIR}/Include)

include_directories(${EXPORT_INCS})

xpackage_glob_src(SRCS ${HAL_DRIVER_DIR}/Src/*.c  EXCLUDES)

list(APPEND SRCS ${CMSIS_ST_DIR}/Source/Templates/system_stm32f3xx.c)
list(APPEND SRCS ${CMSIS_ST_DIR}/Source/Templates/gcc/startup_${mcu_define_lcase}.s)

xpackage_build_target(${PACKAGE_NAME} ${BUILD_ROOT} SRCS EXPORT_INCS FLAGS)



