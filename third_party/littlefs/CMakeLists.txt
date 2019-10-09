cmake_minimum_required(VERSION 3.10)

include(${CMAKE_CURRENT_LIST_DIR}/../xpackage.cmake)


set(PACKAGE_NAME littlefs)

set(PACKAGE_VERSION 2.1.2)

set(SRC_DIR ${PACKAGE_NAME}-${PACKAGE_VERSION})

set(PACKAGE_URL https://github.com/ARMmbed/littlefs/archive/v${PACKAGE_VERSION}.zip)

list(APPEND EXCLUDES ".*/emubd/.*")
list(APPEND EXCLUDES ".*/scripts/.*")
list(APPEND EXCLUDES ".*/tests/.*")


if(NOT ${PACKAGE_NAME} IN_LIST LIBS)
	return()
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	set(FLAGS "")
else()
	set(FLAGS "-DLFS_NO_DEBUG -DLFS_NO_WARN -DLFS_NO_ERROR -DLFS_NO_ERROR -DLFS_NO_ASSERT")
endif()

xpackage_init(${PACKAGE_NAME} ${PACKAGE_VERSION} ${PACKAGE_URL} "${FLAGS}" SRC_ROOT BUILD_ROOT)

set(EXPORT_INCS ${SRC_ROOT}/${SRC_DIR})

xpackage_glob_src(SRCS ${SRC_ROOT}/${SRC_DIR}/*.c  EXCLUDES)

xpackage_build_target(${PACKAGE_NAME} ${BUILD_ROOT} SRCS EXPORT_INCS "")



