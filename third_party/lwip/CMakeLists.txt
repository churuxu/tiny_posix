cmake_minimum_required(VERSION 3.10)

include(${CMAKE_CURRENT_LIST_DIR}/../xpackage.cmake)


set(PACKAGE_NAME lwip)

set(PACKAGE_VERSION 2.1.2)

set(SRC_DIR ${PACKAGE_NAME}-${PACKAGE_VERSION})

set(PACKAGE_URL http://mirror.ossplanet.net/nongnu/lwip/lwip-${PACKAGE_VERSION}.zip)

list(APPEND EXCLUDES ".*/apps/.*")
list(APPEND EXCLUDES ".*/doc/.*")
list(APPEND EXCLUDES ".*/test/.*")
list(APPEND EXCLUDES ".*/ppp/.*")

if(NOT ${PACKAGE_NAME} IN_LIST LIBS)
	return()
endif()


set(CONFIG_HASH "")
set(HEADER_HASH_STR "")
file(GLOB_RECURSE HFS ${CMAKE_CURRENT_LIST_DIR}/include/*.h)
foreach(HF IN LISTS HFS)
	file(MD5 ${HF} HASH)
	set(HEADER_HASH_STR "${HEADER_HASH_STR} ${HASH}")
	string(MD5 HASH ${HEADER_HASH_STR})
	#message("${HEADER_HASH_STR}")
endforeach()
set(FLAGS "-DCONFIG_HASH=${HASH}")

xpackage_init(${PACKAGE_NAME} ${PACKAGE_VERSION} ${PACKAGE_URL} "${FLAGS}" SRC_ROOT BUILD_ROOT)

if(NOT EXISTS ${BUILD_ROOT}/include)
	file(COPY ${CMAKE_CURRENT_LIST_DIR}/include DESTINATION ${BUILD_ROOT})
endif()


set(EXPORT_INCS ${BUILD_ROOT}/include  ${SRC_ROOT}/${SRC_DIR}/src/include)

include_directories(${EXPORT_INCS})

xpackage_glob_src(SRCS ${SRC_ROOT}/${SRC_DIR}/src/*.c  EXCLUDES)

xpackage_build_target(${PACKAGE_NAME} ${BUILD_ROOT} SRCS EXPORT_INCS "")



