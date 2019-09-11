set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_C_COMPILER "i686-w64-mingw32-gcc" )
set(CMAKE_CXX_COMPILER "i686-w64-mingw32-g++" )
set(CMAKE_AR "i686-w64-mingw32-gcc-ar" )


# 不依赖 libgcc libstdc++ libwinpthread
if(NOT EXISTS ${CMAKE_BINARY_DIR}/libpthread.a)
	execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libwinpthread.a OUTPUT_VARIABLE LIBPATH OUTPUT_STRIP_TRAILING_WHITESPACE)
	file(COPY ${LIBPATH} DESTINATION "${CMAKE_BINARY_DIR}")
	file(RENAME "${CMAKE_BINARY_DIR}/libwinpthread.a" "${CMAKE_BINARY_DIR}/libpthread.a")  
endif()

set(LDFLAGS "-L. -static-libgcc -static-libstdc++")

