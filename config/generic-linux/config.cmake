
#用于编译出的文件，不依赖 libstdc++.so 不依赖libgcc_s.so

if(NOT EXISTS ${CMAKE_BINARY_DIR}/libstdc++.a)
	execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libstdc++.a OUTPUT_VARIABLE LIBSTDCXXPATH OUTPUT_STRIP_TRAILING_WHITESPACE)
	execute_process(COMMAND ln -s ${LIBSTDCXXPATH} WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
endif()

set(LDFLAGS "-L. -static-libgcc")