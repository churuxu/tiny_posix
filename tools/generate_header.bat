@echo off

set THIS_DIR=%~dp0
cd /d "%THIS_DIR%"

set OUT_DIR=%1
if "%OUT_DIR%" == "" set OUT_DIR=..\include
if not exist "%OUT_DIR%" md  "%OUT_DIR%"
if not exist "%OUT_DIR%\sys" md  "%OUT_DIR%\sys"

set TEST_FILE=%TEMP%\t.c

set INC_FILE=posix_runtime.h

if "%CC%" == "" set CC=gcc
echo testing compiler %CC% ...
echo void a(){} > %TEST_FILE% || goto error
"%CC%" %CFLAGS%  -c %TEST_FILE% -o %TEST_FILE%.o > nul  2>&1 || goto error
echo OK


setlocal enabledelayedexpansion
for /f %%f in (headers.txt) do (
echo testing %%f ...
echo #include ^<%%f^> > %TEST_FILE% || goto error
"%CC%" %CFLAGS%  -c %TEST_FILE% -o %TEST_FILE%.o > nul  2>&1 
if "!ERRORLEVEL!" == "0" (
echo FOUND
) else (
echo NOT FOUND
echo #pragma once > "%OUT_DIR%\%%f"
echo #include "%INC_FILE%" >> "%OUT_DIR%\%%f"
)

)


goto ok

:error
echo ERROR
exit /b 1

:ok



