@echo off
setlocal

set MSYS2=C:\msys64\ucrt64\bin
set CMAKE=%MSYS2%\cmake.exe
set NINJA=%MSYS2%\ninja.exe
set BUILD_DIR=%~dp0build

if not exist "%CMAKE%" (
    echo ERROR: cmake not found at %CMAKE%
    echo Run in MSYS2 UCRT64: pacman -S mingw-w64-ucrt-x86_64-cmake
    exit /b 1
)
if not exist "%NINJA%" (
    echo ERROR: ninja not found at %NINJA%
    echo Run in MSYS2 UCRT64: pacman -S mingw-w64-ucrt-x86_64-ninja
    exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

echo [1/7] Configuring...
"%CMAKE%" -S "%~dp0." -B "%BUILD_DIR%" -G Ninja ^
    -DCMAKE_MAKE_PROGRAM="%NINJA%" ^
    -DCMAKE_PREFIX_PATH=C:\msys64\ucrt64 ^
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
if %ERRORLEVEL% neq 0 ( echo Configure failed. & exit /b %ERRORLEVEL% )

echo [2/7] Building...
"%CMAKE%" --build "%BUILD_DIR%"
if %ERRORLEVEL% neq 0 ( echo Build failed. & exit /b %ERRORLEVEL% )

echo [3/7] Attitude handedness self-check (SWE1-GUI-001 evidence, headless)...
"%CMAKE%" --build "%BUILD_DIR%" --target attitude_selfcheck
if %ERRORLEVEL% neq 0 ( echo Attitude self-check build failed. & exit /b %ERRORLEVEL% )
"%BUILD_DIR%\attitude_selfcheck.exe"
if %ERRORLEVEL% neq 0 ( echo Attitude self-check FAILED - handedness math regression. & exit /b %ERRORLEVEL% )

echo [4/7] Position panel self-check (SWE1-GUI-006/-007/-008 evidence, headless)...
"%CMAKE%" --build "%BUILD_DIR%" --target position_selfcheck
if %ERRORLEVEL% neq 0 ( echo Position self-check build failed. & exit /b %ERRORLEVEL% )
"%BUILD_DIR%\position_selfcheck.exe"
if %ERRORLEVEL% neq 0 ( echo Position self-check FAILED - position math regression. & exit /b %ERRORLEVEL% )

echo [5/7] Diagnostics word-grouping self-check (headless)...
"%CMAKE%" --build "%BUILD_DIR%" --target diag_selfcheck
if %ERRORLEVEL% neq 0 ( echo Diagnostics self-check build failed. & exit /b %ERRORLEVEL% )
"%BUILD_DIR%\diag_selfcheck.exe"
if %ERRORLEVEL% neq 0 ( echo Diagnostics self-check FAILED - word-grouping regression. & exit /b %ERRORLEVEL% )

echo [6/7] Motors slider-mapping self-check (headless)...
"%CMAKE%" --build "%BUILD_DIR%" --target motor_selfcheck
if %ERRORLEVEL% neq 0 ( echo Motor self-check build failed. & exit /b %ERRORLEVEL% )
"%BUILD_DIR%\motor_selfcheck.exe"
if %ERRORLEVEL% neq 0 ( echo Motor self-check FAILED - slider/DShot mapping regression. & exit /b %ERRORLEVEL% )

echo [7/7] Motors tab safety self-check (widget level, offscreen)...
"%CMAKE%" --build "%BUILD_DIR%" --target motor_view_selfcheck
if %ERRORLEVEL% neq 0 ( echo Motor view self-check build failed. & exit /b %ERRORLEVEL% )
"%BUILD_DIR%\motor_view_selfcheck.exe"
if %ERRORLEVEL% neq 0 ( echo Motor view self-check FAILED - Motors tab safety regression. & exit /b %ERRORLEVEL% )

echo.
echo Build successful: %BUILD_DIR%\SerialMonitor.exe
