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

echo [1/2] Configuring...
"%CMAKE%" -S "%~dp0." -B "%BUILD_DIR%" -G Ninja ^
    -DCMAKE_MAKE_PROGRAM="%NINJA%" ^
    -DCMAKE_PREFIX_PATH=C:\msys64\ucrt64
if %ERRORLEVEL% neq 0 ( echo Configure failed. & exit /b %ERRORLEVEL% )

echo [2/2] Building...
"%CMAKE%" --build "%BUILD_DIR%"
if %ERRORLEVEL% neq 0 ( echo Build failed. & exit /b %ERRORLEVEL% )

echo.
echo Build successful: %BUILD_DIR%\SerialMonitor.exe
