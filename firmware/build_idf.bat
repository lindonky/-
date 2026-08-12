@echo off
setlocal

rem Git Bash may inject these variables; ESP-IDF's Windows launcher rejects them.
set "MSYSTEM="
set "MSYS="

rem Prefer an existing ESP-IDF environment. This fallback matches the original
rem development machine and can be overridden by setting IDF_PATH beforehand.
if not defined IDF_PATH set "IDF_PATH=C:\Espressif\frameworks\esp-idf-v5.1.2"
if not defined IDF_TOOLS_PATH set "IDF_TOOLS_PATH=C:\Espressif"
if not defined IDF_PYTHON_ENV_PATH if exist "%IDF_TOOLS_PATH%\python_env\idf5.1_py3.11_env\Scripts\python.exe" set "IDF_PYTHON_ENV_PATH=%IDF_TOOLS_PATH%\python_env\idf5.1_py3.11_env"

if not exist "%IDF_PATH%\export.bat" (
    echo ERROR: ESP-IDF was not found at "%IDF_PATH%".
    echo Set IDF_PATH to an ESP-IDF v5.1.2 installation and retry.
    exit /b 1
)

call "%IDF_PATH%\export.bat" >nul 2>&1
if errorlevel 1 exit /b %ERRORLEVEL%

pushd "%~dp0"
if defined IDF_PYTHON_ENV_PATH if exist "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" (
    "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" "%IDF_PATH%\tools\idf.py" %*
) else (
    python.exe "%IDF_PATH%\tools\idf.py" %*
)
set "BUILD_RESULT=%ERRORLEVEL%"
popd

exit /b %BUILD_RESULT%
