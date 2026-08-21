@echo off
setlocal
set "APP=%~dp0artifacts\precision-v5\ZedAnchorHoleInspection.exe"

if not exist "%APP%" (
    echo Latest build was not found:
    echo %APP%
    pause
    exit /b 1
)

start "" "%APP%"
