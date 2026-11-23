@echo off
setlocal DisableDelayedExpansion
if not exist "%~dp0Windows\Blocks.exe" (
    echo Keep this launcher beside the complete Windows package folder.
    exit /b 2
)
if not exist "%~dp0settings.json" (
    echo Missing settings.json beside this launcher.
    exit /b 2
)
"%~dp0Windows\Blocks.exe" -dx11 -windowed -ResX=960 -ResY=600 -nosplash "-settings=%~dp0settings.json" "-UserDir=%~dp0UserData" %*
exit /b %ERRORLEVEL%
