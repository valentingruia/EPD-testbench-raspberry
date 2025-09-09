@echo off
set WORK_DIR=%CD%
SET EXT_DIR=%CD%\.vscode-ext

echo Folder: %WORK_DIR%

if exist %WORK_DIR% (
    start "" /b code %WORK_DIR% --extensions-dir %EXT_DIR%
) else (
    echo Folder %WORK_DIR% not found!
	timeout /t 2 >nul
)

exit
