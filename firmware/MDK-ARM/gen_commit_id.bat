@echo off
setlocal enabledelayedexpansion

REM 获取 Git commit ID
for /f "delims=" %%i in ('git rev-parse HEAD 2^>nul') do set GIT_COMMIT=%%i

REM 检查 Git 命令是否成功
if "%GIT_COMMIT%"=="" (
    set GIT_COMMIT=unknown
)

REM 生成 commit_id.h 文件
echo #ifndef COMMIT_ID_H > "..\Core\Inc\commit_id.h"
echo #define COMMIT_ID_H >> "..\Core\Inc\commit_id.h"
echo. >> "..\Core\Inc\commit_id.h"
echo #define GIT_COMMIT_ID "%GIT_COMMIT%" >> "..\Core\Inc\commit_id.h"
echo. >> "..\Core\Inc\commit_id.h"
echo #endif >> "..\Core\Inc\commit_id.h"

:end
echo Git commit ID: %GIT_COMMIT%