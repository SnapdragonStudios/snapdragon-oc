@echo off
@break off
@title Generate SDOC Visual Studio Solution

set WORK_DIR=".\build"
if not exist %WORK_DIR% (
    mkdir %WORK_DIR%
)

pushd %WORK_DIR%

del CMakeCache.txt
cmake ../ -G "Visual Studio 17 2022" 
popd
