@echo off
setlocal
call C:\oss-cad-suite\environment.bat

pushd %~dp0

iverilog -g2012 -o sim.vvp ^
    -I ..\gateware ^
    -I ..\..\papilio_wb_register\gateware ^
    -I ..\..\papilio_spi_slave\gateware ^
    ..\gateware\pwb_spi_wb_bridge.v ^
    ..\..\papilio_wb_register\gateware\wb_register_block.v ^
    ..\..\papilio_spi_slave\gateware\fifo_sync.v ^
    tb_pwb_multi_width.v
if %errorlevel% neq 0 (
    echo Compilation failed.
    popd
    exit /b %errorlevel%
)

echo Running simulation...
vvp sim.vvp
if %errorlevel% neq 0 (
    echo Simulation failed.
    popd
    exit /b %errorlevel%
)

echo Done. VCD: tb_pwb_multi_width.vcd
popd
