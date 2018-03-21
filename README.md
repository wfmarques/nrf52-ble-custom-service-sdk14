# nrf52-ble-custom-service-sdk14
Custom BLE service example for NRF52 SDK 14 using Timers

You can set the SDK. Compiler and NRJJPROG variables directly in the Makefile or using another external tool like DIRENV:

SDK_ROOT := /YOUR_PATH/nRF5_SDK_14.0.0_3bcc1f7
GNU_INSTALL_ROOT := /YOUR_PATH/arm-none-eabi-4.9
nrfjprog := /YOUR_PATH/nRF5x-Command-Line-Tools_9_0_0_OSX/nrfjprog/nrfjprog

Make rules:

make (Compile)
make clean 
make flash (Use nrfjprog to flash in NRF52 Devboard)
