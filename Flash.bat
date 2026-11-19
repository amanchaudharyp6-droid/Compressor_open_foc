@ECHO OFF

@REM openocd.exe -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program Build/Pegasus_v4.bin reset exit 0x8000000"
openocd.exe -f interface/stlink.cfg -f target/stm32h7x.cfg -c "init; reset halt; stm32h7x mass_erase 0; exit"
openocd.exe -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program Build/Pegasus_v4-Bootloader.bin reset verify exit 0x8000000"
openocd.exe -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program Build/Pegasus_v4-App1.bin reset verify exit 0x8040000"
openocd.exe -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program Build/Pegasus_v4-App2.bin reset verify exit 0x8080000"

 @REM 0x08000000"