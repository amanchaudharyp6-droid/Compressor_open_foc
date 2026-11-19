:: Deletes All files in the Build and Rebuilds 
::(Hidden, System, and Read-Only Files are Not Affected) 
::
@ECHO OFF 
@REM  rd /s /q Build
@REM  mkdir Build
cd .\Build\
cmake -G "MinGW Makefiles" --toolchain ../arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_DEBUG=0 .. 
cmake --build . -- -j16
cd ..
python patch_image_header.py Build/Pegasus_v4-App1.bin
python patch_image_header.py Build/Pegasus_v4-App2.bin

@REM --verbose