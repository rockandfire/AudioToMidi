To successfully build .uf2 for pico programming, ensure the following packages are installed:
cmake
build-essential
gcc-arm-none-eabi

If not on Linux, use Ubuntu on WSL to install the packages.
Download Hathach's TinyUSB and the Pico SDK at:
https://github.com/hathach/tinyusb
https://github.com/raspberrypi/pico-sdk

and place the contents of TinyUSB into pico-sdk-master/lib/tinyusb/
This will enable USB support between the Pico and a COM shell.

Set the correct path to the Pico SDK in the CMake cache file for the FFT project, and everything should compile correctly.
Create a build directory, navigate to it, and run cmake ../
Finally, run make

A .uf2 file should have been generated in the parent folder.
Connect the Pico in bootloader mode to program.
