# AVRCore
## Arduino support package for the AVR Dx/Ex Curiosity Nano Development boards
AVRCore is a fork from the `DxCore` AVR Dx support package created by Spence Konde
It simplifies the menu choices the Tools Top Level directory by predefining many of the options that are
specific to the Curiosity Nano boards. This prevents configuration errors that may prevent a perfectly valid target sketch to compile.  


The DxCore package is geared towards every specific AVR Dx or Ex part that is available  and presents a large number of options that allows customization to a very specific device.  

There are even options presented, such as clock speed that exceed the operational specification of the AVR parts themselves which have also been removed from AVRCore.

A large amount of effort has been made to maintain the standard Arduino API there are some differences specifically relating to how the  enhanced functions of peripheral like the ADC and PWM (analogWrite) that will be tuned specifically to the Curiosity Nano boards themselves. 

Optiboot support has been removed in AVRCore, not that there is anything wrong with Optiboot but every Curiosity Nano board comes equipped with it's own programmer / debugger so there's no need for a bootloader and the flash space it uses.


## Acknowledgements 
None of the AVRCore package would be possible without the incredible work and passion provided by Spence Konde which provided the solid base for all the code contained in AVRCore with the majority of the unchanged from the original. Go visit his Tindie store and buy some cool stuff.

`https://github.com/SpenceKonde/DxCore`

Thank you Spence for all you have done to bring the new AVR devices into the Arduino development environment

Bob Martin - Wizard of Make
March 2025

## Update Cycle
The intent is to have major updates on a quarterly cycle (3 months).  Latest code will always be available on the master branch but there are no guarentees that the code will work or even compile correctly.

No more than two versions will be available in the official install package.  This will help simplifying pull requests and issue reporting to three sources, last release, current release and master branch.

## Current Status / Version Updates

### Release 1.1 - March 31 2025

Boards supported in this release
* AVR128DA48 Curiosity Nano
* AVR128DB48 Curiosity Nano
* AVR64DD32 Curiosity Nano

There will be mini updates 1.11, 1.12 along the way as I sort out and clean up the offical 1.1 release

### Release 1.2 - Scheduled for July 1 2025

* upcoming features
* add AVR64DU + USB support - this is a big one
* refactor analogWrite - dedicate TC0 or TCA1 to basic PWM tasks including Servo library

## Validation / Test Code
In addition to the examples already provided in the various subsystem menus there is a dedicated directory for all of the test/validation sketches used.
All of these sketches are located in  /testcode and are targeted towards the supporting all three of the DA/DB and DD Curiosity boards

## AVRCore Support
While Every one is free to submit pull requests to me but for the first few release cycles it will probably be better to just email me directly at 
`bob.martin@microchip.com`
Please try to prefix the subject line with "AVRCore-" so my email filter can deposit it into the correct inbox subfolder

If you want to contribute to the project please let me know, I will be more than happy to ship you a set of boards for free to help that effort along.

## OS Support
For now AVRCore is only supported on Windows. The DxCore installation has links to a wider range of compilers for the Linux and MacOs environments but are not guaranteed to work because of specific AVR Microcontroller device support.  One of the bigger updates coming in the following year is to change teh build system such that it uses the Device Pack system used by Microchip Studio and MPLAB X which removes teh need to add specific device support directly into the compiler. That will all the compiler builds to focus on the AVR-GCC specific changes rather than having to keep updating the tool chain with new device header files.


