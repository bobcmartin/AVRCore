# AVRCore
# Arduino support package for the AVR Dx/Ex Curiosity Nano Development boards
AVRCore is a fork from the DxCore AVR Dx suport spackage created by Spence Konde
It sinplifies the menu choices the Tools Top Level directory by predefining many of the options that are
specific to the Curiousity Nano boards themselves such as memory sizes and package types.  

The DxCore package is geared towards the specific AVR DA, DB, DD , EA and EB parts themselves and exposes a large numberof options in the Tools Menu that allows customizatiom 

While a large amount of effort has been made to maintain the standard Arduino API there are some difference is the enhanced ffunctions of the new AVR family between AVRCore and DxCore which will be fully covered by this README file and additional READ files as noted

The original DxCore README file is also contained in this repo atr README_DxCore.md

# Acknowledgements 
None of AVRCore would be possible without the incredible work and passion provided by Spence Konde which provided the solid base for all the code contained in AVRCore with the majority of the unchanged from the orininal

https://github.com/SpenceKonde/DxCore

Thank you Spence for all you have done to bring the new AVR devices into the Arduino development environment

Bob Martin - Wizard of Make
January 2025

## Current Status / Updates

Release 1.1.0 - March 31 2025

Boards supported in this release
* AVR128DA48 Curiosity Nano
* AVR128DB48 Curiosity Nano
* AVR64DD32 Curiosity Nano
* AVR64EA48 Curiosity Nano


## Validation / Test Code
In addition to the examples already provided in the various subsystem menus there is a dedicated directory for all of the test/validation sketches used.
All of these sketces are located in  /testcode and are targeted towards the supporting the Curiosity Nano Explorer board which provides all of the off chip targets required.







