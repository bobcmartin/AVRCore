#  AVRCore Validation Code V1.1

All three sketches are geared specifically towards demostrting the all teh basic interfaces using the perheral targets available on the Curioisiy Nano Explorer board.  The Exploer Board isn't required to compile an download the sketches but there they still remain excellent examples on general use of all thebasic functions such as GPIO (igitalWrite), Wire, SPI and Serial.  
Basic infomation about on board LEDs and default CDC Serial ports are also illustrated in the examples 


## Boards and their part numbers

* AVR128DA48 Curiosity Nano : DM164151
* AVR128DB48 Curiosity Nano : EV35L42A
* AVR64DD32 Curiosity Nano : EVY42A
* Curiosity Nano Explorer : EV58G97A


## Basic Code Tour

All three test suites are basically the same in what they accomplish.  There are 6 main functions called in main 

  * sys_blink();
  * scan_adc_channels();
  * cylon();
  * ring_chase();
  * meatball_drop();
  * dac_test();

Serial output is available at 115200 baud in all test code



### sys_blink()
Basic LED blink task using the millis() timer

### scan_adc_channels()
This task scans two of the ADC channels that have are useful to look at when the C-Nano is plugged into the Nano Explorer board speicallt the fist two analog chanels available on the C-Nano boards down the right hand side looking with the USB connector faciing toward the top.  

  * AVR DA - PD7 and PD6
  * AVR DB - PD6 and PD3
  * AVR DD - PD7 and PD7

The special thing about these two channels is the highest numbered one is by default connected to a potentiometer that exists on the C-Nano Explorer board and the remaining channel can be connected via a jumper wire from the 1.5 Volt also on board. So one gets a variable voltage and a stable level to sample.
The results from both samples are transmitted through the USB Serial port in ADC raw counts.
Two addional function calls are also used

  * analogRead_Resolution(10);
  * analogRead_setsample(1)

All three AVR Dx parts offere 10 or 12 bit ADC blocks and also provide the ability to take multiple samples during one formal ADC cycle from 1 to 128 samples and accumulate the results. Consult the datasheets for more information but a sample depth of 16 is a good trade off between speed and accuracy. Taking multiple samples in a row like this way is an excellent way to reduce noise in the readings. The results returned from analogRead() are correctly processed to return the proper value.  
Invalid values for resolution or samples are ignored with teh defaults set to 12 bits and 1 sample

### cylon()
Basically a throwback to an old science fiction TV series with chrome villans and/or another science fiction series with a black Camero.  It's basically using the Wire (I2C) library to sweep a display of 8 LED back and forth via an I2C Prrt expander IC (MCP23008).
The implementation isn't really elegant but it gets the job done.

### ring_chase()
Another Wire (I2C) demo function using the `tinyNeoPixel` library to implement a simple RBG ring display on the circle of 8 WS2812B addressable LEDs provided on the Nano Explorer.  The stadard Neopixel librray from Adafruit won't work because it's hand colded assembly for AVR microcontrollers with clock speeds upto 16MHz.  The tinyNepPixel librray can deal with the default clock speed of 24MHz set by AVRCore

### meatball_drop()
This is the third and last Wire(I2C) demo function that used third party drivers from Adafruit for the SDD1306 OLED display which is a popular choice for small OLED displays.  The Microchip logo is known internally as the 'meatball' so this is a simple demo function that scrolls a bitmap of the Microchip meatball down the screen.

### dac_test()
This is a small demo function that uses the SPI interface to write a single cycle of a since wave to the outboard SPI DAC (Digital to Analog Convertor) that's provided on the Curiosity Nano Explorer Board (mcp4821). The cycle cycle sinve wave provuces a 1.3kHz blip through the speaker on the Explorer.  














