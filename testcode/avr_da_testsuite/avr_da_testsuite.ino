/*

    AVR128DA48 C-Nano test suite for AVRCore

    Bob Martin Dec 2024

    Serial settings for nEDBG com
    PC0/PC1 - Serial1 default
    Curiosity Nano board LED on PC6

    hardware reqquired : AVR128DA48 Curiosity Nano 


*/


#include "SPI.h"
#include "sine_wave.h"
#include <tinyNeoPixel.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_MCP23008.h"

// OLED screen 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// NoepIxel stuff

tinyNeoPixel pixel_ring = tinyNeoPixel(8, PIN_PC3, NEO_GRB + NEO_KHZ800);

// global CDC uart string
char uart_str[80];


void setup() 
{
  
  
  pinMode(PIN_PC6, OUTPUT);       // C Nano on board LED

  Serial1.begin(115200);      // default CDC channel on Serial 1
  Serial1.println(" AVRCore test validation suite");
  Serial1.println("AVR128DA48 Curisoity Nano + Nano Explorer");

  dac_init();
  
  pwm_test();



}


// arguments to analogReference() for AVR DA 
// enum adc_refsel {REF_VDD = 0x00,REF_EXTVREF = 0x02,REF_1v024=0x04,REF_2V048=0x05,REF_4V096=0x06,REF_2V500 = 0x07 };
enum adc_refsel {REF_1v024=0x00, REF_2V048=0x01,REF_4V096=0x02,REF_2V500 = 0x03, REF_VDD = 0x05, REF_EXTVREF = 0x06 };
#define da_digitalPinToAnalogInput(p)        (((p) > PIN_PC7 && (p) < PIN_PF0) ? ((p) - PIN_PD0) : ((p) < PIN_PF6 ? ((p) - 18) : NOT_A_PIN))

void config_adc_test(void)
{

  uint8_t mux_pos;

  analogReadResolution(12);       // go 12 bit 
  analogRead_setsample(1);
  analogReference(REF_VDD);
  mux_pos = da_digitalPinToAnalogInput(PIN_PD7);
  sprintf(uart_str,"pin PD7 %d muxpos %d\r\n",PIN_PD7,mux_pos);
  Serial1.print(uart_str);
  
  mux_pos = da_digitalPinToAnalogInput(PIN_PD6);
  sprintf(uart_str,"pin PD6 %d muxpos %d\r\n",PIN_PD6,mux_pos);
  Serial1.print(uart_str);

}





uint32_t scan_count = 0;

void loop() 
{
  

  scan_count++;
  
  digitalWrite(PIN_PC6, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  
  // scan_adc_channels();
  dac_test();



  digitalWrite(PIN_PC6, HIGH);   // turn the LED off by making the voltage LOW
  delay(900);                      // wait for a second
}


void scan_adc_config(void)
{
  uint8_t ctrla, muxpos;


  ctrla = ADC0.CTRLA;
  muxpos = ADC0.MUXPOS;
 
  sprintf(uart_str,"ctrla %x muxpos %x \r\n",ctrla,muxpos);
  Serial1.print(uart_str);

}

void scan_adc_channels(void)
{

  int16_t adc_sample;

  sprintf(uart_str,"============== adc scan %lu  =================\r\n",scan_count);
  Serial1.print(uart_str);

  adc_sample = analogRead(PIN_PD7);
  sprintf(uart_str,"analog PD7 : %d\r\n",adc_sample);
  Serial1.print(uart_str);
  scan_adc_config();

  adc_sample = analogRead(PIN_PD6);
  sprintf(uart_str,"analog PD6 : %d\r\n",adc_sample);
  Serial1.print(uart_str);
  scan_adc_config();
  
}




// SPI test code using DAC on Curioisty Explorer board
// defualt SPI channel created by SPI ojbect is correct for this implementation


#define DAC_CS    PIN_PE2       // IO 26 on C Nano Explorer board



void dac_init(void)
{

  SPI.begin();
  // SPI.beginTransaction(100000,MSBFIRST,SPI_MODE0);
  pinMode(DAC_CS,OUTPUT);       // set up DAC Chip Select
  digitalWrite(DAC_CS,HIGH);    
  
}


// extern uint16_t sin_wave[256];

void dac_test(void)
{

    int array_index, sample;
    
    uint8_t high_byte,low_byte;


    for(array_index = 0;array_index < 256;array_index++)
    {

        sample = sin_wave[array_index];
        high_byte = (uint8_t)(sample >> 8);
        high_byte |= 0x30;          // set gain X1 bit
        low_byte = (uint8_t)(sample & 0xFF);

        digitalWrite(DAC_CS,LOW);
        SPI.transfer(high_byte);
        SPI.transfer(low_byte);
        digitalWrite(DAC_CS,HIGH);
        delay(1);

    }

}


// drive the single RGB LED


#define PWM1  PIN_PD2
#define PMM2  PIN_PD1
#define PWM3  PIN_PD0


void pwm_test2(void)
{

    pinMode(PIN_PD2,OUTPUT);
    digitalWrite(PIN_PD2,LOW);

  while(1)
  {

      digitalWrite(PIN_PD2,HIGH);
      delay(500);
      digitalWrite(PIN_PD2,LOW);
      delay(500);
  }

}




void pwm_test(void)
{
    
    while(1)
    {
      analogWrite(PWM1,200);
      delay(500);
      analogWrite(PWM1,0);
      delay(500);
    }

}





