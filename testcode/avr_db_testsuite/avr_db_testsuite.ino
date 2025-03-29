/*

    AVR128DB48 Curiosity Nano test suite for AVRCore

    Bob Martin March 2025

    Serial settings for nEDBG com
    PB2/PB3 - Serial3 default
    I2C - defauls for Wire() PA2/PA3
    Curiosity Nano board LED on PB3

    hardware reqquired : AVR128DB48 Curiosity Nano  + Curiosity Nano Explorer

*/
/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
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


// Neopixel Ring Setup
tinyNeoPixel pixel_ring = tinyNeoPixel(8, PIN_PE3, NEO_GRB + NEO_KHZ800);

Adafruit_MCP23008 mcp_leds;

// global CDC uart string
char uart_str[80];


void setup() 
{
  

  pinMode(PIN_PB3, OUTPUT);       // C Nano board LED
  digitalWrite(PIN_PB3,HIGH);     // LED is active low

  Serial3.begin(115200);      // default CDC channel on Serial 1
  Serial3.println(F("\r\n AVRCore test validation suite\r\n"));
  Serial3.println("AVR128DA48 Curisoity Nano + Nano Explorer");
  
  mcp_led_init();
  dac_init();
  pixel_ring.begin();
  oled_init();

}

void loop() 
{
 
  sys_blink();
  scan_adc_channels();
  cylon();
  ring_chase();
  meatball_drop();
  dac_test();
}

void sys_blink(void)
{

  static uint32_t led_time_last = 0UL,led_time_now;
  static uint8_t led_state = false;

  led_time_now = millis();
  if(led_state)   // board led on
    {
        if((led_time_now - led_time_last) > 100UL)    // led on time expired
          {
              led_time_last = led_time_now;
              led_state = false;
              digitalWrite(PIN_PB3,HIGH);
          }
    }

  else
    {
        if((led_time_now - led_time_last) > 900UL)
          {
              led_time_last = led_time_now;
              led_state = true;
              digitalWrite(PIN_PB3,LOW);
          }
    }

}

void oled_init(void)
{

 if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
   {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
   }

}





void mcp_led_init(void)
{

  char status = mcp_leds.begin(0x25);
  
  Serial.println("Set up MCP23008 LED IO Expander");
  
  Serial.print("LED status : ");
  Serial.println(status);

  for(char pin_id = 0;pin_id < 8;pin_id++)
    {
        mcp_leds.pinMode(pin_id,OUTPUT);
        mcp_leds.digitalWrite(pin_id,HIGH);   

    }

}

void cylon(void)
{
    static uint8_t led_index = 0,led_state = false,led_dir = true;
    static unsigned long cylon_timer = 0UL;
    unsigned long now;
    uint8_t clr_index;

    now = millis();
    if(now - cylon_timer > 37UL)
    {
        cylon_timer = now;
        if(led_state)
          {
              led_state = false;
              mcp_leds.digitalWrite(led_index,HIGH);
          }
        else
          {
            led_state = true;
            mcp_leds.digitalWrite(led_index,LOW);
          }
        
        if(led_state == false)
        {
           if(led_dir)
            {
              led_index++;
              if(led_index > 4)
                {
                  led_index = 4;
                  led_dir = false;
                }
            }
           else
              {
                led_index--;
                if(led_index > 127)     // unsigned 8 bit gone negative
                  {
                    led_index = 0;
                    led_dir = true;
                  }
              }
        }
  }

}

void scan_adc_channels(void)
{

  static uint32_t last_scan_time = 0UL, current_scan_time;
  int16_t adc_sample;
  static uint32_t scan_count = 0UL;

  current_scan_time = millis();

  if((current_scan_time - last_scan_time) > 2000)   // 2 second scans
  {

      last_scan_time = current_scan_time;
      scan_count++;
      analogRead_Resolution(10);       // 10 bit resolution
      analogRead_setsample(1);        // single sample mode

      sprintf(uart_str,"adc scan %lu\r\n",scan_count);
      Serial3.print(uart_str);

      adc_sample = analogRead(PIN_PD6);
      sprintf(uart_str,"analog PD6 : %d\r\n",adc_sample);
      Serial3.print(uart_str);
 
      adc_sample = analogRead(PIN_PD3);
      sprintf(uart_str,"analog PD3 : %d\r\n",adc_sample);
      Serial3.print(uart_str);

  }
 
}

// SPI test code using DAC on Curiosity Explorer board
// defualt SPI channel created by SPI ojbect is correct for this implementation

#define DAC_CS    PIN_PE2       // IO 26 on C Nano Explorer board

void dac_init(void)
{

  SPI.begin();
  pinMode(DAC_CS,OUTPUT);       // set up DAC Chip Select
  digitalWrite(DAC_CS,HIGH);    
  
}


// tick speaker @ 1.3 kHz for a full sinewave cycle
void dac_test(void)
{

    static unsigned long last_beep_time = 0UL,now;
    
    int array_index, sample;  
    uint8_t high_byte,low_byte;

    now = millis();
    if((now - last_beep_time) > 2000)    // beep every three seconds
    {

        last_beep_time = now;
        Serial3.println(">> beep");
        for(array_index = 0;array_index < 199;array_index += 4)       // array step 4 gets about 1.3 kHz saw tooth
          {

            sample = sine_wave[array_index] + 2048;       // add DC offset
            high_byte = (uint8_t)(sample >> 8);
            high_byte |= 0x30;                            // set gain X1 bit
            low_byte = (uint8_t)(sample & 0xFF);

            digitalWrite(DAC_CS,LOW);
            SPI.transfer(high_byte);
            SPI.transfer(low_byte);
            digitalWrite(DAC_CS,HIGH);
            // delay(1);

        }

    }

}


unsigned long color_grid[3] = {0x00007f,0x007f00,0x7f0000};

void ring_chase(void)
{
  static unsigned long ring_timer = 0UL;
  unsigned long now;
  static uint8_t pixel_index = 0,color_index = 0;


   now = millis();
   if(now - ring_timer > 211UL)
   {

      ring_timer = now;
      pixel_ring.clear();
      pixel_ring.show();    
      pixel_ring.setPixelColor(pixel_index,color_grid[color_index]);
      pixel_ring.show();
      pixel_index++;
      if(pixel_index > 7)
             pixel_index = 0;
      color_index++;
      if(color_index > 2)
        color_index = 0;
       
   }

}

#define MEATBALL_H    51
#define MEATBALL_W    52

const unsigned char meatball [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xf0, 0xff, 0xff, 0xfe, 0x03, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xf0, 0x00, 0x3f, 0xff, 
	0xf0, 0xff, 0xff, 0x80, 0x00, 0x0e, 0x1f, 0xf0, 0xff, 0xfe, 0x00, 0x00, 0x06, 0x1f, 0xf0, 0xff, 
	0xfc, 0x00, 0x00, 0x02, 0x1f, 0xf0, 0xff, 0xf0, 0x00, 0x00, 0x02, 0x3f, 0xf0, 0xff, 0xe0, 0x00, 
	0x00, 0x01, 0xff, 0xf0, 0xff, 0xc0, 0x00, 0x00, 0x00, 0xff, 0xf0, 0xff, 0xc0, 0x00, 0x00, 0x00, 
	0xff, 0xf0, 0xff, 0x80, 0x10, 0x00, 0x20, 0x7f, 0xf0, 0xff, 0x00, 0x3c, 0x00, 0x70, 0x7f, 0xf0, 
	0xff, 0x00, 0x7c, 0x00, 0xf8, 0x3f, 0xf0, 0xfe, 0x00, 0x7e, 0x01, 0xf8, 0x1f, 0xf0, 0xfe, 0x00, 
	0xff, 0x01, 0xfc, 0x1f, 0xf0, 0xfc, 0x01, 0xff, 0x03, 0xfe, 0x0f, 0xf0, 0xfc, 0x00, 0xff, 0x83, 
	0xfe, 0x07, 0xf0, 0xfc, 0x00, 0xff, 0xc1, 0xff, 0x07, 0xf0, 0xfc, 0x00, 0x7f, 0xc1, 0xff, 0x03, 
	0xf0, 0xf8, 0x00, 0x7f, 0xe0, 0xff, 0x81, 0xf0, 0xf8, 0x08, 0x3f, 0xf0, 0xff, 0xc1, 0xf0, 0xf8, 
	0x1c, 0x1f, 0xf0, 0x7f, 0xc0, 0xf0, 0xf8, 0x3e, 0x1f, 0xf8, 0x3f, 0xe0, 0x70, 0xf8, 0x3e, 0x0f, 
	0xf8, 0x3f, 0xf0, 0x70, 0xf8, 0x7f, 0x07, 0xfc, 0x1f, 0xf0, 0x70, 0xfc, 0xff, 0x83, 0xfe, 0x0f, 
	0xf8, 0x70, 0xfd, 0xff, 0x83, 0xfe, 0x0f, 0xfc, 0xf0, 0xff, 0xff, 0x01, 0xff, 0x07, 0xfc, 0xf0, 
	0xff, 0xff, 0x01, 0xff, 0x03, 0xff, 0xf0, 0xff, 0xfe, 0x00, 0xfe, 0x03, 0xff, 0xf0, 0xff, 0xfc, 
	0x00, 0xfc, 0x01, 0xff, 0xf0, 0xff, 0xf8, 0x00, 0x78, 0x00, 0xff, 0xf0, 0xff, 0xf8, 0x00, 0x38, 
	0x00, 0xff, 0xf0, 0xff, 0xf0, 0x00, 0x10, 0x00, 0x7f, 0xf0, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x3f, 
	0xf0, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0xff, 
	0xf8, 0x00, 0x00, 0x00, 0x7f, 0xf0, 0xff, 0xfe, 0x00, 0x00, 0x01, 0xff, 0xf0, 0xff, 0xff, 0x80, 
	0x00, 0x07, 0xff, 0xf0, 0xff, 0xff, 0xe0, 0x00, 0x1f, 0xff, 0xf0, 0xff, 0xff, 0xfc, 0x00, 0xff, 
	0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xf0
};





void meatball_drop(void)
{
  static unsigned long meatball_timer = 0UL;
  unsigned long now;
  static int8_t x_pos = ((display.width() - MEATBALL_W ) / 2), y_pos = -MEATBALL_H,dx = 0;

  now = millis();
  if(now - meatball_timer > 101UL)
  {
     meatball_timer = now;
     display.clearDisplay();
     display.drawBitmap(x_pos,y_pos, meatball, MEATBALL_W, MEATBALL_H, 1);
     display.display();

     y_pos += 2;
       if(y_pos >= display.height())
          {
            y_pos = -MEATBALL_H;
            x_pos = ((display.width() - MEATBALL_W ) / 2);
            dx = (random(1,10) > 5 ? -15 : 15);
            x_pos += dx;
          }
  }

}




