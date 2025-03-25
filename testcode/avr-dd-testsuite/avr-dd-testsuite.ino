
/*
    AVR6DD32 test suite v1.0
    v 1.0 Jan 2025
    Bob Martin

    Test franmework for AVRCore support

*/

// ea translation utilitties from pins_arduino.h EA 48


#define da_digitalPinToAnalogInput(p)        (((p) > PIN_PC7 && (p) < PIN_PF0) ? ((p) - PIN_PD0) : ((p) < PIN_PF6 ? ((p) - 18) : NOT_A_PIN))
#define da_analogChannelToDigitalPin(p)      (((p) < 12) ? ((p) + PIN_PD0) : (((p) < 16 || p > 21) ? NOT_A_PIN : ((p) + PIN_PF0 - 16)))
#define da_analogInputToDigitalPin(p)                        analogChannelToDigitalPin((p) & 0x7F)
#define da_digitalOrAnalogPinToDigital(p)    (((p) & 0x80) ? analogChannelToDigitalPin((p) & 0x7f) : (((p) <= NUM_DIGITAL_PINS) ? (p) : NOT_A_PIN))

#define portToPinZero(port)               (((port) < PF) ? (((port) * 8) - (((port) > 1) ? 2 : 0) : ((port) == PF ? PIN_PF0 : NOT_A_PIN))



#define PIN_PC7 21

#define dd_digitalPinToAnalogInput(p)           ((p) >= PIN_PD0 ? (((p) < PIN_PF0)   ? ((p) - PIN_PD0) : ((p) < PIN_PF6)   ? ((p) -  4)      : NOT_A_PIN)  : (((p) > PIN_PA1 && (p) < PIN_PC0)      ? ((p) + 20) : NOT_A_PIN))
#define dd_analogChannelToDigitalPin(p)         ((p) > 27 ? NOT_A_PIN : ((p) < 8     ? ((p) + PIN_PD0) : ((p) > 21)        ? ((p) - 20)      : ((p) > 15   ? ((p) + 4) : NOT_A_PIN)))
  

#define ea_digitalPinToAnalogInput(p)        (((p) > PIN_PC7 && (p) < PIN_PF0) ? ((p) - PIN_PD0) : ((p) < PIN_PF6 ? ((p) - 18) : NOT_A_PIN))
#define ea_analogChannelToDigitalPin(p)      (((p) < 12) ? ((p) + PIN_PD0) : (((p) < 16 || p > 21) ? NOT_A_PIN : ((p) + PIN_PF0 - 16)))
#define ea_analogInputToDigitalPin(p)                        analogChannelToDigitalPin((p) & 0x7F)
#define ea_digitalOrAnalogPinToDigital(p)    (((p) & 0x80) ? analogChannelToDigitalPin((p) & 0x7f) : (((p)<=NUM_DIGITAL_PINS) ? (p) : NOT_A_PIN))

#define ea_portToPinZero(port)               (((port) < PF) ? (((port) * 8) - (((port) > 1) ? 2 : 0) : ((port) == PF ? PIN_PF0 : NOT_A_PIN))


// Arduino pin# to muxpos values for avr64ea48, return 255 if invalid channel
uint8_t ea_adcmux[40] = {
                              0xFF, // 0
                              0xFF, // 1
                              0x16, // 2
                              0x17, // 3                       
                              0x18, // 4
                              0x19, // 5
                              0x1A, // 6
                              0x1B, // 7
                              0xFF, // 8 
                              0xFF, // 9
                              0xFF, // 10
                              0xFF, // 11
                              0xFF, // 12
                              0xFF, // 13
                              0x1C, // 14
                              0x1D, // 15
                              0x1E, // 16
                              0x1F, // 17
                              0xFF, // 18
                              0xFF, // 19
                              0xFF, // 20
                              0xFF, // 21
                              0x00, // 22
                              0x01, // 23
                              0x02, // 24
                              0x03, // 25
                              0x04, // 26
                              0x05, // 27
                              0x06, // 28
                              0x07, // 29,
                              0x08, // 30
                              0x09, // 31
                              0x0A, // 32
                              0x0B, // 33
                              0xFF, // 34 PF0 32k crystal
                              0xFF, // 35 PF1 32k crystal
                              0x12, // 36
                              0x13, // 37
                              0x14, // 38
                              0x15  // 39
                          } ;

uint8_t dd_adcmux[26] = {
                              0xFF, // 0
                              0xFF, // 1
                              0x16, // 2
                              0x17, // 3
                              0x18, // 4
                              0x19, // 5
                              0x1A, // 6
                              0x1B, // 7
                              0xFF, // 8 mvio lockout
                              0xFF, // 9 mvio lockoput
                              0xFF, // 10 mvio lockout
                              0xFF, // 11 mvio lockout
                              0xFF, // 12 (not a pin)
                              0x01, // 13
                              0x02, // 14
                              0x03, // 15
                              0x04, // 16
                              0x06, // 17
                              0x07, // 18
                              0x08, // 19
                              0xFF, // PF0 32k XTAL
                              0xFF, // PF1 32k XTAL
                              0x12, // 22
                              0x13, // 23
                              0x14, // 24
                              0x15, // 25
};










char uart_str[80];

void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(PIN_PF5, OUTPUT);

  Serial.swap(3);
  Serial.begin(115200);

  Serial.println(">>>> AVRCore test harness1 ");

  // analogReadResolution(12);       // go 12 bit 

  // translate_analogpins();

  // while(1);
  
}

// the loop function runs over and over again forever
// on board led on DD C-Nano on PIN_PF5 active low

uint32_t scan_count = 0;

void loop() 
{
  
  int16_t adc_sample;

  scan_count++;

  sprintf(uart_str,"============== adc scan %lu  =================\r\n",scan_count);
  Serial.print(uart_str);
  digitalWrite(PIN_PF5, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  
  analogRead_setsample(4);
  adc_sample = analogRead(PIN_PD7);
  sprintf(uart_str,"analog PD7 : %d\r\n",adc_sample);
  Serial.print(uart_str);
  scan_adc_config();

  adc_sample = analogRead(PIN_PD6);
  sprintf(uart_str,"analog PD6 : %d\r\n",adc_sample);
  Serial.print(uart_str);
  scan_adc_config();
  
  adc_sample = analogRead_diff(PIN_PD7,PIN_PD6);
  sprintf(uart_str,"analog PD7-PD6 : %d\r\n",adc_sample);
  Serial.print(uart_str);
  scan_adc_config();
  
  digitalWrite(PIN_PF5, HIGH);   // turn the LED off by making the voltage LOW
  delay(1900);                      // wait for a second
}


void scan_adc_config(void)
{
  uint8_t ctrla, muxpos,muxneg;


  ctrla = ADC0.CTRLA;
  muxpos = ADC0.MUXPOS;
  muxneg = ADC0.MUXNEG;

  sprintf(uart_str,"ctrla %x muxpos %x muxneg %x\r\n",ctrla,muxpos,muxneg);
  Serial.print(uart_str);

}



void translate_analogpins(void)
{
    uint8_t pin_number,mux_pos1, mux_pos2;

    Serial.println(">> AVR64EA48 mux translation");
    // plot out AVR64EA48    
    for(pin_number = 0; pin_number < 39; pin_number++)
    {

        mux_pos1 = ea_digitalPinToAnalogInput(pin_number);
        mux_pos2 = ea_adcmux[pin_number];
        sprintf(uart_str,"pin %d muxpos1 %X   muxpos2 %X\r\n",pin_number,mux_pos1,mux_pos2);
        Serial.print(uart_str);

    }

    // plot out AVR64DD32

    Serial.println(">> AVR62DD32mux translation");
    for(pin_number = 0; pin_number < 26; pin_number++)
    {

        mux_pos1 = dd_digitalPinToAnalogInput(pin_number);
        mux_pos2 = dd_adcmux[pin_number];
        sprintf(uart_str,"pin %d muxpos1 %X   muxpos2 %X\r\n",pin_number,mux_pos1,mux_pos2);
        Serial.print(uart_str);

    }


}


// Code generated by MCHP Chatbot
/*
int dd_digitalPinToAnalogInput(int p) {
    if (p >= PIN_PD0) {
        if (p < PIN_PF0) {
            return p - PIN_PD0;
        } else if (p < PIN_PF6) {
            return p - 4;
        } else {
            return NOT_A_PIN;
        }
    } else {
        if (p > PIN_PA1 && p < PIN_PC0) {
            return p + 20;
        } else {
            return NOT_A_PIN;
        }
    }
}
*/









