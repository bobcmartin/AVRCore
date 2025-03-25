/*

    AVR 128DB48 C-Nano test suite for AVRCore

    Bob Martin Dec 2024

    Serial settings for nEDBG com on AVR128DB48

    PB0 / PB1
    Serial3 default
    board LED on PB3

    C-Nano Explorer ADC test points PD6 (pot) and PD3 (1.5V ref in)

*/

// arguments to analogReference() for AVR DA 
// enum adc_refsel {REF_VDD = 0x00,REF_EXTVREF = 0x02,REF_1v024=0x04,REF_2V048=0x05,REF_4V096=0x06,REF_2V500 = 0x07 };
enum adc_refsel {REF_1v024=0x00, REF_2V048=0x01,REF_4V096=0x02,REF_2V500 = 0x03, REF_VDD = 0x05, REF_EXTVREF = 0x06 };

#define DB_LED  PIN_PB3




#define db_digitalPinToAnalogInput(p)        (((p) > PIN_PC7 && (p) < PIN_PF0) ? ((p) - PIN_PD0) : ((p) < PIN_PF6 ? ((p) - 18) : NOT_A_PIN))



char uart_str[80];

void setup() 
{
  
  uint8_t mux_pos;
  
  pinMode(DB_LED, OUTPUT);
  digitalWrite(DB_LED,LOW);
  
  Serial3.begin(115200);      // default CDC channel on Serial 1

  Serial3.println(">>>> AVRCore test suite for AVR DU");

  // analogReadResolution(12);       // go 12 bit 
  // analogRead_setsample(1);
  analogReference(REF_VDD);
  mux_pos = db_digitalPinToAnalogInput(PIN_PD6);
  sprintf(uart_str,"pin PD7 %d muxpos %d\r\n",PIN_PD6,mux_pos);
  Serial3.print(uart_str);
  
  mux_pos = db_digitalPinToAnalogInput(PIN_PD3);
  sprintf(uart_str,"pin PD6 %d muxpos %d\r\n",PIN_PD3,mux_pos);
  Serial3.print(uart_str);


}


uint32_t scan_count = 0;

void loop() 
{
  

  scan_count++;
  
  digitalWrite(DB_LED, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  
  scan_adc_channels();


  digitalWrite(DB_LED, HIGH);   // turn the LED off by making the voltage LOW
  delay(900);                      // wait for a second
}


void scan_adc_config(void)
{
  uint8_t ctrla, muxpos;


  ctrla = ADC0.CTRLA;
  muxpos = ADC0.MUXPOS;
 
  sprintf(uart_str,"ctrla %x muxpos %x \r\n",ctrla,muxpos);
  Serial3.print(uart_str);

}

void scan_adc_channels(void)
{

  int16_t adc_sample;

  sprintf(uart_str,"============== adc scan %lu  =================\r\n",scan_count);
  Serial3.print(uart_str);

  adc_sample = analogRead(PIN_PD6);
  sprintf(uart_str,"analog PD6 : %d\r\n",adc_sample);
  Serial3.print(uart_str);
  // scan_adc_config();

  adc_sample = analogRead(PIN_PD3);
  sprintf(uart_str,"analog PD3 : %d\r\n",adc_sample);
  Serial3.print(uart_str);
  // scan_adc_config();
  
}









