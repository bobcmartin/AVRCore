/*

    functions for 
    AVR DA, AVR DB and AVR DD devices.
    All ADC blocks are the same

    Sept 2024
    core code extracted from DxCore / Spence Konde
    
    avrcore v1.x
    bob martin

*/ 


#include "Arduino.h"
#include "wiring_private.h"
#include "util/delay.h"
#include "adc_core.h"         // config inits for ADC0
#include <avr/pgmspace.h>


/*

  ADC_CORE1
    AVR DA, AVR DB, AVR DD 


  ADC_CORE2
    AVR DU  


*/

void init_ADC0_type1(void);
void init_ADC0_type2(void);

int16_t analogRead_type1(uint8_t pin);
int16_t analogRead_dif(uint8_t pin_plus,uint8_t pin_minus);
void analogSampleDuration(uint8_t dur);
int8_t getAnalogReadResolution(void);
inline uint8_t getAnalogSampleDuration(void);
ADC0_status_t* read_adc_status(void);

ADC0_config_t adc_config;
ADC0_status_t adc_status;

u

  void analogReference(uint8_t mode) 
  {
    check_valid_analog_ref(mode);
      if (mode < 7 && mode != 4)
        VREF.ADC0REF = (VREF.ADC0REF & ~(VREF_REFSEL_gm))|(mode);
  }


// fuuntion pointers for correct analogRead functions
int16_t (*adc_read)(uint8_t pin);
void (*adc_resolution)(uint8_t pin);
void (^adc_sample_count)(uint8_t sample_count);



// Initialize the ADC0 peripheral according to the chip type

void init_ADC0(void) 
  {
    
  
    #if defined(ADC_TYPE1)
      init_ADC0_type1();
    #endif

    #if defined(ADC_TYPE2)
      init_ADC_type2();
    #endif

  }


/*
uint8_t prescaler;
uint8_t left_adjust;
uint8_t sample_number;
uint8_t dif_mode;
uint8_t mux_pos;
uint8_t mux_neg;
uint8_t init_delay;
uint8_t samp_delay;
uint8_t reference;
uint8_t bit_depth;
*/


void init_ADC0_type1(void)
{

   ADC_t* pADC;

   pADC = &ADC0;
    
   adc_config.init_delay = 2;
   adc_config.left_adjust = false;
   adc_config.dif_mode = false;                                // single ended mode
   adc_config.bit_depth = 10;
   adc_config.mux_pos = 0x40;                                    // init MUXPOS to GND
   adc_config.mux_neg = 0x40;
   adc_config.samp_delay = 0x02;                               // arbitrary samp delay of 2 clock cycles
   adc_config.init_delay = ADC_INITDLY_DLY64_gc;               // arbitrary init delay of 64 clocks
   adc_config.reference = VDD;                  
   adc_config.sample_number = 1;                               // single sample mode

   analogReference(VDD);                                       // analog ref set to VDD by default
   
  // scale the ADC speed to be around 1 MHz regardless of CPU frequency
  
   #if F_CPU >= 24000000
       pADC->CTRLC = ADC_PRESC_DIV20_gc; // 1.2 @ 24, 1.25 @ 25, 1.4 @ 28  MHz
   #elif F_CPU >= 20000000
        pADC->CTRLC = ADC_PRESC_DIV16_gc; // 1.25 @ 20 MHz
   #elif F_CPU >  12000000
        pADC->CTRLC = ADC_PRESC_DIV12_gc; // 1 @ 12, 1.333 @ 16 MHz
   #elif F_CPU >= 8000000
        pADC->CTRLC = ADC_PRESC_DIV8_gc;  // 1-1.499 between 8 and 11.99 MHz
   #elif F_CPU >= 4000000
        pADC->CTRLC = ADC_PRESC_DIV4_gc;  // 1 MHz
   #else                              
      pADC->CTRLC = ADC_PRESC_DIV2_gc;   // 1 MHz / 2 = 500 kHz
   #endif
       
   // 16 ADC clock sampling time 
   pADC->SAMPCTRL = 14; 
      
   // VREF init delay
   pADC->CTRLD = ADC_INITDLY_DLY64_gc; 
     
   /* Enable ADC in 10 bit mode */
   pADC->CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;

   // set correct function pointers
   adc_read = analogRead_type1;
   adc_resolution = analogReadResolution1;

  } // end init_ADC0_type1


int16_t analogRead(uint8_t pin)
{

  uint16_t adc_val;

  adc_val = adc_read(pin);
  if(adc_config.sample_number > 1)
    adc_val /= (uint16_t)adc_config.sample_number;

  return(adc_val);

}



int16_t analogRead_type1(uint8_t pin)
 {
  
  check_valid_analog_pin(pin);

  if (pin < 0x80)
    pin = digitalPinToAnalogInput(pin);
  
  ADC0.CTRLA &= ~(ADC_CONVMODE_DIFF_gc);  // set single ended mode

  // set negative input to GND
  ADC0.MUXNEG = 0x40;

  // fill entries in adc_status
  adc_status.mux_plus = pin;
  adc_status.mux_neg = 0x40;


  /* Select channel */
  ADC0.MUXPOS = ((pin & 0x7F) << ADC_MUXPOS_gp);

  /* Start conversion */
  ADC0.COMMAND = ADC_STCONV_bm;

  /* Wait for result ready */
  while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  return ADC0.RES;

}


#ifdef ADC_TYPE1  // AVR DA,DB,DD
int16_t analogRead_diff(uint8_t pin_plus,uint8_t pin_minus)
{

  int16_t adc_val = 0;

  check_valid_analog_pin(pin_plus);
  check_valid_negative_pin(pin_minus);
    
  if (pin_plus < 0x80)
     pin_plus = digitalPinToAnalogInput(pin_plus);
  if (pin_minus < 0x80)
     pin_minus = digitalPinToAnalogInput(pin_minus);
  
  // set ADC0 into diff mode
  ADC0.CTRLA |= ADC_CONVMODE_DIFF_gc; 
    
    /* Select positive channel */
  ADC0.MUXPOS = ((pin_plus & 0x7F) << ADC_MUXPOS_gp);

  // select negative channel
  ADC0.MUXNEG = ((pin_minus & 0x7F) << ADC_MUXNEG_gp);
 
  /* Start conversion */
  ADC0.COMMAND = ADC_STCONV_bm;

  /* Wait for result ready */
  while(!(ADC0.INTFLAGS & ADC_RESRDY_bm)); 
   adc_val = ADC0.RES;
   
  if(adc_config.sample_count != 1)
    adc_val = adc_val / adc_config.sample_count;

  return(adc_val);     
}

#endif

// AVR DU
#ifdef ADC_TYPE2
int16_t analogRead_diff(uint8_t pin_plus,uint8_t pin_minus)
{

  int16_t adc_val = 0;

  check_valid_analog_pin(pin_plus);
  check_valid_negative_pin(pin_minus);
    
  if (pin_plus < 0x80)
     pin_plus = digitalPinToAnalogInput(pin_plus);
  if (pin_minus < 0x80)
     pin_minus = digitalPinToAnalogInput(pin_minus);
  
  // set ADC0 into diff mode
  ADC0.CTRLA |= ADC_CONVMODE_DIFF_gc; 
    
    /* Select positive channel */
  ADC0.MUXPOS = ((pin_plus & 0x7F) << ADC_MUXPOS_gp);

  // select negative channel
  ADC0.MUXNEG = ((pin_minus & 0x7F) << ADC_MUXNEG_gp);
 
  /* Start conversion */
  ADC0.COMMAND = ADC_STCONV_bm;

  /* Wait for result ready */
  while(!(ADC0.INTFLAGS & ADC_RESRDY_bm)); 
   adc_val = ADC0.RES;
   
  return(adc_val);     
}

#endif


int8_t sample_index[2][8] =
  {
      {0,1,2,3,4,5,6,7},
      {1,2,4,8,16,32,64,128}
  }

// sample number

#ifdef ADC_TYPE1

void analogRead_setsample(uint8_t sample_count)
{

  uint8_t scan_index,ctrlb_val = 0xFF;

  for(scan_index = 0,scan_index < 8;scan_index++)
  {

    if(sample_index[1][scan_index] == sample_count)
      ctrlb_val = sample_index[0][scan_index];    // extract correct value CTRLB reg
  }

  if(ctrlb_val = 0xFF)
    {
      ctrlb_val = 0x00;      // set to single sample if not valid
      adc_config.sample_number = 1;  
    }
  else adc_config.sample_number = sample_count;

  ADC0.CTRLB = crtlb_val;

}
#endif 



void analogSampleDuration(uint8_t dur) 
{
    ADC0.SAMPCTRL = dur;
}  



void analogReadResolution(uint8_t res)
 {

    adc_resolution(res);
  
 }

void analogReadResolution1(uint8_t res)
{

  if (res == 12) 
     ADC0.CTRLA = (ADC0.CTRLA & (~ADC_RESSEL_gm)) | ADC_RESSEL_12BIT_gc;
  else ADC0.CTRLA = (ADC0.CTRLA & (~ADC_RESSEL_gm)) | ADC_RESSEL_10BIT_gc;

}

void analogReadResolution2(uint8_t res)
{

  if (res == 12) 
     ADC0.CTRLA = (ADC0.CTRLA & (~ADC_RESSEL_gm)) | ADC_RESSEL_12BIT_gc;
  else ADC0.CTRLA = (ADC0.CTRLA & (~ADC_RESSEL_gm)) | ADC_RESSEL_10BIT_gc;

}

int8_t getAnalogReadResolution(void)
 {
  return ((ADC0.CTRLA & (ADC_RESSEL_gm)) == ADC_RESSEL_12BIT_gc) ? 12 : 10;
}


inline uint8_t getAnalogSampleDuration(void)
{
  return ADC0.SAMPCTRL;
}


ADC0_status_t* read_adc_status(void)
{

    return(&adc_status);



}
