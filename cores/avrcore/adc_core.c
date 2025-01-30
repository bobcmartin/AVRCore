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

// highligh current ADC TYPE
// #define ADC_TYPE1
// #define ADC_TYPE2

/*

  ADC_TYPE1
    AVR DA, AVR DB, AVR DD 

  ADC_TYPE2
    AVR DU  

  ADC_TYPE3
    AVR EA, EB probasbly EC as well
*/



// fuuntion pointers for correct analogRead functions
int16_t (*adc_read)(uint8_t pin);
void (*adc_resolution)(uint8_t pin);
void (*adc_sample_count)(uint8_t sample_count);


void init_ADC0_type1(void);
void init_ADC0_type2(void);

int16_t analogRead_type1(uint8_t pin);
int16_t analogRead_type2(uint8_t pin);
int16_t analogRead_dif(uint8_t pin_plus,uint8_t pin_minus);
void analogSampleDuration(uint8_t dur);
int8_t getAnalogReadResolution(void);
inline uint8_t getAnalogSampleDuration(void);
ADC0_status_t* read_adc_status(void);

ADC0_config_t adc_config;
ADC0_status_t adc_status;

int8_t samplenum_index[2][8] = 
  {
      {0,1,2,3,4,5,6,7},
      {1,2,4,8,16,32,64,128}
  };




#ifdef ADC_TYPE1
void analogReference(uint8_t mode) 
  {
      if (mode < 7 && mode != 4)
        VREF.ADC0REF = (VREF.ADC0REF & ~(VREF_REFSEL_gm))|(mode);
 }
#endif 


#ifdef ADC_TYPE2
void analogReference(uint8_t mode) 
  {
    
    if (mode < 7 && mode != 0x01 && mode != 0x03)
        ADC0.CTRLC = mode;
    else ADC0.CTRLC = 0x00;      // default to VDD if invalid mode
 }
#endif 

#ifdef ADC_TYPE3
void analogReference(uint8_t mode) 
  {
    
    if (mode < 7 && mode != 0x01 && mode != 0x03)
        ADC0.CTRLC = mode;
    else ADC0.CTRLC = 0x00;      // default to VDD if invalid mode
 }
#endif 



// Initialize the ADC0 peripheral according to the chip type
// ADC_TYPE1 - AVR DA, DB and DD
// ADC_TYPE2 - AVR DU
// ADC_TYPE3 - AVR EA, EB

void init_ADC0(void) 
  {
    
  
    #ifdef ADC_TYPE1
      init_ADC0_type1();
    #endif

    #ifdef ADC_TYPE2
      init_ADC0_type2();
    #endif

    #ifdef ADC_TYPE3  
      init_ADC_type3();
    #endif

  }


#ifdef ADC_TYPE1
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
   // adc_resolution = analogReadResolution1;

  } // end init_ADC0_type1
#endif

int16_t analogRead(uint8_t pin)
{

  uint16_t adc_val;

  adc_val = adc_read(pin);
  if(adc_config.sample_number > 1)
    adc_val /= (uint16_t)adc_config.sample_number;

  return(adc_val);

}

#ifdef ADC_TYPE1
int16_t analogRead_type1(uint8_t pin)
 {
  
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
#endif


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
   
  if(adc_config.sample_number != 1)
    adc_val = adc_val / (int16_t)adc_config.sample_number;

  return(adc_val);     
}

#endif




// set sample number for accumulation mode in ADC TYPE1

#ifdef ADC_TYPE1
void analogRead_setsample(uint8_t sample_count)
{

  uint8_t scan_index,ctrlb_val = 0xFF;

  for(scan_index = 0;scan_index < 8;scan_index++)
  {

    if(samplenum_index[1][scan_index] == sample_count)
      ctrlb_val = samplenum_index[0][scan_index];    // extract correct value CTRLB reg
  }

  if(ctrlb_val == 0xFF)
    {
      ctrlb_val = 0x00;      // set to single sample if not valid
      adc_config.sample_number = 1;  
    }
  else adc_config.sample_number = sample_count;

  ADC0.CTRLB = ctrlb_val;

}
#endif 



/*

      ADC Type 2
      Substantial register map difference and ioavr define
      differences in this ADC definition    

*/

#ifdef ADC_TYPE2
void init_ADC0_type2(void)
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
   // adc_config.init_delay = ADC_INITDLY_DLY64_gc;               // arbitrary init delay of 64 clocks
   adc_config.reference = VDD;                  
   adc_config.sample_number = 1;                               // single sample mode 

  // scale the ADC speed to be around 1 MHz regardless of CPU frequency
   #if F_CPU >= 24000000
       pADC->CTRLB = ADC_PRESC_DIV20_gc; // 1.2 @ 24, 1.25 @ 25, 1.4 @ 28  MHz
   #elif F_CPU >= 20000000
        pADC->CTRLB = ADC_PRESC_DIV16_gc; // 1.25 @ 20 MHz
   #elif F_CPU >  12000000
        pADC->CTRLB = ADC_PRESC_DIV12_gc; // 1 @ 12, 1.333 @ 16 MHz
   #elif F_CPU >= 8000000
        pADC->CTRLB = ADC_PRESC_DIV8_gc;  // 1-1.499 between 8 and 11.99 MHz
   #elif F_CPU >= 4000000
        pADC->CTRLB = ADC_PRESC_DIV4_gc;  // 1 MHz
   #else                              
      pADC->CTRLB = ADC_PRESC_DIV2_gc;   // 1 MHz / 2 = 500 kHz
   #endif

  pADC->CTRLC = ADC_REFSEL_VDD_gc;       // set VDD as reference

  // disable window mode
  pADC->CTRLD = 0x00;

   // VREF init delay
  pADC->CTRLE = 64;                     // arbitrary for now

  // set 10 bit mode, non burst
  pADC->COMMAND = ADC_MODE_SINGLE_10BIT_gc;

   /* Enable ADC */ 
  pADC->CTRLA = ADC_ENABLE_bm;

   // set correct function pointers
  adc_read = analogRead_type2;
  adc_resolution = NULL;

  } // end init_ADC0_type1
#endif



   
#ifdef ADC_TYPE2
void analogRead_setsample(uint8_t sample_count)
{

  uint8_t scan_index,ctrlf_val = 0xFF;

  if(sample_count < 65)   // 64 is the cut off for AVR DU
  {
    for(scan_index = 0;scan_index < 8;scan_index++)
    {

      if(samplenum_index[1][scan_index] == sample_count)
        ctrlf_val = samplenum_index[0][scan_index];    // extract correct value CTRLF value
    }
  }
  
  // catch fall through
  if(ctrlf_val == 0xFF)
    {
      ctrlf_val = 0x00;      // set to single sample if not valid
      adc_config.sample_number = 1;  
    }
  else adc_config.sample_number = sample_count;

  ADC0.CTRLF |= ctrlf_val;      // preserve upper nibble of CTRLF

}
#endif 







#ifdef ADC_TYPE2
uint8_t pin_to_muxpos[25] = {255,255,22,23,24,25,26,27,31,0,1,2,3,4,5,6,7,16,17,18,19,20,21};
#endif

#ifdef ADC_TYPE2
int16_t analogRead_type2(uint8_t pin)
 {
  
   uint8_t mux_pos;
   uint16_t spinlock = 0;   
   mux_pos = pin_to_muxpos[pin];
  /* Select channel */
   ADC0.MUXPOS = mux_pos;

  /* Start conversion */
   ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;

  /* Wait for result ready */
  while(ADC0.STATUS & ADC_ADCBUSY_bm)
    spinlock++;

  return ADC0.RESULT;
}
#endif

// AVR EA/EB
#ifdef ADC_TYPE3  
void init_ADC0_type3(void)
{
    
    ADC_t* pADC;

    pADC = &ADC0;
    // load up defualts into config structure
    
    
    adc_config.init_delay = 2;
    adc_config.left_adjust = false;
    adc_config.dif_mode = false;                                // single ended mode
    adc_config.bit_depth = 12;
    adc_config.mux_pos = 0x40;                                    // init MUXPOS to GND
    adc_config.mux_neg = 0x40;
    adc_config.samp_delay = 0x02;                               // arbitrary samp delay of 2 clock cycles
    // adc_config.init_delay = ADC_INITDLY_DLY64_gc;               // arbitrary init delay of 64 clocks
    adc_config.reference = VDD;                  
    adc_config.sample_number = 1;                 

    // scale the ADC speed to be around 1 MHz regardless of CPU frequency
   #if F_CPU >= 24000000
       pADC->CTRLB = ADC_PRESC_DIV20_gc; // 1.2 @ 24, 1.25 @ 25, 1.4 @ 28  MHz
   #elif F_CPU >= 20000000
        pADC->CTRLB = ADC_PRESC_DIV16_gc; // 1.25 @ 20 MHz
   #elif F_CPU >  12000000
        pADC->CTRLB = ADC_PRESC_DIV12_gc; // 1 @ 12, 1.333 @ 16 MHz
   #elif F_CPU >= 8000000
        pADC->CTRLB = ADC_PRESC_DIV8_gc;  // 1-1.499 between 8 and 11.99 MHz
   #elif F_CPU >= 4000000
        pADC->CTRLB = ADC_PRESC_DIV4_gc;  // 1 MHz
   #else                              
      pADC->CTRLB = ADC_PRESC_DIV2_gc;   // 1 MHz / 2 = 500 kHz
   #endif

  pADC->CTRLC = ADC_REFSEL_VDD_gc;       // set VDD as reference

 // disable window mode
  pADC->CTRLD = 0x00;
 
 // safe number of samples
  pADC->CTRLE = 0x80;

 // no chopping, left adjust, not free running, one sample
 pADC->CTRLF = 0x00;

// PGA off
 pADC->PGACTRL = 0x00;

// Positive ADC input: direct , MUXPOS = AIN0
pADC->MUXPOS = 0x00;

// negative ADC input: direct, MUX_POS = AIN0
pADC-> MUXNEG = 0x00;

// set the function pointer
adc_read = analogRead_type3;

}

#endif

#ifdef ADC_TYPE3
int16_t analogRead_type3(uint8_t pin)
 {
  
  check_valid_analog_pin(pin);

  if (pin < 0x80)
    pin = digitalPinToAnalogInput(pin);
  
    // fill entries in adc_status
  adc_status.mux_plus = pin;
  

  /* Select channel */
  ADC0.MUXPOS = (pin & 0x7F);

  /* Start conversion */
  // ADC0.COMMAND = ADC_STCONV_bm;

  /* Wait for result ready */
  while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  return ADC0.RES;

}
#endif
