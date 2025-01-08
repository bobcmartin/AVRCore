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
#include "adc_Dx.h"         // config inits for ADC0
#include <avr/pgmspace.h>



ADC0_config_t adc_config;


// this is not working correctly
#if defined(__AVR_DX__)

#pragma message "Using DA, DB or DD ADC block"

void analogReference(uint8_t mode) 
{
  check_valid_analog_ref(mode);
  if (mode < 7 && mode != 4) {
    VREF.ADC0REF = (VREF.ADC0REF & ~(VREF_REFSEL_gm))|(mode);
  }
}


void init_ADC0(void) 
  {
    ADC_t* pADC;
   
   
    pADC = &ADC0;
    
    adc_config.init_delay = 2;
    adc_config.left_adjust = false;
    adc_config.mode = false;          // single ended mode

    analogReference(VDD);       
   
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
     
      /* Enable ADC */
      pADC->CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
      
      #if (defined(__AVR_DA__) && (!defined(NO_ADC_WORKAROUND)))
        // That may become defined when DA-series silicon is available with the fix
        pADC->MUXPOS = 0x40;
        pADC->COMMAND = 0x01;
        pADC->COMMAND = 0x02;
      #endif
  } // end init_ADC0


int16_t analogRead(uint8_t pin)
 {
  check_valid_analog_pin(pin);
  if (pin < 0x80) {
    pin = digitalPinToAnalogInput(pin);
    if (pin == NOT_A_PIN) {
      return ADC_ERROR_BAD_PIN_OR_CHANNEL;
    }
  }

  /* Select channel */
  ADC0.MUXPOS = ((pin & 0x7F) << ADC_MUXPOS_gp);

  /* Start conversion */
  ADC0.COMMAND = ADC_STCONV_bm;

  /* Wait for result ready */
  while(!(ADC0.INTFLAGS & ADC_RESRDY_bm));

  #if (defined(__AVR_DA__) && (!defined(NO_ADC_WORKAROUND)))
    // That may become defined when DA-series silicon is available with the fix
    ADC0.MUXPOS = 0x40;
  #endif
  return ADC0.RES;
}


inline __attribute__((always_inline)) void check_valid_negative_pin(uint8_t pin) {
  if(__builtin_constant_p(pin)) {
    if (pin < 0x80) {
      // If high bit set, it's a channel, otherwise it's a digital pin so we look it up..
      pin = digitalPinToAnalogInput(pin);
    }
    pin &= 0x3F;
    if (pin != 0x40 && pin != 0x48 && pin > 0x0F) { /* Not many options other than pins are valid */
      badArg("Invalid negative pin - valid options are ADC_GROUND, ADC_DAC0, or any pin on PORTD or PORTE.");
    }
  }
}

bool analogSampleDuration(uint8_t dur) {
    ADC0.SAMPCTRL = dur;
    return true;
}

bool analogReadResolution(uint8_t res)
 {
 
  if (res == 12) 
     ADC0.CTRLA = (ADC0.CTRLA & (~ADC_RESSEL_gm)) | ADC_RESSEL_12BIT_gc;
  else ADC0.CTRLA = (ADC0.CTRLA & (~ADC_RESSEL_gm)) | ADC_RESSEL_10BIT_gc;
  return true;
}

int8_t getAnalogReadResolution(void)
 {
  return ((ADC0.CTRLA & (ADC_RESSEL_gm)) == ADC_RESSEL_12BIT_gc) ? 12 : 10;
}


inline uint8_t getAnalogSampleDuration(void)
{
  return ADC0.SAMPCTRL;
}


#endif




