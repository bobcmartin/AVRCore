/*

    analogRead functions for 
    AVR DA, AVR DB and AVR DD devices.
    All ADC blocks are the same

    Sept 2024 // bob martin


*/ 





#include "wiring_private.h"
#include "util/delay.h"
#include "adc_Dx.h"         // config inits for ADC0


ADC0_config_t adc_config;


#ifndef F_CPU
  #error "F_CPU not defined. F_CPU must always be defined as the clock frequency in Hz"
#endif
#ifndef CLOCK_SOURCE
  #error "CLOCK_SOURCE not defined. Must be 0 for internal, 1 for crystal, or 2 for external clock"
#endif
 
#define __AVR_DA__ 1


// #if defined ((__AVR_DA__) || (__AVR_DB__) || (__AVR_DD__))
#pragma message "Using DA, DB or DD ADC block"
 
 
 void init_ADC0(void) 
  {
    ADC_t* pADC;
    // _fastPtr_d(pADC, &ADC0);
   
    adc_config.init_delay = 2;
    adc_config.left_adjust = false;
    adc_config.mode = false;          // single ended mode

    





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
      #else  // 1 MHz / 2 = 500 kHz - the lowest setting
        pADC->CTRLC = ADC_PRESC_DIV2_gc;
      #endif
      pADC->SAMPCTRL = 14; // 16 ADC clock sampling time - should be about the same amount of *time* as originally?
      
      pADC->CTRLD = ADC_INITDLY_DLY64_gc; // VREF can take 50uS to become ready, and we're running the ADC clock
      // at around 1 MHz, so we want 64 ADC clocks when we start up a new reference so we don't get bad readings at first
      /* Enable ADC */
      pADC->CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
      // start at 10 bit for compatibuility with existing code.

      #if (defined(__AVR_DA__) && (!defined(NO_ADC_WORKAROUND)))
        // That may become defined when DA-series silicon is available with the fix
        pADC->MUXPOS = 0x40;
        pADC->COMMAND = 0x01;
        pADC->COMMAND = 0x02;
      #endif


int16_t analogRead(uint8_t pin) 
{
    
    check_valid_analog_pin(pin);
    
    if (pin < 0x80)
     {
              pin = digitalPinToAnalogInput(pin);
    } else {
      pin &= 0x3F;
    }
    
      return ADC_ERROR_BAD_PIN_OR_CHANNEL;
    }
    
    
    ADC0.MUXPOS = pin;    
    uint8_t command = (_analog_options & 0x0F) > 8 ? 0x11 : 0x01;
    /* Start conversion */
    ADC0.COMMAND = command;

    /* Wait for result ready */
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    // if it's 10 bit compatibility mode, have to rightshift twice.
    if ((_analog_options & 0x0F) == 10) {
      int16_t temp = ADC0.RESULT;
      temp >>= 2;
      return temp;
    }
    return ADC0.RESULT;
  }


int analogReadDiff(uint8_t pin_pos,uint8_t pin_neg)
{
  
    check_valid_analog_pin(pin_pin);





}


/*

#else
#pragma message "not using AVR DA, DB or DD adc block"
  void __attribute__((weak)) init_ADC0() 
    {

      return();
    }
    
#endif

*/


