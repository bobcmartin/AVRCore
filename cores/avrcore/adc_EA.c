/*

    ADC driver for AVR EA C-Nano

    bob martin / microchip dec 2024






*/

#include "Arduino.h"
#include "wiring_private.h"
#include "util/delay.h"
// #include "adc_Dx.h"         // config inits for ADC0
#include <avr/pgmspace.h>



#ifndef F_CPU
  #error "F_CPU not defined. F_CPU must always be defined as the clock frequency in Hz"
#endif
#ifndef CLOCK_SOURCE
  #error "CLOCK_SOURCE not defined. Must be 0 for internal, 1 for crystal, or 2 for external clock"
#endif
 

 #if defined(__AVR_EA__)
  #pragma message "using AVR EA adc"

 
void init_ADC0(void) 
  {
    ADC_t* pADC;
   
   
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


 #endif 

