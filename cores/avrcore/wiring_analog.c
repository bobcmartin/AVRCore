/*  OBLIGATORY LEGAL BOILERPLATE
 This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation;
 either version 2.1 of the License, or (at your option) any later version. This library is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details. You should have received a copy of the GNU Lesser General Public License along with this library;
 if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*//*
  wiring_analog.c - analog input and output
  Servo.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
  Copyright (c) 2005-2006 David A. Mellis, modified 2010 by David Sproul,
  and at least one other individual, since *someone* ported it to "megaavr"
  (aka, modern AVR), and it wasn't me. Finally, Since megaTinyCore was released
  in 2018 this has been extensively modified first for mTC, and then later for DxC
  by Spence Konde (2018-2023).

  This file is included with megaTinyCore and DxCore; note that unlike some files,
  but like most of the "core" files, while some pieces of this code have been shared
  between the two, and DxC was forked from mTC, the differing demands of the two platforms
  are such that an omnibus file is not tenable.
*/


#include "wiring_private.h"
#include "pins_arduino.h"
#include "Arduino.h"
#include <avr/pgmspace.h>


// inline __attribute__((always_inline)) void check_valid_analog_pin(pin_size_t pin) {

#ifdef ADC_TYPE1
void check_valid_analog_pin(pin_size_t pin)
 {

  if (__builtin_constant_p(pin)) {
    if (
    #if defined(ADC_TEMPERATURE)
        pin != ADC_TEMPERATURE &&
    #endif
    #if defined(ADC_GROUND)
        pin != ADC_GROUND &&
    #endif
    #if defined(ADC_DACREF0)
        pin != ADC_DACREF0 &&
    #endif
    #if defined(ADC_DACREF1)
        pin != ADC_DACREF1 &&
    #endif
    #if defined(ADC_DACREF2)
        pin != ADC_DACREF2 &&
    #endif
    #if defined(ADC_VDDDIV10)
        pin != ADC_VDDDIV10 &&
    #endif
    #if defined(ADC_VDDDIV10)
        pin != ADC_VDDIO2DIV10 &&
    #endif
    #if defined(ADC_DAC0) // 1-series
        pin != ADC_DAC0 &&
    #endif
        true)
    { // if it is one of those constants, we know it is valid. Otherwise, make sure it's a valid channel.
      if (pin > 0x80) { // given as a channel, not a pin, but not one of the special inputs???
        pin &= 0x7f;
        if (pin > ADC_MAXIMUM_PIN_CHANNEL)
          badArg("analogRead called with constant channel number which is neither a valid internal source nor an analog pin");
      }
      pin = digitalPinToAnalogInput(pin);
      if (pin == NOT_A_PIN) {
        badArg("analogRead called with constant pin that is not a valid analog pin");
      }
    }
  }
}
#endif



// inline __attribute__((always_inline)) void check_valid_negative_pin(uint8_t pin) {
#ifdef ADC_TYPE1
void check_valid_negative_pin(uint8_t pin) 
{

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
#endif


void check_valid_analog_ref(uint8_t mode) 
{
  if (__builtin_constant_p(mode)) {
    if (!(mode == EXTERNAL || mode == VDD || mode == INTERNAL1V024 || mode == INTERNAL2V048 || mode == INTERNAL4V1 || mode == INTERNAL2V5))
    badArg("analogReference called with argument that is not a valid analog reference");
  }
}



inline __attribute__((always_inline)) void check_valid_duty_cycle(int16_t val) {
  if (__builtin_constant_p(val)) {
    if (val < 0 || val >255)
      badArg("analogWrite cannot produice duty cycle higher 100% or below 0%");
  }
}



#ifdef DAC0
  void DACReference(uint8_t mode) {
    check_valid_analog_ref(mode);
    VREF.DAC0REF = mode | (VREF.DAC0REF & (~VREF_REFSEL_gm));
  }
#else
  void DACReference(__attribute__ ((unused))uint8_t mode) {
    badCall("DACreference is not available - this part does not have a DAC");
  }
#endif



#ifdef DAC0
uint8_t getDACReference() {
  return VREF.DAC0REF & VREF_REFSEL_gm;
}

#endif
 




// PWM output only works on the pins with hardware support.
// These are defined in the appropriate pins_arduino.h
// for the variant in use. On all other pins, the best we
// can do is output a HIGH or LOW except on PIN_PD6, which
// is the DAC output pin.
#if defined(TCA1)
  const PROGMEM uint8_t _tcaonemux[8] = {0x20, 0x00, 0x08, 0x28, 0x10, 0xFF, 0x18}; // just enough order that no efficient operation works! Really?
  // mux(port) = 4, 0, 1, 5, 2, -, 3, - ?
  // port(mux) = 1, 2, 4, 6, 0, 3, -, - ?
#endif

#if defined(TCD0)
  const PROGMEM uint8_t _tcdmux[8]={0, 1, 5, 6, 4, -1, -1, -1};
#endif

void analogWrite(uint8_t pin, int val) {
  check_valid_digital_pin(pin);   // Compile error if pin is constant and isn't a pin.
  check_valid_duty_cycle(val);    // Compile error if constant duty cycle isn't between 0 and 255, inclusive. If those are generated at runtime, it is truncated to that range.
  uint8_t bit_mask = digitalPinToBitMask(pin);
  if (bit_mask == NOT_A_PIN) return; //this catches any run-time determined pin that isn't a pin.
  uint8_t offset = 0;
  uint8_t portnum  = digitalPinToPort(pin);
  uint8_t portmux_tca = PORTMUX.TCAROUTEA;
  TCA_t* timer_A = NULL;
  #if defined(TCA1)
  uint8_t threepin = 0;
  #endif
  // It could be a TCA0 or 1 or TCA1 mux 0 or 3;
  #if (defined(TCA1))
    if  (bit_mask < 0x40 && ((portmux_tca & 0x07) == portnum) && (__PeripheralControl & TIMERA0)) {
      timer_A = &TCA0;
    } else if ((__PeripheralControl & TIMERA1) && ((portmux_tca & 0x38) == pgm_read_byte_near(&_tcaonemux[portnum]))) {
      if (portnum == 1 || portnum == 6) {
        if (bit_mask < 0x40) {
          timer_A = &TCA1;
        }
      } else if ((bit_mask & 0x70) && portnum != 5) {
        timer_A = &TCA1;
        threepin = 1;
      }
    }

  #else
    if  ((bit_mask < 0x40) && ((portmux_tca == portnum) && (__PeripheralControl & TIMERA0))) {
      timer_A = &TCA0;
    }
  #endif
  if (timer_A != NULL) {
    offset = bit_mask;
    #if defined(TCA1)  //
      if (threepin) {  // and it's a 3-pin map then offset = 0b0xxx0000
        _SWAP(offset); // So swapo to get 0b00000xxx
      }
    #endif

    uint8_t ctrlb = offset;
    if (offset > 0x04) { // if 0b00xx x000
      ctrlb <<= 1;       // we leftshift what we're going to write to CTRLB one bit to get it into high nybble
    }
    // Now have ctrlb sorted. But we also need to do work on the offset
    if (offset & 0x07){ // 0b0000 0xxx
      offset &= 0xFE;   // WO0-2 ok - 0x0000 0xx0 0x01 -> 0x00, 0x02 -> 0x02, 0x04 -> 0x04
    } else {
      offset <<= 1;     //0b0xxx 0000
      _SWAP(offset);    //0b0000 0xxx
      offset |= 0x01;   //0b0000 0xx1 OK! 0x08 -> 0x01, 0x10 -> 0x03 0x20 -> 0x05
    }
    // THERE!! FINALLY!
    // Write the value to the register.
    *(((volatile uint8_t*) &(timer_A->SPLIT.LCMP0)) + offset) = val;
    // and ctrlb to ctrlb
    GPIOR2 = ctrlb;
    uint8_t t = timer_A->SPLIT.CTRLB;
    uint8_t oldsreg = SREG;
    cli();
    t |= ctrlb;
    GPIOR3 = t;
    timer_A->SPLIT.CTRLB = ctrlb;
    GPIOR1 = timer_A->SPLIT.CTRLB;
    SREG = oldsreg;
    /* Okay, this layout tends towards maximum pervosity. You basically have to treat them as entirely separate timers at this point!
     * PORT | DA | DB | DD | EA | portnum
     *    A | XX | XX | XX | 20 | 0; portmux == 0x20 EA only                    portnux >> 2 == 4
     *    B | 00 | 00 | XX | 00 | 1; portmux == 0    DA, DB, EA 48 pin - 6 pins portmux >> 2 == 0
     *    C | 08 | 08 | XX | 08 | 2 == portmux >> 2; DA, DB, EA 48 pin - 3 pins portmux >> 2 == 2
     *    D | XX | XX | XX | 28 | 3; portmux == 0x28 EA only                    portmux >> 2 == 5
     *    E | 10 | 10 | XX | XX | 4 == portmux >> 2  DB, 48 pin (and DA, maybe) - 3 pins
     *    F | XX | XX | XX | XX | -
     *    G | 18 | 18 | XX | XX | 6 == portmux >> 2; DB, 48 pin (and DA, maybe) - 6 pins
     *
     * PORTG and PORTE do not function on currently available DA hardware.
     *
     * PORTC, PORTA, PORTD, and PORTE are pins 4-6 only. No PORTE except on 64-pin parts.
     *
     * No TCA1 on 32-pin or less DB.
     * No PORTA or PORTD except on EA (it was a newly added mux option) there.
     * No TCA1 on DD at all.
     */
    //#if !defined(__AVR_DD__)
      _setOutput(portnum, bit_mask);
    //#endif
  } else {
    TCB_t *timer_B;
    // TCA_t *timer_A;
    uint8_t digital_pin_timer = digitalPinToTimer(pin);
    switch (digital_pin_timer) {
      case NOT_ON_TIMER:{
        if (val < 128) {
          _setValueLow(portnum, bit_mask);
        } else {
          _setValueHigh(portnum, bit_mask);
        }
        break;
      }
      case TIMERB0:
      case TIMERB1:
      case TIMERB2:
      case TIMERB3:
      case TIMERB4:
        /* Get pointer to timer, TIMERB0 order definition in Arduino.h*/
        // assert (((TIMERB0 - TIMERB3) == 2));
        timer_B = ((TCB_t *)&TCB0 + (digital_pin_timer - TIMERB0));
        // make sure the timer is in PWM mode
        if (((timer_B->CTRLB) & TCB_CNTMODE_gm) == TCB_CNTMODE_PWM8_gc ) {
          /* set duty cycle */
          #if defined(ERRATA_TCB_CCMP) && ERRATA_TCB_CCMP == 0
            timer_B->CCMPH = val; /* does not yet exist */
          #else
            timer_B->CCMPL = timer_B->CCMPL;   // load temp register with the period, 254 have to first make sure temp register holds 254
            timer_B->CCMPH = val;              /* We can leave interrupts on - only a read of the count in the ISR would mess things up.
             * That is a wacky corner case. If they have timer in 8-bit PWM mode, and they write the value in with another call, yet ALSO
             * insist on reading the timer value from within an ISR, yes that's a race condition, and it will shit on the compare value */
          #endif
          /* Enable Timer Output */
          timer_B->CTRLB |= (TCB_CCMPEN_bm);
          return;
        } // if it's not, we don't have PWM on this pin!
        break;
      #if defined(DAC0)
        case DACOUT: {
          _setInput(portnum, bit_mask);
          uint8_t ctrla = DAC0.CTRLA;
          if (val == 0 || val == 255) {
            ctrla &= ~0x41; // clear we want to turn off the DAC in this case
          }
          volatile uint8_t* pinctrl_ptr = (volatile uint8_t*) 0x0476; // PD6 PINnCTRL;
          *pinctrl_ptr |= PORT_ISC_INPUT_DISABLE_gc;
          #if defined(DAC0_DATAH)
            DAC0.DATAH = val;
            DAC0.CTRLA |= 0x41; // OUTEN = 1, ENABLE = 1, but don't trash run stby
          #else
            DAC0.DATA = val;
            DAC0.CTRLA |= 0x41; // OUTEN = 1, ENABLE = 1, but don't trash run stby
          #endif
          return;
        }
      #endif
      #if (defined(TCD0) && defined(USE_TIMERD0_PWM))
      // Else, it's on TCD0
        default: {
          #if defined(NO_GLITCH_TIMERD0)
          // "No glitch timerd" mode means that if analogWrite(pin,val) is called for a pin on a type D timer
          // with a duty cycle of 0% or 100%, instead of digitalWrite()'ing the pin, we will leave the timer
          // connected, and instead set the duty cycle to 0%. If the requested duty cycle is 100%, the pin
          // will then be inverted.
          //
          // If this is not defined, then the 0% and 100% cases will instead have been caught by the conditional
          // at the start of analogWrite().

            uint8_t set_inven = 0; // this will be set to 1 if we're setting the pin to a duty cycle of 100%
            if(val <= 0){
              val = 0;
            } else if (val >= 255){
              val = 0;
              set_inven = 1;
            }
          #endif
          /**************************************
          Determine the bit within TCD0.FAULTCTRL
          On Dx-series, WOA is always on bit 0 or bit 4 and so on
          On tinyAVR 1-series, WOA/WOB is on PA4/PA5, and WOC, WOD is on PC0/PC.
          In the past the same copy of this function was used for both cores. That has become untenable
          ***************************************/
          // Dx-series
          uint8_t port = digitalPinToPort(pin);
          uint8_t tcdmux = pgm_read_byte_near(&_tcdmux[(digital_pin_timer & 0x07)]);
          // First, if TCD portmux busted, but it's not set to 0, we can't get PWM, don't try
          uint8_t fc_mask = bit_mask ;
          #if defined(ERRATA_TCD_PORTMUX) && ERRATA_TCD_PORTMUX == 0
            if ((tcdmux != PORTMUX.TCDROUTEA && ((digital_pin_timer & 0x44) != 0x44 ))) {
              break;
            }
            if (!(tcdmux & 0x04)) {
              if (bit_mask < 0x10) { //cpi
                fc_mask <<= 4;// swap andi, hopefully.
              }
            } else {
              if (port == 3) { //cpse rjmp .+4
                fc_mask <<= bit_mask << 2; // lsr lsr
              }
            }
          #else
            if (((tcdmux & 0x07) != 0)) {
              /* On these parts, there is no available silicon with a working TCD portmux! */
              if (val < 128) {
                _setValueLow(portnum, bit_mask);
              } else {
                _setValueHigh(portnum, bit_mask);
              }
              _setOutput(portnum, bit_mask); // remove this or replace with errata test if it turns out that the direction override is errata and will befixed.
              break;
            }
          #endif

          // 128 input  with the max set to 1019 should get us 509. This was WRONG.
          // We need to subtract from 256 now, leaving us with a number from 1 to 256
          uint8_t temp = TCD0.CMPBCLRL;
          temp = TCD0.CMPBCLRH;
          //
          // Read both, only retaining the high byte. Need to read both to see high byte because 16-bit register
          // Reading just high doesn't work. Checking for CMPBCLR = 509, 1019, or at 32 MHz+, 2039 or 4079 for which we need to shift
          // the duty cycle left to match
          if (temp) {   // TOP > 254
            val <<= 1;  // leftshift once is good for 509
            if (temp   >= 0x03) {
              val <<= 1;  // 1019, 2039 or 4079
              #if F_CPU >= 32000000
                if (temp >= 0x07) {
                  val <<= 1;  // 2039
                  if (temp == 0x0F) {
                    val <<= 1;  // 4079
                    val = 4080 - val;
                  } else {
                    val = 2040 - val;
                  }
                } else {
                  val = 1020 - val;
                }
              #else
                val = 1020 - val;
                } else {
              #endif
              val = 510 - val;
            }
          } else {
          val = 255 - val;
        }

        #if defined(NO_GLITCH_TIMERD0)
          volatile uint8_t *pin_ctrl_reg = getPINnCTRLregister(portToPortStruct(port), digitalPinToBitPosition(pin));
          // if things aren't compile-time known, this is not a lightning fast operation.
          // We had been doing it closer to where we needed it, but there's no need to wait
          // until we have interrupts off to figure this out (though we do need them off when)
          // access it!)
        #endif
        // interrupts off while this runs - we really don't want this interrupted!
        uint8_t oldSREG = SREG;
        cli();
        /*-----------------------------------------------------------------------------------------
         * bit_mask & 0xAA? 0xAA = 0b10101010
         * This is true if the bitmask corresponds to an odd bit in the port, meaning it's
         * going to be driven by CMPB, otherwise by CMPA. On all existing parts, WOA is on
         * bit 0 or 4, WOB on 1 or 5, WOC on 0, 2, 4, or 6, and WOD on 1, 3, 5, or 7.
         * Pins WOA and WOB are bound to CMPA and CMPB, but WOC and WOD can each be put on
         * either WOA or WOB. So if WOC is assigned to follow WOA and WOD to follow WOB, this
         * test gives the answer. This means, in theory, flexible PWM on TCD0 could be improved
         * by detecting the case where WOA or WOB is outputting PWM already, and the user then
         * calls analogWrite() on the other pin assigned to that channel, and we could swap that
         * pin to the other channel. But the code would be ugly (read: slow) and I don't think
         * the added capability would even be an improvement overall, because the cost in
         * terms of less consistent behavior is significant: the results become path-dependant,
         * since writing WOA, WOC, WOB in that order would result in WOC getting swapped to
         * WOB (now WOB and WOC would output same thing) while WOA, WOB, WOC would not, so
         * WOA and WOC would be the pair outputting the same thing). And then you'd need to
         * decide how to handle the above situation when the user then wrote to WOD.
         * Better to just declare that CMPA shall drive WOC, and CMPB shall drive WOD.
         *-----------------------------------------------------------------------------------------*/
        if (bit_mask & 0xAA) {
          TCD0.CMPBSET = val - 1;
        } else {
          TCD0.CMPASET = val - 1;
        }
        /* Check if channel active, if not, have to turn it on */
        if (!(TCD0.FAULTCTRL & fc_mask)) {
          /*-----------------------------------------------------------------------------------------
           * need to be careful here - analogWrite() can be called by a class constructor, for
           * example in which case the timer hasn't been started yet. We must not start it in this
           * case, as it would then fail to initialize and have the wrong clock prescaler and other
           * settings. Similarly, in any other situation where the timer isn't running when we started
           * the most likely result of being automatically started by an analogWrite() is naught but
           * woe and misery.
           * Instead, we should do everything else, and when the timer is next enabled, the PWM will
           * be configured and waiting. This is also probably what users would expect and hope to
           * happen if they are modifying TCD0 registers themselves. Though per core docs, we make
           * no promises in that case, the fact that the fix for a call to analogWrite() in a class
           * constructor (something that is not proscribed by docs, and hence is supposed to work)
           * makes that case less bad is an added bonus.
           *---------------------------------------------------------------------------------------*/
          uint8_t temp2 = TCD0.CTRLA;
          TCD0.CTRLA = temp2 & (~TCD_ENABLE_bm);
          while(!(TCD0.STATUS & 0x01));    // wait until it can be re-enabled
          _PROTECTED_WRITE(TCD0.FAULTCTRL, (fc_mask | TCD0.FAULTCTRL));
          // while(!(TCD0.STATUS & 0x01));    // wait until it can be re-enabled
          TCD0.CTRLA = temp2; // re-enable it if it was enabled
        } else {
          TCD0.CTRLE = TCD_SYNCEOC_bm; // it was already on - just set new value and set sync flag.
        }

        #if defined(NO_GLITCH_TIMERD0)
          // In this mode, we need to check set_inven, and set INVEN if it was called with 100% duty cycle
          // and unset that bit otherwise.
          if (set_inven == 0){
            // we are not setting invert to make the pin HIGH when not set; either was 0 (just set CMPxSET > CMPxCLR)
            // or somewhere in between.
            *pin_ctrl_reg &= ~PORT_INVEN_bm;
          } else {
            // we *are* turning off PWM while forcing pin high - analogwrite(pin,255) was called on TCD0 PWM pin...
            *pin_ctrl_reg |= PORT_INVEN_bm;
          }
        #endif
        SREG = oldSREG; // Turn interrupts back on, if they were off.
        }
      #endif
    }
  // now hastily set the pin output with this quickie macro since we know alll we need in order to do so now.
  }
  //_setOutput(portnum, bit_mask);
}
void takeOverTCA0() {
  #if defined(MILLIS_USE_TIMERA0)
    stop_millis();
  #endif
  TCA0.SPLIT.CTRLA = 0;          // Stop TCA0
  __PeripheralControl &= ~TIMERA0; // Mark timer as user controlled
  TCA0.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03; // Reset TCA0
  /* Okay, seriously? The datasheets and io headers disagree here for tinyAVR
     about whether the low bits even exist! Much less whether they need to be
     set - but if they are not set, it will not work */
}

void resumeTCA0() {
  TCA0.SPLIT.CTRLA = 0;         // Stop TCA0
  TCA0.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03; // Reset TCA0
  init_TCA0();                  // reinitialize TCA0
  __PeripheralControl |= TIMERA0; // Mark timer as core controlled
  #if defined(MILLIS_USE_TIMERA0)
    restart_millis();              // If we stopped millis for takeover, restart
  #endif
}

#if defined(TCA1)
void takeOverTCA1() {
  #if defined(MILLIS_USE_TIMERA1)
    stop_millis();
  #endif
  TCA1.SPLIT.CTRLA = 0;               // Stop TCA1
  __PeripheralControl &= ~TIMERA1;      // Mark timer as user controlled
  TCA1.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03; // Reset TCA1
}

void resumeTCA1() {
  TCA1.SPLIT.CTRLA = 0;         // Stop TCA1
  TCA1.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03; // Reset TCA1
  init_TCA1();                  // reinitialize TCA1
  __PeripheralControl |= TIMERA1; // Mark timer as core controlled
  #if defined(MILLIS_USE_TIMERA1)
    restart_millis();              // If we stopped millis for takeover, restart
  #endif
}
#endif
#if defined(TCD0)
void takeOverTCD0() {
#if !defined(MILLIS_USE_TIMERD0)
  TCD0.CTRLA = 0;                     // Stop TCD0
  _PROTECTED_WRITE(TCD0.FAULTCTRL,0); // Turn off all outputs
  __PeripheralControl &= ~TIMERD0;      // Mark timer as user controlled
#else
  badCall("TCD0 takeover not permitted when TCD0 is millis source");
  /* Note that it's just TCD0 we protect like this... With TCA's, the user has
     a much better chance of being able to put it back together with the
     millis() control functions. */
#endif
}

void resumeTCD0() {
  badCall("Resuming core control of type D timer not supported.");
}
#endif

uint8_t digitalPinToTimerNow(uint8_t p) {
  uint8_t bit_pos = digitalPinToBitPosition(p);
  if (bit_pos == NOT_A_PIN) return NOT_ON_TIMER;     /* Use bit position to check for invalid pins */
  uint8_t port = digitalPinToPort(p);                /* If bit_pos is valid, port will be too      */
  if ( bit_pos < 6) {                                /* SPLIT MODE TCA output is on pins 0-5       */
  #if defined(TCA1)
    uint8_t tcamux = PORTMUX.TCAROUTEA;
    if ( __PeripheralControl & TIMERA0) {              /* make sure user hasn't taken over TCA0      */
      if (((tcamux & PORTMUX_TCA0_gm) == port)) {    /* TCA0 mux is EASY - same as the port number */
        return TIMERA0;
      }
    }
    tcamux &= 0x18;
    if (__PeripheralControl & TIMERA1) {               /* make sure user hasn't taken over TCA0      */
      if ((tcamux == 0 && port == PB ) || (tcamux == 0x18 && port == PG)) { /* supports only 6-ch  */
        return TIMERA1;                              /* mux options, not 3-channel ones on bit 4:6 */
      }
    }
  #elif defined(TCA0)
    if (__PeripheralControl & TIMERA0) {               /* here we don't need to store tcamux */
      if ((PORTMUX.TCAROUTEA & PORTMUX_TCA0_gm) == port) { /* because it is only used once */
        return TIMERA0;
      }
    }
  #else
    /* EB-series doesn't have any TCA's!
     * Instead, they've got WEX and his crony TCE
     * we'll track their position from this lookout
     * Unless my crystal ball has given out again,
     * TCE and WEX will move much like a TCA, as a whole
     * Not like TCA channels on tiny, or TCB channels.
     */
  #endif
  }
  uint8_t timer = digitalPinToTimer(p);
  #if defined(TCD0)
    if ( __PeripheralControl & TIMERD0) {
      if (timer & TIMERD0) {
        byte tcdmux = (PORTMUX.TCDROUTEA & PORTMUX_TCD0_gm);
        if (tcdmux == (timer & ~TIMERD0)) {
          return TIMERD0;
        }
      }
    }
  #endif
  #if defined(TCF0)
    /* The EB also has something called a TCF that might generate waveform output.
     * Nothing is known about it other than it's home pins and that it's letter of
     * the alphabet. Time will tell if that F'ing timer is any good for PWM, or if
     * it's an F'ing utility timer like TCB - lord knows with how stingy they were
     * with peripherals on the EB, it needs it.
     */
  #endif
  if (timer & TIMERB0) { /* Finally check TCBn, if we made it here w/out returning */
    TCB_t* timer_B;
    timer_B = ((TCB_t *)&TCB0 + (timer - TIMERB0)); /* get timer struct */
    if (((timer_B->CTRLB) &  TCB_CNTMODE_gm) != TCB_CNTMODE_PWM8_gc )
      return NOT_ON_TIMER; /* If the timer isn't in PWM mode, user has reconfigured
                              it, and nothing good can come of trying to use it. */
  }
  return timer;
}
