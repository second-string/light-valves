/**
* \file main.c
*
* \brief Main source file.
*
(c) 2021 Microchip Technology Inc. and its subsidiaries.
    Subject to your compliance with these terms, you may use this software and
    any derivatives exclusively with Microchip products. It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
*/

#include <avr/io.h>
#include "SoftwareSerial.h"

#define LIGHT_VALVE_PIN_1 PIN_PB0  // bottom pin
#define LIGHT_VALVE_PIN_2 PIN_PB1  // top pin

#define UART_TX LIGHT_VALVE_PIN_1
#define UART_RX LIGHT_VALVE_PIN_2
static SoftwareSerial debug_serial(UART_RX, UART_TX);

/*Using default clock 3.3MHz */

void TCD0_init(void);
void TCD0_enableOutputChannels(void);
void EVENT_SYSTEM_init(void);
void PORT_init(void);

void TCD0_init(void)
{
      // Enable-protected reg must have 0 written to enable bit before any other bits can be changed, and it defaults to
    // enabled on startup
    /* while (!(TCD0.STATUS & TCD_ENRDY_bm)) { */
    /*     ; */
    /* } */
    TCD0.CTRLA &= ~TCD_ENABLE_bm;

    // Don't need overlapping PWM signals so just do oneramp
//    TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;
//
//    // Disable all input control
//    TCD0.INPUTCTRLA = TCD_INPUTMODE_NONE_gc;
//    TCD0.INPUTCTRLB = TCD_INPUTMODE_NONE_gc;
//
//    // Enable WOA and WOB (don't need C and D). Since FAULTCTRL reg is write protected we need to make it editable in
//    // CCP reg
//    CPU_CCP        = CCP_IOREG_gc;
//    TCD0.FAULTCTRL = TCD_CMPAEN_bm | TCD_CMPBEN_bm;
//
//    // We will always start high with A output and end high with B output. The clear for A and
//    // set for B values are what are adjusted based on desired duty cycle
//    TCD0.CMPASET = 0x000;
//    TCD0.CMPBCLR = 0xFFF;
//
//    TCD0.CMPACLR = 0x7FF;
//    TCD0.CMPBSET = 0x801;
//
//    // System 20mhz clock w/ largest prescaler possible of 32 but no synchronization prescaler. Must be done as last
//    // operation before starting timer with ENABLE bit
//    TCD0.CTRLA = TCD_CLKSEL_20MHZ_gc | TCD_CNTPRES_DIV32_gc | TCD_SYNCPRES_DIV1_gc | TCD_ENABLE_bm;

 
    /* set the waveform mode */
    TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;
                  
    TCD0.CMPASET = 0x000;
    TCD0.CMPBCLR = 0xFFF;

    TCD0.CMPACLR = 0x7FF;
    TCD0.CMPBSET = 0x801;
    
    
    /* ensure ENRDY bit is set */
    while(!(TCD0.STATUS & TCD_ENRDY_bm))
    {
        ;
    }
    
    TCD0.CTRLA = TCD_CLKSEL_20MHZ_gc        /* choose the timer's clock */
               | TCD_CNTPRES_DIV4_gc        /* choose the prescaler */
               | TCD_ENABLE_bm;                /* enable the timer */
               
}

void TCD0_enableOutputChannels(void)
{
    /* enable write protected register */
    CPU_CCP = CCP_IOREG_gc;
    
    TCD0.FAULTCTRL = TCD_CMPAEN_bm            /* enable channel A */
                   | TCD_CMPBEN_bm;            /* enable channel B */
} 


void PORT_init(void)
{   
  uint8_t bit  = digitalPinToBitMask(PIN_PA5);
    PORTA.DIRSET = bit;
    bit          = digitalPinToBitMask(PIN_PA4);
    PORTA.DIRSET = bit;

        
              
}

void setup(void)
{    
  debug_serial.begin(9600);
      debug_serial.println(TCD0.CTRLA, HEX);

    PORT_init();    

    
    TCD0_init();
        TCD0_enableOutputChannels();
    debug_serial.println(TCD0.CTRLA, HEX);

}

uint32_t prev_time;
void loop() {
  uint32_t now = millis();
  if (now - prev_time > 1000) {
    debug_serial.println(now);
    prev_time = now;
  }
}
