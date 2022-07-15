#include <avr/io.h>

#define DATA_PIN PIN_PA5
#define DATA_PIN_INT_CTRL_REG (*(uint8_t *)(PORTA.PIN0CTRL + DATA_PIN))

#define LIGHT_VALVE_PIN_1 PIN_PB0
#define LIGHT_VALVE_PIN_2 PIN_PB1

#define DEBUG_PIN_1 PIN_PA7
#define DEBUG_PIN_2 PIN_PA6

typedef enum {
    DECODE_STATE_IDLE,
    DECODE_STATE_MEASURING_CLOCK_RATE,

    DECODE_STATE_COUNT,
} decode_state_t;

static int                     previous_time;
static volatile decode_state_t current_state;
static volatile unsigned long  clock_rate_start_us;
static volatile unsigned long  clock_period_us;

void data_pin_isr(void) {
    uint8_t bit  = digitalPinToBitMask(DEBUG_PIN_1);
    PORTA.OUTTGL = bit;
    switch (current_state) {
        case DECODE_STATE_IDLE:
            clock_rate_start_us = micros();
            current_state       = DECODE_STATE_MEASURING_CLOCK_RATE;
            break;
        case DECODE_STATE_MEASURING_CLOCK_RATE:
            clock_period_us = micros() - clock_rate_start_us;
            current_state   = DECODE_STATE_IDLE;
            break;
        default:
            uint8_t bit  = digitalPinToBitMask(DEBUG_PIN_2);
            PORTA.OUTSET = bit;
            break;
    }
}

void setup() {
    // - set up data pin as input and 2 light valve pins + test GPIO as output and low
    // - start pwm for output valve pins at either fully clear or fully opaque (TODO :: run a fancy startup or bink to
    // indicate stored node ID)

    //////////////////////////
    // If running with simple edge-triggered interrupt:
    // - set up interrupt for rising and falling edges to trigger for data pin
    // - set up light valve pins as output w/ PWM support
    // - set up timer with no period, that will be set from second pair of transitions in
    /////////////////////////
    uint8_t bit  = digitalPinToBitMask(DATA_PIN);
    PORTA.DIRCLR = bit;
    /* DATA_PIN_INT_CTRL_REG |= 0x1;   // interrupt on both edges - attiny 212 DS sec. 16.5.11 */
    attachInterrupt(DATA_PIN, data_pin_isr, CHANGE);

    bit = digitalPinToBitMask(DEBUG_PIN_2);
    PORTA.DIR |= bit;
    PORTA.DIRSET = bit;
    bit          = digitalPinToBitMask(DEBUG_PIN_1);
    PORTA.DIRSET = bit;
    PORTA.OUTCLR = 0xFF;

    bit          = digitalPinToBitMask(LIGHT_VALVE_PIN_1);
    PORTB.DIRSET = bit;
    bit          = digitalPinToBitMask(LIGHT_VALVE_PIN_2);
    PORTB.DIRSET = bit;
    PORTB.OUTCLR = 0xFF;

    // Globally enable interupts
    /* SREG |= (1 << 7); */

    current_state       = DECODE_STATE_IDLE;
    clock_rate_start_us = 0;
    clock_period_us     = ((unsigned long)500) * 1000;
    previous_time       = millis();
}

void loop() {
    //////////////////////////
    // If running with simple edge-triggered interrupt:
    // - while in overall wait state, spin until receive a transition interrupt
    // - transition to preamble state and start timer
    // - on next edge interrupt, stop timer
    //   - if within expected bounds of preamble (+/- some microseconds), transition to clock sense state
    //   - if not, transition back to overall wait state
    // - on next transition, start an unbounded timer
    // - on next transition, save timer count as expected clock period and transition to data sense state
    // decode implementation 1:
    // - on next transition, start a timer bounded to expected clock period
    // - on next transition, check timer count
    //   - if it's 1/2 (give or take) expected clock period, it's a 0
    //   - if it's full (give or take) expected clock period, it's a 1
    // decode implementation 2:
    // - on next transition, start a timer bounded to 3/4 expected clock period
    //   - if a transition interrupt fires, it's 0. Stop timer and set callback for next transition to start 3/4 period
    //   bounded timer
    //   - if timer expires w/ no transition interrupt firing, it's a 1. Set callback for next transition to start 3/4
    //   period bounded timer
    // decode implementation 3:
    // - manually set timer count to at or near clock period and set single handler to run every transition interrupt
    // - every transition:
    //   - if timer count at or near clock period:
    //     - check flag indicating a 1/2 period transition and push a 0 if set, push a 1 if not (need to special case
    //     the very first run of this per-message so don't push any data but we do start timer).
    //     - reset flag
    //     - restart unbounded timer
    //     - can also use this timer count to adjust clock period w/ running avg
    //   - else set the 1/2 period transition flag
    //
    // - not sure if we should check 8-bit address once we get those bits or continue to read the next 8-bit data value
    // no matter what to prevent dropped data
    //    - if address does not equal our stored node ID, return to overall wait state
    //    - if address equals our stored node ID, translate rx byte to PWM period and write to both output PWM registers

    //   - if it's 1/4 overall baud rate period it's a 1. Need to somehow bail out of second transition counting (or
    //   still measure it to ensure it's the same 1/4 period)
    //   - if it's 1/2 overall baud rate period, it's a 0. Immediately start the next measurement period on  the next
    //   transiton
    /////////////////////////

    unsigned long now = millis();
    if (now - previous_time > (clock_period_us / 1000)) {
        uint8_t bit  = digitalPinToBitMask(DEBUG_PIN_2);
        PORTA.OUTTGL = bit;

        previous_time = now;
    }
}
