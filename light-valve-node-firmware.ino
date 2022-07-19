#include <avr/io.h>
#include "SoftwareSerial.h"

#define DATA_PIN PIN_PA5
#define DATA_PIN_INT_CTRL_REG (*(uint8_t *)(PORTA.PIN0CTRL + DATA_PIN))

#define LIGHT_VALVE_PIN_1 PIN_PB0  // bottom pin
#define LIGHT_VALVE_PIN_2 PIN_PB1  // top pin

#define UART_TX LIGHT_VALVE_PIN_1
#define UART_RX LIGHT_VALVE_PIN_2

#define GREEN_LED PIN_PA7  // GREEN LED
#define RED_LED PIN_PA6    // RED LED

#define HEARTBEAT_PERIOD_DEFAULT_MS (500)
#define HEARTBEAT_PERIOD_ERROR_MS (100)
/* #define CLOCK_PERIOD_PADDING_US (12000)  // TODO :: change this once we set period to actual value */

#define DEBUG_MODE 0
#if DEBUG_MODE
#define LOG(...) debug_serial.print(__VA_ARGS__)
#define LOGLN(...) debug_serial.println(__VA_ARGS__)
#else
#define LOG(...)
#define LOGLN(...)
#endif

typedef enum {
    HEARTBEAT_MODE_DEBUG,
    HEARTBEAT_MODE_HEARTBEAT,
} heartbeat_mode_t;

typedef enum {
    DECODE_STATE_IDLE,
    DECODE_STATE_MEASURING_CLOCK_RATE,
    DECODE_STATE_WAITING_FOR_START,
    DECODE_STATE_READING_START,
    DECODE_STATE_READING_DATA,
    DECODE_STATE_READING_END,
    DECODE_STATE_ERROR,

    DECODE_STATE_COUNT,
} decode_state_t;

static volatile bool data_transition_flag;

static heartbeat_mode_t heartbeat_mode;
static uint32_t         heartbeat_period_ms;
static uint32_t         previous_time;
static decode_state_t   current_state;
static uint32_t         clock_rate_start_us;
static uint32_t         clock_period_us;
static uint32_t         clock_period_padding_us;
static uint8_t          data_edges_counted;
static uint8_t          clock_edges_counted;

static uint8_t rx_start_bits;
static uint8_t rx_data_bits;

static uint32_t clock_edge_start;

/* #if DEBUG_MODE */
static SoftwareSerial debug_serial(UART_RX, UART_TX);
/* #endif */

void data_pin_isr(void) {
    data_transition_flag = true;
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

    bit = digitalPinToBitMask(RED_LED);
    PORTA.DIR |= bit;
    PORTA.DIRSET = bit;
    bit          = digitalPinToBitMask(GREEN_LED);
    PORTA.DIRSET = bit;
    PORTA.OUTCLR = 0xFF;

    bit          = digitalPinToBitMask(UART_TX);
    PORTB.DIRSET = bit;
    PORTB.OUTCLR = 0xFF;
    bit          = digitalPinToBitMask(UART_RX);
    PORTB.DIRCLR = bit;
    /* bit          = digitalPinToBitMask(LIGHT_VALVE_PIN_1); */
    /* PORTB.DIRSET = bit; */
    /* bit          = digitalPinToBitMask(LIGHT_VALVE_PIN_2); */
    /* PORTB.DIRSET = bit; */
    /* PORTB.OUTCLR = 0xFF; */

    // Globally enable interupts
    /* SREG |= (1 << 7); */

    current_state        = DECODE_STATE_IDLE;
    data_transition_flag = false;

    // Default to toggle heartbeat ever half second. Transition to rapid blink indicates error.
    heartbeat_mode      = HEARTBEAT_MODE_DEBUG;
    heartbeat_period_ms = HEARTBEAT_PERIOD_DEFAULT_MS;
    clock_rate_start_us = 0;
    clock_period_us     = (uint32_t)500 * 1000;
    previous_time       = millis();
    clock_edge_start    = 0;
    data_edges_counted  = 0;
    clock_edges_counted = 0;

    /* #if DEBUG_MODE */
    debug_serial.begin(9600);
    debug_serial.println("Light valve node firmware");
    /* #endif */
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

    if (data_transition_flag) {
        data_transition_flag = false;

        uint32_t now_us     = micros();
        uint32_t elapsed_us = now_us - clock_edge_start;
        switch (current_state) {
            case DECODE_STATE_IDLE: {
                clock_rate_start_us = now_us;

                uint8_t bit   = digitalPinToBitMask(RED_LED);
                PORTA.OUTTGL  = bit;
                current_state = DECODE_STATE_MEASURING_CLOCK_RATE;
                break;
            }
            case DECODE_STATE_MEASURING_CLOCK_RATE: {
                clock_period_us         = now_us - clock_rate_start_us;
                clock_period_padding_us = clock_period_us / 10;
                LOG("clck per us: ");
                LOGLN(clock_period_us);

                uint8_t bit   = digitalPinToBitMask(RED_LED);
                PORTA.OUTTGL  = bit;
                current_state = DECODE_STATE_WAITING_FOR_START;
                break;
            }
            case DECODE_STATE_WAITING_FOR_START: {
                data_edges_counted  = 0;
                clock_edges_counted = 0;
                clock_edge_start    = now_us;
                rx_start_bits       = 0x0;

                current_state = DECODE_STATE_READING_START;
                break;
            }
            case DECODE_STATE_READING_START: {
                if (elapsed_us >= (clock_period_us - clock_period_padding_us) &&
                    elapsed_us <= (clock_period_us + clock_period_padding_us)) {
                    // Close to what our clock period should be,  restart clock_edge_start to right now (and potentially
                    // update clock_period_us running avg) no matter what the stage of measurement we're in.
                    clock_edge_start = now_us;
                    clock_edges_counted++;

                    // Push new data bit
                    rx_start_bits <<= 1;
                    switch (data_edges_counted) {
                        case 0: {
                            LOGLN("rx 1");
                            rx_start_bits |= 0x1;
                            break;
                        }
                        case 1: {
                            LOGLN("rx 0");
                            rx_start_bits |= 0x0;
                            break;
                        }
                        default:
                            debug_serial.print("err DSRS");
                            debug_serial.println(data_edges_counted);
                            current_state = DECODE_STATE_ERROR;
                            break;
                    }

                    data_edges_counted = 0;

                    // If we've reached the expected number of start clock periods, check preamble and ideally
                    // transition to read data state. Otherwise increment clock_edges_counted and reset
                    // data_edges_counted for the next clock period
                    if (clock_edges_counted == 4) {
                        // RX all expected clock ticks for preamble, check for expected value
                        LOG("start bits rx: 0b");
                        LOGLN(rx_start_bits, BIN);
                        if (rx_start_bits == 0xA) {
                            clock_edges_counted = 0;
                            rx_data_bits        = 0x00;
                            uint8_t bit         = digitalPinToBitMask(GREEN_LED);
                            PORTA.OUTTGL        = bit;

                            current_state = DECODE_STATE_READING_DATA;
                        } else {
                            debug_serial.println("err DSRS strtbts");
                            debug_serial.print("start bits: 0b");
                            debug_serial.println(rx_start_bits, BIN);
                            current_state = DECODE_STATE_ERROR;
                        }
                    }
                } else {
                    // Increment edge count since we haven't reached the end of the expected clock period yet
                    data_edges_counted++;
                }

                break;
            }
            case DECODE_STATE_READING_DATA: {
                if (elapsed_us >= (clock_period_us - clock_period_padding_us) &&
                    elapsed_us <= (clock_period_us + clock_period_padding_us)) {
                    // Close to what our clock period should be,  restart clock_edge_start to right now (and potentially
                    // update clock_period_us running avg) no matter what the stage of measurement we're in.
                    clock_edge_start = now_us;
                    clock_edges_counted++;

                    // Push new data bit
                    rx_data_bits <<= 1;
                    switch (data_edges_counted) {
                        case 0: {
                            LOGLN("rx 1");
                            rx_data_bits |= 0x1;
                            break;
                        }
                        case 1: {
                            LOGLN("rx 0");
                            rx_data_bits |= 0x0;
                            break;
                        }
                        default:
                            debug_serial.print("err DSRD");
                            debug_serial.println(data_edges_counted);
                            current_state = DECODE_STATE_ERROR;
                            break;
                    }

                    data_edges_counted = 0;

                    // If we've reached the expected number of data clock periods, transition to check address state.
                    // Otherwise increment clock_edges_counted and reset data_edges_counted for the next clock period
                    if (clock_edges_counted == 8) {
                        // RX all expected clock ticks for preamble, check for expected value
                        LOG("data bits rx: 0x");
                        LOGLN(rx_data_bits, HEX);
                        clock_edges_counted = 0;

                        debug_serial.print("start bits: 0b");
                        debug_serial.println(rx_start_bits, BIN);
                        debug_serial.print("addr bits: 0b");
                        debug_serial.println((rx_data_bits & 0xF0) >> 4, BIN);
                        debug_serial.print("data bits: 0b");
                        debug_serial.println((rx_data_bits & 0x0F), BIN);

                        current_state = DECODE_STATE_IDLE;
                    }
                } else {
                    // Increment edge count since we haven't reached the end of the expected clock period yet
                    data_edges_counted++;
                }

                break;
            }
            case DECODE_STATE_READING_END:
                current_state = DECODE_STATE_ERROR;
                break;
            case DECODE_STATE_ERROR:
                heartbeat_mode      = HEARTBEAT_MODE_HEARTBEAT;
                heartbeat_period_ms = HEARTBEAT_PERIOD_ERROR_MS;
                break;
            default:
                break;
        }
    }

    // Heartbeat for debug
    unsigned long now = millis();
    if ((now - previous_time > heartbeat_period_ms)) {
        if (heartbeat_mode == HEARTBEAT_MODE_HEARTBEAT) {
            uint8_t bit   = digitalPinToBitMask(RED_LED);
            PORTA.OUTTGL  = bit;
            previous_time = now;
        }
    }
}
