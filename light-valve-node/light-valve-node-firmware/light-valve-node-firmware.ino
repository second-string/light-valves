#include <avr/io.h>

#define DATA_PIN PIN_PA3
#define DATA_PIN_INT_CTRL_REG (*(uint8_t *)(PORTA.PIN0CTRL + DATA_PIN))

#define LIGHT_VALVE_PIN_A PIN_PA6
#define LIGHT_VALVE_PIN_B PIN_PA7

#define UART_TX PIN_PA1
#define UART_RX PIN_PA2

#define HEARTBEAT_PERIOD_DEFAULT_MS (1000)
#define HEARTBEAT_PERIOD_ERROR_MS (10)

#define DEBUG_MODE 0
#if DEBUG_MODE
#define LOG(...) Serial.print(__VA_ARGS__)
#define LOGLN(...) Serial.println(__VA_ARGS__)
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

// TODO :: non-linear opacity mapping here. Lowest values show no change (0 to 10, 10 to 20), then seems solid for a
// bit, then highest values indistinguishable w/o light behind valve
static const uint16_t pwm_counts_by_data_index[16] = {
    0,
    10,
    20,
    30,
    50,
    70,
    90,
    120,
    150,
    180,
    210,
    240,
    270,
    310,
    360,
    410,
};

static volatile bool data_transition_flag;
static volatile bool timeout;

static heartbeat_mode_t heartbeat_mode;
static uint32_t         heartbeat_period_ms;
static uint32_t         previous_time;
static decode_state_t   current_state;

static uint32_t clock_edge_start;
static uint32_t clock_rate_start_us;
static uint32_t clock_period_us;
static uint32_t clock_period_padding_us;
static uint8_t  data_edges_counted;
static uint8_t  clock_edges_counted;
static uint16_t edges = 0;

static uint8_t rx_start_bits;
static uint8_t rx_data_bits;
static uint8_t device_addr    = 0x1;  // TODO :: eventually this is set dynamically
static uint8_t num_packets_rx = 0;

// TODO :: debugging delete
uint8_t  elapsed[4];
uint16_t elapsed_idx = 0;
uint8_t  timeouts    = 0;

void data_pin_isr(void) {
    data_transition_flag = true;
}

ISR(TCA0_OVF_vect) {
    // immediately clear overflow flag
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
    timeout              = true;
}

/*
 * Set up timer to expected timeout values and enabled interrupt. Does not start timer
 */
void timeout_timer_init(void) {
    // With prescaler of 16 @ 20MHz (aka 1.25MHz) 2000 tick period is 1.6ms. Should be se so if in worst case there's
    // one extra transition at the end of the packet, timeout timer will still expire and reset state before next
    // packet starts.
    // TCA0.SINGLE.PER = 8125 - 1;
    // TCA0.SINGLE.PER = 2000 - 1;
    TCA0.SINGLE.PER = 400 - 1;
    TCA0.SINGLE.CNT = 0;

    // Divide 20MHz down to 5MHz
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc;

    // Set to single mode not split 8-bit mode
    TCA0.SINGLE.CTRLD = 0x00;

    // Enable interrupts just for overflow
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;

    timeout = false;
}

/*
 * Start counting and enable interrupts
 */
void timeout_timer_start(void) {
    // Clear all TCA0 interrupt flags since they can trigger even when periph not enabled
    TCA0.SINGLE.INTFLAGS = 0xF;

    // Set enable bit to turn it on
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}

/*
 * Stop counting but don't touch interrupts, they will preemptively be cleared before starting timer again
 */
void timeout_timer_stop(void) {
    // Clear enable bit to turn it off
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
}

/*
 * Resetting CNT while it's counting is safe as it preempts any other counter commands (UPDATE, etc).
 */
void timeout_timer_refresh(void) {
    TCA0.SINGLE.CNT = 0;
}

void pwm_timer_init(void) {
    // Enable-protected reg must have 0 written to enable bit before any other bits can be changed, and it defaults to
    // enabled on startup
    TCD0.CTRLA &= ~TCD_ENABLE_bm;

    // Don't need overlapping PWM signals so just do oneramp
    TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;

    // Disable all input control
    TCD0.INPUTCTRLA = TCD_INPUTMODE_NONE_gc;
    TCD0.INPUTCTRLB = TCD_INPUTMODE_NONE_gc;

    // Enable WOA and WOB (don't need C and D). Since FAULTCTRL reg is write protected we need to make it editable in
    // CCP reg
    CPU_CCP        = CCP_IOREG_gc;
    TCD0.FAULTCTRL = TCD_CMPAEN_bm | TCD_CMPBEN_bm;

    // We will always start high with A output and end high with B output. The clear for A and
    // set for B values are what are adjusted based on desired duty cycle
    TCD0.CMPASET = 0x000;
    TCD0.CMPBCLR = 0xFFF;

    TCD0.CMPACLR = 0x7FF;
    TCD0.CMPBSET = 0x801;

    // System 20mhz clock w/ largest prescaler possible of 32 but no synchronization prescaler. Must be done as last
    // operation before starting timer with ENABLE bit
    // NOTE :: DEFAULT TIMER FOR MILLIS/MICROS FUNCTIONS ON 1-SERIES ATTINIES IS TCD. YOU MUST CHANGE THIS TO TCB IN
    // ARDUINO PORT SETTINGS
    TCD0.CTRLA = TCD_CLKSEL_20MHZ_gc | TCD_CNTPRES_DIV32_gc | TCD_SYNCPRES_DIV1_gc;
}

void pwm_timer_start(void) {
    while (!(TCD0.STATUS & TCD_ENRDY_bm)) {
        ;
    }

    TCD0.CTRLA |= TCD_ENABLE_bm;
}

void pwm_timer_sync(void) {
    TCD0.CTRLE = TCD_SYNCEOC_bm;
}

void light_valve_set_opacity(uint8_t opacity_index) {
    TCD0.CMPACLR = 0x000 + pwm_counts_by_data_index[opacity_index];
    TCD0.CMPBSET = 0xFFF - pwm_counts_by_data_index[opacity_index];

    // Values will never load into TCD buffers until re synced
    pwm_timer_sync();
}

void uart_init(void) {
    // Must init pin dirs manually, not automatically taken care of in Serial.begin like it is for default pins
    uint8_t bit  = digitalPinToBitMask(UART_TX);
    PORTA.DIRSET = bit;
    bit          = digitalPinToBitMask(UART_RX);
    PORTA.DIRCLR = bit;

    Serial.begin(9600);

    // Set alternate pins for USART0, default is on our light valve pins.
    // Must come AFTER Serial.begin, otherwise it will be overriden/ineffective
    PORTMUX.CTRLB |= PORTMUX_USART0_ALTERNATE_gc;

    Serial.println("LVNFW");
}

void setup() {
    timeout_timer_init();
    pwm_timer_init();
    uart_init();

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

    bit          = digitalPinToBitMask(LIGHT_VALVE_PIN_A);
    PORTA.DIRSET = bit;
    bit          = digitalPinToBitMask(LIGHT_VALVE_PIN_B);
    PORTA.DIRSET = bit;
    PORTA.OUTCLR = 0xFF;

    // Init globals
    current_state        = DECODE_STATE_IDLE;
    data_transition_flag = false;
    heartbeat_mode       = HEARTBEAT_MODE_HEARTBEAT;
    heartbeat_period_ms  = HEARTBEAT_PERIOD_DEFAULT_MS;
    clock_rate_start_us  = 0;
    clock_period_us      = (uint32_t)100 * 1000;
    previous_time        = millis();
    clock_edge_start     = 0;
    data_edges_counted   = 0;
    clock_edges_counted  = 0;

    // Globally enable interupts
    sei();
    pwm_timer_start();
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
    /////////////////////////

    // Timer is turned off unless we're actively decoding a packet so we don't need to check for state here. If there is
    // a timeout, reset state back to idle an stop timer
    if (timeout) {
        timeout = false;

        timeout_timer_stop();
        elapsed_idx   = edges;
        current_state = DECODE_STATE_IDLE;
        timeouts++;
    }

    if (data_transition_flag) {
        edges++;

        data_transition_flag = false;

        // Reset timer every time we get an incoming transition to trigger if we dont get all bits expected in a packet
        timeout_timer_refresh();

        uint32_t now_us     = micros();
        uint32_t elapsed_us = now_us - clock_edge_start;
        switch (current_state) {
            case DECODE_STATE_IDLE: {
                // Got (assumed) first edge of a packet, start measuring clock rate and kick off timeout timer
                clock_rate_start_us = now_us;
                timeout_timer_start();

                current_state = DECODE_STATE_MEASURING_CLOCK_RATE;
                break;
            }
            case DECODE_STATE_MEASURING_CLOCK_RATE: {
                clock_period_us = now_us - clock_rate_start_us;

                // TODO :: tune this
                // At clock periods shorter than 100uS, can't use division anymore
                clock_period_padding_us = clock_period_us >> 2;

                LOG("clck per us: ");
                LOGLN(clock_period_us);
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
                // ONLY CHECK THE BOTTOM BOUND HERE! If elapsed time is less than expected clock period with some
                // padding, count it as a data pulse. Do NOT bound the upper end, as if the intended clock edge is a
                // little late, we'd count it as a data edge, which would then throw everything off. We don't care if
                // it's a bit over as we'll re-sync the clock at whatever time it is and continue using the same period,
                // and if it's too long the timeout will take care of state
                if (elapsed_us >= (clock_period_us - clock_period_padding_us)) {
                    // Restart clock_edge_start to right now (and potentially update clock_period_us running avg) no
                    // matter what the stage of measurement we're in.
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
                            // Serial.print("err DSRS");
                            // Serial.println(data_edges_counted);
                            current_state = DECODE_STATE_IDLE;
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
                            current_state       = DECODE_STATE_READING_DATA;
                        } else {
                            LOG("err strtbts: 0b");
                            LOGLN(rx_start_bits, BIN);
                            current_state = DECODE_STATE_IDLE;
                        }
                    }
                } else {
                    // Increment edge count since we haven't reached the end of the expected clock period yet
                    data_edges_counted++;
                }

                break;
            }
            case DECODE_STATE_READING_DATA: {
                // ONLY CHECK THE BOTTOM BOUND HERE! If elapsed time is less than expected clock period with some
                // padding, count it as a data pulse. Do NOT bound the upper end, as if the intended clock edge is a
                // little late, we'd count it as a data edge, which would then throw everything off. We don't care if
                // it's a bit over as we'll re-sync the clock at whatever time it is and continue using the same period,
                // and if it's too long the timeout will take care of state
                if (elapsed_us >= (clock_period_us - clock_period_padding_us)) {
                    // Restart clock_edge_start to right now (and potentially
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
                            LOG("err DSRD");
                            LOGLN(data_edges_counted);
                            current_state = DECODE_STATE_IDLE;
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

                        // Received full packet, turn off timeout timer. Will be re-enabled in DECODE_STATE_IDLE on
                        // first edge of next packet RXd
                        timeout_timer_stop();
                        uint8_t rx_addr_bits = (rx_data_bits & 0xF0) >> 4;
                        if (rx_addr_bits == device_addr) {
                            light_valve_set_opacity(rx_data_bits & 0x0F);
                            num_packets_rx++;

                            LOG("rx data: 0b");
                            LOGLN((rx_data_bits & 0x0F), BIN);
                        } else {
                            LOG("rx wrong addr: ");
                            LOGLN(rx_addr_bits, BIN);
                        }

                        // uint8_t bit   = digitalPinToBitMask(RED_LED);
                        // PORTA.OUTTGL  = bit;
                        current_state = DECODE_STATE_IDLE;
                    }
                } else {
                    // Increment edge count since we haven't reached the end of the expected clock period yet
                    data_edges_counted++;
                }

                break;
            }
            case DECODE_STATE_READING_END:
                current_state = DECODE_STATE_IDLE;
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
    unsigned long  now     = millis();
    static uint8_t opacity = 0;
    if ((now - previous_time > heartbeat_period_ms)) {
        light_valve_set_opacity(opacity);
        opacity++;
        if (opacity > 15) {
            opacity = 0;
        }
        LOG("Opacity idx: ");
        LOGLN(opacity);

        // Serial.print(timeouts);
        // delay(50);
        // Serial.print('-');
        // delay(50);
        // Serial.println(num_packets_rx);
        timeouts       = 0;
        num_packets_rx = 0;

        // Serial.println(elapsed_idx);
        // elapsed_idx = 0;
        // edges       = 0;

        previous_time = now;
    }
}
