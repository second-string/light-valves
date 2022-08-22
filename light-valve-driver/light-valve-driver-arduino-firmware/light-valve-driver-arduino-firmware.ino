#include <assert.h>

#define DATA_OUT_PIN 10
#define HEARTBEAT_PIN LED_BUILTIN
#define DEBUG_INPUT_PIN 8

#define CLOCK_PERIOD_US (50)

static uint16_t heartbeat_period_ms;
static uint32_t previous_heartbeat_time;
static uint8_t  prev_input_level;
static uint32_t last_transition;

void send_bit(uint8_t bit) {
    assert(bit == 0 || bit == 1);

    /* uint32_t now_ms = millis(); */
    uint8_t level = digitalRead(DATA_OUT_PIN);
    if (bit) {
        delayMicroseconds(CLOCK_PERIOD_US);
        digitalWrite(DATA_OUT_PIN, !level);
        // digitalWrite(HEARTBEAT_PIN, !level);
    } else {
        delayMicroseconds(CLOCK_PERIOD_US / 2);
        digitalWrite(DATA_OUT_PIN, !level);
        // digitalWrite(HEARTBEAT_PIN, !level);
        delayMicroseconds(CLOCK_PERIOD_US / 2);
        digitalWrite(DATA_OUT_PIN, level);
        // digitalWrite(HEARTBEAT_PIN, level);
    }
}

void send_test_packet(uint8_t addr_nibble, uint8_t data_nibble) {
    // Initial transition
    uint8_t level = digitalRead(DATA_OUT_PIN);
    digitalWrite(DATA_OUT_PIN, !level);
    // digitalWrite(HEARTBEAT_PIN, !level);

    // send clock rate
    send_bit(1);

    // waiting for start state between clock rate and preamble bits
    send_bit(1);

    // start bits
    send_bit(1);
    send_bit(0);
    send_bit(1);
    send_bit(0);

    // address bits
    for (int i = 0; i < 4; i++) {
        send_bit((addr_nibble >> (3 - i)) & 0x1);
    }

    // address bits
    for (int i = 0; i < 4; i++) {
        send_bit((data_nibble >> (3 - i)) & 0x1);
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(DATA_OUT_PIN, OUTPUT);
    pinMode(HEARTBEAT_PIN, OUTPUT);
    pinMode(DEBUG_INPUT_PIN, INPUT);

    digitalWrite(DATA_OUT_PIN, HIGH);

    heartbeat_period_ms = 2000;
    delay(3000);
    uint8_t addr = 0x1;
    for (int i = 0; i < 32; i++) {
        if (i % 2 == 0) {
            addr = 0x1;
        } else {
            addr = 0x2;
        }

        send_test_packet(addr, i);
        delay(2);
    }
}

void loop() {
    uint32_t now_ms      = millis();
    uint8_t  input_level = digitalRead(DEBUG_INPUT_PIN);
    if (now_ms - previous_heartbeat_time > heartbeat_period_ms) {
        // send_test_packet(0x1);
        // delay(2);
        // send_test_packet(0x1);
        // delay(2);
        // send_test_packet(0x1);

        // Serial.println("seent");
        previous_heartbeat_time = millis();
    }

    if (prev_input_level == HIGH && input_level == LOW) {
        Serial.println("button press");

        // send_test_packet();
    }

    prev_input_level = input_level;
}
