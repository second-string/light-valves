unsigned long last_print_millis;

void setup() {
  Serial.begin(115200);

  last_print_millis = 0;

  pinMode(A0, INPUT);

  pinMode(9, OUTPUT);                                              // Set digital pin 11 as an output
  pinMode(10, OUTPUT);                                              // Set digital pin 12 as an output
  GTCCR = _BV(TSM) | _BV(PSRSYNC);                                  // Halt and synchronize the timers
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0);                  // Enable the PWM outputs OC1A, OC1B on digital pins 9 and 10         
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);         // Set fast PWM and prescaler of 8 on timer 1  (add CS10 for 64 prescaler)
  ICR1 = 100;                                                     // Set the timer 1 frequency to 20kHz
  OCR1A = 50;                                                     // Set the timer 1, channel A duty-cycle to 50%
  OCR1B = 50;                                                     // Set the timer 1, channel B duty-cycle to 50%
  TCNT1 = 0;                                                        // Set the timer 1 count to 0
  GTCCR = 0;     
}

void loop() {
  /*
  // put your main code here, to run repeatedly:
  int raw_pot = analogRead(A0);
  int mapped_pot = map(raw_pot, 0, 1023, 0, 100);
  // int new_period_ticks = (16000000 / mapped_pot / 64) - 1;  // Convert pwm period to counter ticks, assumes 64 prescaler
  int new_duty_ticks = (2500 * mapped_pot) / 100;
  // ICR1 = new_period_ticks;
  OCR1A = new_duty_ticks;
  OCR1B = 2500 - new_duty_ticks;
  // Serial.print("Updated PWM period to ");
  if ((millis() - last_print_millis) > 500) { 
        last_print_millis = millis();
    Serial.print("Updated duty to ");
    Serial.print(new_duty_ticks);
    Serial.print(" ticks (");
    Serial.print(mapped_pot);
    Serial.println("%)");
  }
  delay(20);
  */
}
