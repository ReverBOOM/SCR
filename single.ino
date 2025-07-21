#define PULSE_PIN 9
#define ANALOG_PIN A0
#define IN_PIN 2

volatile bool trigger = false;

void setup() {
    pinMode(PULSE_PIN, OUTPUT);
    pinMode(ANALOG_PIN, INPUT);
    pinMode(IN_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(IN_PIN), zeroCrossISR, RISING);
    Serial.begin(9600);
}

void loop() {
    if (trigger) {
        trigger = false;

        // Read analog value (0–1023)
        int analogValue = analogRead(ANALOG_PIN);

        // Map to delay time: e.g., 0–180 degrees = 0–10 ms delay (for 50Hz, half-cycle = 10ms)
        int delayTime = map(analogValue, 0, 1023, 0, 10000);  // in microseconds (0 to 10 ms)

        delayMicroseconds(delayTime); // wait before triggering the SCR

        // Send 2ms pulse to SCR
        digitalWrite(PULSE_PIN, HIGH);
        delay(2);  // 2 ms pulse
        digitalWrite(PULSE_PIN, LOW);
    }
}

void zeroCrossISR() {
    trigger = true;
}
