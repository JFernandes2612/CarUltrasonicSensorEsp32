#define TRIG 25
#define ECHO 26

#define RX_PIN 16
#define TX_PIN 17

#define TONE_PIN 32

char data[3];

const unsigned char maxCallLoop = 5;
char unsigned callLoop = maxCallLoop;
uint16_t lastReading = 0;
bool atMinDistance = false;

void setup() {
  Serial.begin(115200);

  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  if (Serial2.available()) {
    if (Serial2.read() == 0xFF) {
      Serial2.readBytes(data, 3);

      uint8_t checksum = 0xFF + data[0] + data[1];

      if (checksum == data[2]) {
        uint16_t total_distance = ((uint16_t)data[0] << 8) + data[1];
        if (total_distance != 6016)
          lastReading = total_distance;
        callLoop = min(maxCallLoop, callLoop);
        callLoop--;
        if (!callLoop) {
          uint16_t meaningfull_distance = constrain(lastReading, 300, 2000);
          uint16_t toneValue = 750 - meaningfull_distance / 50;
          if (meaningfull_distance == 300 && !atMinDistance) {
            atMinDistance = true;
            tone(TONE_PIN, toneValue);
          } else if (meaningfull_distance > 300 && atMinDistance) {
            atMinDistance = false;
            noTone(TONE_PIN);
          }

          if (meaningfull_distance > 300 && !atMinDistance)
            tone(TONE_PIN, toneValue, map(meaningfull_distance, 300, 2000, 80 * (maxCallLoop + 1) + 25, 25));
        }
        Serial.printf("%dmm\n", lastReading);
      }
    }
  }
}
