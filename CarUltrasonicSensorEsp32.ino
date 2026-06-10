#include <SoftwareSerial.h>

#define RX_PIN_SENSOR_0 13
#define RX_PIN_SENSOR_1 12
#define RX_PIN_SENSOR_2 14
#define RX_PIN_SENSOR_3 27

#define TONE_PIN 32

#define NUMBER_OF_SENSORS 4

#define ERROR_DISTANCE 6016

#define FIRST_BYTE 0xFF

static const unsigned char SENSOR_READING_LOOPS = 5;

char sensorsData[NUMBER_OF_SENSORS][3];

unsigned char sensorReadingLoop = SENSOR_READING_LOOPS;
uint16_t latestSensorsDistance[NUMBER_OF_SENSORS] = {9999, 9999, 9999, 9999};
bool atMinDistance = false;

EspSoftwareSerial::UART sensors[NUMBER_OF_SENSORS];

unsigned long previousTime = 0;

void setup()
{
  Serial.begin(115200);

  sensors[0].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN_SENSOR_0, -1);
  sensors[1].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN_SENSOR_1, -1);
  sensors[2].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN_SENSOR_2, -1);
  sensors[3].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN_SENSOR_3, -1);

  sleep(1);
}

void loop()
{
  for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
  {
    if (sensors[i].available())
    {
      processSensorData(&sensors[i], sensorsData[i], &latestSensorsDistance[i]);
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - previousTime > (SENSOR_READING_LOOPS / 2) * 75)
  {
    produceTone();
    previousTime = currentTime;
  }
}

void processSensorData(EspSoftwareSerial::UART *sensor, char *data, uint16_t *latestSensorDistance)
{
  if (sensor->read() != FIRST_BYTE)
    return;

  sensor->readBytes(data, 3);

  uint8_t checksum = FIRST_BYTE + data[0] + data[1];

  if (checksum != data[2])
    return;

  uint16_t sensorDistance = ((uint16_t)data[0] << 8) + data[1];
  if (sensorDistance != ERROR_DISTANCE)
    *latestSensorDistance = sensorDistance;
}

void produceTone()
{
  sensorReadingLoop = min(SENSOR_READING_LOOPS, sensorReadingLoop);
  sensorReadingLoop--;

  if (sensorReadingLoop)
    return;

  uint16_t meaningfull_distance = constrain(latestSensorsDistance[0], 300, 2000);
  uint16_t toneValue = 750 - meaningfull_distance / 50;
  if (meaningfull_distance == 300 && !atMinDistance)
  {
    atMinDistance = true;
    tone(TONE_PIN, toneValue);
  }
  else if (meaningfull_distance > 300 && atMinDistance)
  {
    atMinDistance = false;
    noTone(TONE_PIN);
  }

  if (meaningfull_distance > 300 && !atMinDistance)
    tone(TONE_PIN, toneValue, map(meaningfull_distance, 300, 2000, 80 * (SENSOR_READING_LOOPS + 1) + 25, 25));
}
