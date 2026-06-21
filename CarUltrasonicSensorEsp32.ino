#include <SoftwareSerial.h>

#define NUMBER_OF_SENSORS 4

#define RX_PIN_SENSOR_0 13
#define RX_PIN_SENSOR_1 26
#define RX_PIN_SENSOR_2 25
#define RX_PIN_SENSOR_3 27

#define TONE_PIN 32
#define BASE_TONE 750

#define ERROR_DISTANCE 6016
#define MIN_DISTANCE 300
#define MAX_DISTANCE 2000

#define FIRST_BYTE 0xFF

char sensorsData[NUMBER_OF_SENSORS][3];

uint16_t latestSensorsDistance[NUMBER_OF_SENSORS] = {9999, 9999, 9999, 9999};
bool atMinDistance = false;

EspSoftwareSerial::UART sensors[NUMBER_OF_SENSORS];

unsigned long previousTime = 0;

void setup()
{
  Serial.begin(115200);

  sensors[0].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN_SENSOR_0, -1, false, 4);
  sensors[0].enableIntTx(false);
  sensors[0].enableTx(false);
  sensors[1].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN_SENSOR_1, -1, false, 4);
  sensors[1].enableIntTx(false);
  sensors[1].enableTx(false);
  sensors[2].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN_SENSOR_2, -1, false, 4);
  sensors[2].enableIntTx(false);
  sensors[2].enableTx(false);
  sensors[3].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, RX_PIN_SENSOR_3, -1, false, 4);
  sensors[3].enableIntTx(false);
  sensors[3].enableTx(false);

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
  if (currentTime - previousTime > 500)
  {
    produceTone();
    previousTime = currentTime;

    Serial.printf("S0: %dmm   S1: %dmm   S2: %dmm   S3: %dmm\n", latestSensorsDistance[0], latestSensorsDistance[1], latestSensorsDistance[2], latestSensorsDistance[3]);
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
  uint16_t minSensorDistance = 9999;

  for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
    if (minSensorDistance > latestSensorsDistance[i])
      minSensorDistance = latestSensorsDistance[i];

  uint16_t constrainedMinSensorDistance = constrain(minSensorDistance, MIN_DISTANCE, MAX_DISTANCE);
  uint16_t toneValue = BASE_TONE - constrainedMinSensorDistance / 50;
  if (constrainedMinSensorDistance == MIN_DISTANCE && !atMinDistance)
  {
    atMinDistance = true;
    tone(TONE_PIN, toneValue);
  }
  else if (constrainedMinSensorDistance > MIN_DISTANCE && atMinDistance)
  {
    atMinDistance = false;
    noTone(TONE_PIN);
  }

  if (constrainedMinSensorDistance > MIN_DISTANCE && !atMinDistance)
    tone(TONE_PIN, toneValue, map(constrainedMinSensorDistance, MIN_DISTANCE, MAX_DISTANCE, 450, 25));
}
