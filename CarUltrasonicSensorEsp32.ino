#include <SimpleKalmanFilter.h>
#include <SoftwareSerial.h>

#define NUMBER_OF_SENSORS 4

#define HW_RX_0 13
#define HW_TX_0 4
#define HW_RX_1 26
#define HW_TX_1 2

#define SW_RX_2 25
#define SW_TX_2 33
#define SW_RX_3 27
#define SW_TX_3 15

#define TONE_PIN 32
#define BASE_TONE 750

#define ERROR_DISTANCE 6016
#define MIN_DISTANCE 300
#define MAX_DISTANCE 2000

#define FIRST_BYTE 0xFF
#define TRIGGER_BYTE 0x01

char sensorsData[NUMBER_OF_SENSORS][3];
uint16_t latestSensorsDistance[NUMBER_OF_SENSORS] = {9999, 9999, 9999, 9999};
bool atMinDistance = false;

EspSoftwareSerial::UART swSensors[2];
unsigned long previousTelemetryTime = 0;

SimpleKalmanFilter kalman[NUMBER_OF_SENSORS] = {
    SimpleKalmanFilter(30, 30, 0.5),
    SimpleKalmanFilter(30, 30, 0.5),
    SimpleKalmanFilter(30, 30, 0.5),
    SimpleKalmanFilter(30, 30, 0.5)};

void setup()
{
  Serial.begin(115200);

  Serial1.begin(9600, SERIAL_8N1, HW_RX_0, HW_TX_0);
  Serial2.begin(9600, SERIAL_8N1, HW_RX_1, HW_TX_1);

  swSensors[0].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, SW_RX_2, SW_TX_2, false, 64);
  swSensors[1].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, SW_RX_3, SW_TX_3, false, 64);

  delay(1000);
}

void loop()
{
  for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++)
  {
    clearSerialBuffer(i);

    triggerSensor(i);

    unsigned long startTime = millis();
    unsigned long timeSpentReading = 0;

    while ((timeSpentReading = (millis() - startTime)) < 40)
    {
      if (isDataAvailable(i))
      {
        if (peekSerialByte(i) == FIRST_BYTE)
        {
          delay(5);
          if (getAvailableBytes(i) >= 4)
          {
            parseSensorPacket(i);
            break;
          }
        }
        else
        {
          readSerialByte(i);
        }
      }
      yield();
    }

    unsigned long detectionWindowRemainder = 40 - timeSpentReading;
    delay(100 + detectionWindowRemainder);
  }

  produceTone();
  Serial.printf("S0 (HW): %dmm   S1 (HW): %dmm   S2 (SW): %dmm   S3 (SW): %dmm\n",
                latestSensorsDistance[0], latestSensorsDistance[1],
                latestSensorsDistance[2], latestSensorsDistance[3]);
}

void triggerSensor(uint8_t id)
{
  if (id == 0)
    Serial1.write(TRIGGER_BYTE);
  else if (id == 1)
    Serial2.write(TRIGGER_BYTE);
  else
    swSensors[id - 2].write(TRIGGER_BYTE);
}

bool isDataAvailable(uint8_t id)
{
  if (id == 0)
    return Serial1.available() > 0;
  else if (id == 1)
    return Serial2.available() > 0;
  else
    return swSensors[id - 2].available() > 0;
}

int getAvailableBytes(uint8_t id)
{
  if (id == 0)
    return Serial1.available();
  else if (id == 1)
    return Serial2.available();
  else
    return swSensors[id - 2].available();
}

uint8_t peekSerialByte(uint8_t id)
{
  if (id == 0)
    return Serial1.peek();
  else if (id == 1)
    return Serial2.peek();
  else
    return swSensors[id - 2].peek();
}

uint8_t readSerialByte(uint8_t id)
{
  if (id == 0)
    return Serial1.read();
  else if (id == 1)
    return Serial2.read();
  else
    return swSensors[id - 2].read();
}

void clearSerialBuffer(uint8_t id)
{
  if (id == 0)
    while (Serial1.available() > 0)
      Serial1.read();
  else if (id == 1)
    while (Serial2.available() > 0)
      Serial2.read();
  else
    while (swSensors[id - 2].available() > 0)
      swSensors[id - 2].read();
}

void parseSensorPacket(uint8_t id)
{
  readSerialByte(id);

  char data[3];
  data[0] = readSerialByte(id);
  data[1] = readSerialByte(id);
  data[2] = readSerialByte(id);

  uint8_t calculatedChecksum = FIRST_BYTE + (uint8_t)data[0] + (uint8_t)data[1];

  if (calculatedChecksum == (uint8_t)data[2])
  {
    uint16_t sensorDistance = ((uint16_t)(uint8_t)data[0] << 8) + (uint8_t)data[1];
    if (sensorDistance != ERROR_DISTANCE)
    {
      latestSensorsDistance[id] = (uint16_t)kalman[id].updateEstimate(sensorDistance);
    }
  }
}

void produceTone()
{
  uint16_t minSensorDistance = 9999;

  for (unsigned char i = 0; i < NUMBER_OF_SENSORS; i++)
    if (minSensorDistance > latestSensorsDistance[i])
      minSensorDistance = latestSensorsDistance[i];

  uint16_t constrainedMinSensorDistance = constrain(minSensorDistance, MIN_DISTANCE, MAX_DISTANCE);
  uint16_t toneValue = BASE_TONE - constrainedMinSensorDistance / 30;
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
    tone(TONE_PIN, toneValue, map(constrainedMinSensorDistance, MIN_DISTANCE, MAX_DISTANCE, 550, 10));
}