#include <SimpleKalmanFilter.h>
#include <SoftwareSerial.h>

#define DEBUG 1

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
#define BEEP_PULSE_MS 20

#define MIN_DISTANCE 300
#define MAX_DISTANCE 2000

#define FIRST_BYTE 0xFF
#define TRIGGER_BYTE 0x01

volatile uint16_t latestSensorsDistance[NUMBER_OF_SENSORS] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};

EspSoftwareSerial::UART swSensors[2];

SimpleKalmanFilter kalman[NUMBER_OF_SENSORS] = {
    SimpleKalmanFilter(30, 30, 0.3),
    SimpleKalmanFilter(30, 30, 0.3),
    SimpleKalmanFilter(30, 30, 0.3),
    SimpleKalmanFilter(30, 30, 0.3)};

TaskHandle_t ToneTaskHandle;

void setup()
{
#if DEBUG
  Serial.begin(115200);
#endif

  Serial1.begin(9600, SERIAL_8N1, HW_RX_0, HW_TX_0);
  Serial2.begin(9600, SERIAL_8N1, HW_RX_1, HW_TX_1);

  swSensors[0].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, SW_RX_2, SW_TX_2, false, 64);
  swSensors[1].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, SW_RX_3, SW_TX_3, false, 64);

  delay(1000);

  xTaskCreatePinnedToCore(
      produceTone,
      "ToneTask",
      2048,
      NULL,
      1,
      &ToneTaskHandle,
      0);
}

void loop()
{
  for (uint8_t id = 0; id < NUMBER_OF_SENSORS; id++)
  {
    clearSerialBuffer(id);

    triggerSensor(id);

    unsigned long startTime = millis();
    unsigned long timeSpentReading = 0;
    bool packetFound = false;

    while ((timeSpentReading = (millis() - startTime)) < 40)
    {
      if (isDataAvailable(id))
      {
        if (peekSerialByte(id) == FIRST_BYTE)
        {
          delay(5);
          if (getAvailableBytes(id) >= 4)
          {
            parseSensorPacket(id);
            packetFound = true;
            break;
          }
        }
        else
        {
          readSerialByte(id);
        }
      }
      yield();
    }

    if (!packetFound)
    {
      latestSensorsDistance[id] = (uint16_t)kalman[id].updateEstimate(MAX_DISTANCE);
    }

    unsigned long detectionWindowRemainder = 40 - timeSpentReading;
    delay(100 + detectionWindowRemainder);
  }

#if DEBUG
  Serial.printf("S0 (HW): %dmm   S1 (HW): %dmm   S2 (SW): %dmm   S3 (SW): %dmm\n",
                latestSensorsDistance[0], latestSensorsDistance[1],
                latestSensorsDistance[2], latestSensorsDistance[3]);
#endif
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

    uint16_t targetDistance = constrain(sensorDistance, MIN_DISTANCE, MAX_DISTANCE);

    latestSensorsDistance[id] = (uint16_t)kalman[id].updateEstimate(targetDistance);
  }
}

void produceTone(void *pvParameters)
{
  bool atMinDistance = false;
  for (;;)
  {
    uint16_t minSensorDistance = MAX_DISTANCE;

    for (unsigned char id = 0; id < NUMBER_OF_SENSORS; id++)
      if (minSensorDistance > latestSensorsDistance[id])
        minSensorDistance = latestSensorsDistance[id];

    uint16_t toneValue = BASE_TONE - minSensorDistance / 30;

    if (minSensorDistance == MIN_DISTANCE && !atMinDistance)
    {
      atMinDistance = true;
      tone(TONE_PIN, toneValue);
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    else if (minSensorDistance > MIN_DISTANCE && atMinDistance)
    {
      atMinDistance = false;
      noTone(TONE_PIN);
    }

    if (minSensorDistance > MIN_DISTANCE && !atMinDistance)
    {
      tone(TONE_PIN, toneValue, BEEP_PULSE_MS);
      uint32_t silenceIntervalMs = map(minSensorDistance, MIN_DISTANCE, MAX_DISTANCE, 30, 500);
      vTaskDelay((BEEP_PULSE_MS + silenceIntervalMs) / portTICK_PERIOD_MS);
    }
  }
}