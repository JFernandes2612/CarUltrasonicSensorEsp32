#include <SimpleKalmanFilter.h>
#include <SoftwareSerial.h>

#define NUMBER_OF_SENSORS 4

// Hardware Serial Pins (Sensor 0 and 1) - 100% Boot-Safe Pins
#define HW_RX_0 13
#define HW_TX_0 4
#define HW_RX_1 26
#define HW_TX_1 2

// Software Serial Pins (Sensor 2 and 3) - 100% Output-Capable Pins
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
  SimpleKalmanFilter(30, 30, 0.5)
};

void setup()
{
  Serial.begin(115200); // Main USB telemetry

  // Initialize Hardware Serials
  Serial1.begin(9600, SERIAL_8N1, HW_RX_0, HW_TX_0);
  Serial2.begin(9600, SERIAL_8N1, HW_RX_1, HW_TX_1);

  // Initialize Software Serials
  swSensors[0].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, SW_RX_2, SW_TX_2, false, 64);
  swSensors[1].begin(9600, EspSoftwareSerial::SWSERIAL_8N1, SW_RX_3, SW_TX_3, false, 64);

  delay(1000); // Allow sensors to boot up stably after power-on
}

void loop()
{
  // Step through each sensor sequentially to prevent acoustic cross-talk
  for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++)
  {
    // 1. Purge any stray data from the target serial buffer before triggering
    clearSerialBuffer(i);

    // 2. Transmit the 0x55 trigger command to the specific sensor
    triggerSensor(i);

    // 3. Wait for the 4-byte response packet (Max 40ms window)
    unsigned long startTime = millis();
    unsigned long timeSpentReading = 0;

    while ((timeSpentReading = (millis() - startTime)) < 40)
    {
      if (isDataAvailable(i))
      {
        // Peek to ensure we align with the header byte
        if (peekSerialByte(i) == FIRST_BYTE)
        {
          delay(5); // Small padding window for remaining 3 bytes to arrive over 9600 baud
          if (getAvailableBytes(i) >= 4)
          {
            parseSensorPacket(i);
            break; // <-- Exit the while loop early
          }
        }
        else
        {
          readSerialByte(i); // Drop misaligned/garbage byte
        }
      }
      yield();
    }

    // 5. Jitter-free pacing calculation
    unsigned long detectionWindowRemainder = 40 - timeSpentReading;
    delay(100 + detectionWindowRemainder);
  }

  produceTone();
  Serial.printf("S0 (HW): %dmm   S1 (HW): %dmm   S2 (SW): %dmm   S3 (SW): %dmm\n",
                latestSensorsDistance[0], latestSensorsDistance[1],
                latestSensorsDistance[2], latestSensorsDistance[3]);
}

// Low-level abstraction helpers to seamlessly mix HW and SW serial objects
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

// Packet verification logic
void parseSensorPacket(uint8_t id)
{
  readSerialByte(id); // Strip and drop verified FIRST_BYTE (0xFF)

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