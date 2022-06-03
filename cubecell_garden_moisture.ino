#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "RunningMedian.h"
#include "Seeed_BME280.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <simple_logger.h>
#include "credentials.h"

#define WAKE_UP_PIN USER_KEY

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
/**
   30  sec =   30000
    1  min =   60000
    2  min =  120000
    5  min =  300000
    10 min =  600000
    15 min =  900000
    20 min = 1200000
*/

uint32_t appTxDutyCycle = 1200000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

#define BME_280_READINGS 5

char buffer[40];

/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

BME280 bme280;

int moistureSensorPin = ADC3;

int oneWirePin = GPIO5;
OneWire oneWire(oneWirePin);
DallasTemperature sensors(&oneWire);

unsigned int batteryVoltage, uptimeCount, moisture, humidity;

float temperature, dallasTemperature, pressure;

boolean accelWoke;

RunningMedian moistureReadings = RunningMedian(5);

#define DEFAULT_LOG_LEVEL logger::None // DEBUG: set the Debug for more logging statements

void prepareBeforeSleep() {
  Wire.end();
  logger::set_level(DEFAULT_LOG_LEVEL);
  digitalWrite(Vext, HIGH);
  delay(100);
}

void onWakeUp() {
  delay(10);
  if (digitalRead(WAKE_UP_PIN) == HIGH && deviceState == DEVICE_STATE_SLEEP) {
    Serial.println(F("Woke up by WAKE_UP_PIN during sleep"));
    accelWoke = true;
    delay(50);
  }
}

static void prepareTxFrame( uint8_t port ) {
  logger::debug("Uptime Counter:  %d", uptimeCount);

  detachInterrupt(WAKE_UP_PIN);
  batteryVoltage = getBatteryVoltage();
  logger::debug("batteryVoltage(mV): %d", batteryVoltage);
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

  if (digitalRead(Vext) == HIGH) {
    digitalWrite(Vext, LOW);
    delay(500); // let the power line settle
  }

  bme280.init();
  delay(50);
  logger::debug("Start to measure BME280");
  for (int x = 1; x <= BME_280_READINGS; x++) {
    temperature = bme280.getTemperature();
    pressure = bme280.getPressure();
    humidity = bme280.getHumidity();
    delay(50);
  }
  logger::debug("temp(°C):  %d", (int) temperature);
  logger::debug("humidity(%): %d", humidity);
  logger::debug("pressure(hPa):  %d", (int) (pressure / 100));

  logger::debug("Start to measure Dallas Temperature");
  sensors.begin();
  sensors.requestTemperatures();
  dallasTemperature = sensors.getTempCByIndex(0);
  logger::debug("Outside temp(°C):  %d", (int) dallasTemperature);

  logger::debug("Start to measure moisture");
  while (!moistureReadings.isFull()) {
    moistureReadings.add(map(analogRead(moistureSensorPin), 0, 4096, 0, 100));
    delay(10);
  }
  moisture = moistureReadings.getMedian();
  moistureReadings.clear();
  logger::debug("moisture(%): %d", moisture);

  appDataSize = 14;

  appData[0] = highByte(uptimeCount);
  appData[1] = lowByte(uptimeCount);

  appData[2] = highByte(batteryVoltage);
  appData[3] = lowByte(batteryVoltage);

  int temp = temperature * 100;
  appData[4] = highByte(temp);
  appData[5] = lowByte(temp);

  temp = dallasTemperature * 100;
  appData[6] = highByte(temp);
  appData[7] = lowByte(temp);

  int press = pressure;
  appData[8] = press >> 24;
  appData[9] = press >> 16;
  appData[10] = press >> 8;
  appData[11] = press & 0xFF;

  appData[12] = lowByte(humidity);

  appData[13] = lowByte(moisture);

  uptimeCount++;
}

void setup() {
  Serial.begin(115200);
  logger::set_serial(Serial);
  logger::set_level(logger::Debug);

  accelWoke = false;
  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);

#if(AT_SUPPORT)
  enableAt();
#endif

  pinMode(Vext, OUTPUT);

  deviceState = DEVICE_STATE_INIT;

  LoRaWAN.ifskipjoin();
}

void loop() {
  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
#if(LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
        getDevParam();
#endif
        printDevParam();
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame( appPort );
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        prepareBeforeSleep();
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (accelWoke) {
          if (IsLoRaMacNetworkJoined) {
            logger::set_level(logger::Debug);
            prepareTxFrame( appPort );
            LoRaWAN.send();
            prepareBeforeSleep();
          }
          accelWoke = false;
        }
        LoRaWAN.sleep();
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  Serial.printf("+REV DATA: %s, RXSIZE: %d, PORT: %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA: ");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  Serial.println();
  if (mcpsIndication->Port == 4) {
    int newSleepTime = mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8);
    appTxDutyCycle  = newSleepTime * 1000;
    saveDr();
    Serial.print(F("new DutyCycle received: "));
    Serial.print(appTxDutyCycle);
    Serial.println(F("ms"));
  }
  delay(50);
}
