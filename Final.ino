#include <arduino_lmic.h>
#include <arduino_lmic_hal_boards.h>
#include <arduino_lmic_hal_configuration.h>
#include <arduino_lmic_lorawan_compliance.h>
#include <arduino_lmic_user_configuration.h>
#include <lmic.h>



#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SparkFun_VEML6030_Ambient_Light_Sensor.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif


static const PROGMEM u1_t NWKSKEY[16] = { 0xE0, 0xA6, 0x9E, 0xE7, 0x0C, 0x5D, 0xE9, 0x78, 0xE3, 0x92, 0x6D, 0xA4, 0x47, 0xAA, 0x6C, 0xE8 };


static const u1_t PROGMEM APPSKEY[16] = { 0x74, 0xC7, 0x03, 0x25, 0x11, 0xDC, 0x7F, 0x39, 0x81, 0xBF, 0xBB, 0x20, 0x79, 0xF6, 0x43, 0xA8 };


static const u4_t DEVADDR = 0x260DF342;

// Using Adafruit library for creating BME280 
Adafruit_BME280 bme;
SparkFun_Ambient_Light veml6030(0x48); // Pass the I2C address of VEML6030

//Sensor pin connection with Arduino
int sensorPin = A0;
int sensorValue = 0;
int dryValue = 515;   // Replace with your dry calibration value
int wetValue = 265;   // Replace with your wet calibration value

//Variable to compute time for next reading, Initially send to 60 seconds
unsigned long lastSendTime = 0; // Variable to store the last time we sent data
const unsigned long sendInterval = 60000; // Interval at which to send data (1 minute)

void os_getArtEui(u1_t* buf) {}
void os_getDevEui(u1_t* buf) {}
void os_getDevKey(u1_t* buf) {}

//Variable for seding "no Sensor Data message" to Things Network when no sensor is available
static uint8_t mydata[] = "No Sensor Data";
byte downlink_active = 0;
byte newdata[1];
static osjob_t sendjob;


const unsigned TX_INTERVAL = 60;


const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = { 2, 6, 7 },
};

int RX_LED = 12;

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;

    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload: "));
        for (int i = 0; i < LMIC.dataLen; i++) {
          if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
          }
          Serial.println(LMIC.frame[LMIC.dataBeg + i], HEX);
          uint8_t byte_recieved = LMIC.frame[LMIC.dataBeg + i];
          if (byte_recieved == 208 && i == 0) {
            Serial.println(F("Making LED blink once."));
            digitalWrite(RX_LED, HIGH);
            delay(1000);
            digitalWrite(RX_LED, LOW);
          } else if (byte_recieved == 209 && i == 0) {
            Serial.println(F("Making LED blink twice."));
            digitalWrite(RX_LED, HIGH);
            delay(1000);
            digitalWrite(RX_LED, LOW);
            delay(1000);
            digitalWrite(RX_LED, HIGH);
            delay(1000);
            digitalWrite(RX_LED, LOW);
          }
          if (i < 1)
            newdata[i] = byte_recieved + 1;
          downlink_active = 1;
        }
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;

    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    if (downlink_active == 0)
      LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    else
      LMIC_setTxData2(1, newdata, sizeof(newdata), 0);
    Serial.print(F("Packet queued for frequency (Hz): "));
    Serial.println(LMIC.freq);
  }
}

void setup() {
  //    pinMode(13, OUTPUT);
  while (!Serial)
    ;  // wait for Serial to be initialized
  Serial.begin(9600);
  delay(100);  // per sample code on RF_95 test
  Serial.println(F("Starting"));
  Serial.println("BME280 test");
  Wire.begin(); // Join the I2C bus

  // Initialize the BME280 sensor
  if (!bme.begin(0x77)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  if (veml6030.begin(Wire)) { // Check if the sensor is present
    Serial.println("Sensor found");
  } else {
    Serial.println("Sensor not found");
    while (1); // Stop execution if sensor is not found
  }

  // Set gain and integration time as needed
  veml6030.setGain(1); 
  veml6030.setIntegTime(4);

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif


  os_init();

  LMIC_reset();


#ifdef PROGMEM

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
#else

  LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);  // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);   // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);    // g2-band

#elif defined(CFG_us915) || defined(CFG_au915)

  LMIC_selectSubBand(1);
#elif defined(CFG_as923)

#elif defined(CFG_kr920)

#elif defined(CFG_in866)



#else
#error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);


  LMIC_setDrTxpow(DR_SF7, 14);


  do_send(&sendjob);
}

void loop() {
  unsigned long currentMillis = millis(); // Grab the current time
  
  // Check if one minute has passed; note this will also work if millis() wraps
  if (currentMillis - lastSendTime >= sendInterval) { 
    lastSendTime = currentMillis; // Save the last time we sent data
    
    // Check if there is not a current TX/RX job running
    if (!(LMIC.opmode & OP_TXRXPEND)) {
      String sensorReadings = getSensorReadings(); // Call the function to read senosr data and store in String
      
      // Convert string to char array to be compatible with LoRa send function
      int str_len = sensorReadings.length() + 1; 
      char payload[str_len];
      sensorReadings.toCharArray(payload, str_len);
      
      // Prepare upstream data transmission
      LMIC_setTxData2(1, (uint8_t *)payload, sizeof(payload)-1, 0);
      Serial.print(F("Sending data to TTN: "));
      Serial.println(payload);
    } else {
      Serial.println(F("OP_TXRXPEND, not sending"));
    }
  }
  
  os_runloop_once();  // Run the LMIC event loop
}
String getSensorReadings() {
  // Read values from BME280 sensor
  float temp = bme.readTemperature(); // Read temperature
  float pressure = bme.readPressure() / 100.0F; // Read pressure
  float humidity = bme.readHumidity(); // Read humidity
  
  // Read values from VEML6030 Ambient Light Sensor
  uint16_t rawLight = veml6030.readLight(); // Read raw ambient light value
  float lux = convertToLux(rawLight); // Convert raw value to Lux
  
  // Read values from soil moisture sensor
  int sensorValue = analogRead(sensorPin); // Read analog value from moisture sensor
  int moisturePercentage = map(sensorValue, wetValue, dryValue, 100, 0); // Map value to range 0-100
  moisturePercentage = constrain(moisturePercentage, 0, 100); // Constrain value to range 0-100
  
  // Concatenate all values into a single string with comma separation
  String sensorReadings = "Temp:"+String(temp) + ",pressure:" + String(pressure) + ",humidity" + String(humidity) + ",lux:" + String(lux*100) + ",soil:" + String(moisturePercentage);
  
  return sensorReadings;
}

float convertToLux(uint16_t raw) {
  float gain = 1.0; // The gain you set
  float integrationTime = 800.0; // The integration time you set in milliseconds
  return raw * (1.0 / gain) * (1.0 / integrationTime);
}	\
