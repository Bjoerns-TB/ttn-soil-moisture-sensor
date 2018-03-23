/***************************************************
 This example reads Capacitive Soil Moisture Sensor.
 
 Created 2015-10-21
 By berinie Chen <bernie.chen@dfrobot.com>
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

 /***************************************************
 Sketch was modified to work with LoRaWAN and
 TheThingsNetwork
 
 Created 2018-03.23
 By Björn Amann
 ****************************************************/
 
/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here: https://www.dfrobot.com/wiki/index.php?title=Capacitive_Soil_Moisture_Sensor_SKU:SEN0193
 2.This code is tested on Arduino Pro Mini.
 3.Sensor is connect to Analog 0 port.
 ****************************************************/
 
  * ------------------------------------------------------------------------
 * Software used : 
 *  - LMIC https://github.com/matthijskooijman/arduino-lmic 
 *  - LowPower library https://github.com/rocketscream/Low-Power
 *  
 * For licenses of the used libraries, check the links above.
 * ------------------------------------------------------------------------ 

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h> 
#include "LowPower.h"

bool next = false;


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 904; //multiple of 8

// Pin mapping CH2I (check out : https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-and-1-3ua-sleep-mode/8059 ) 
#define LMIC_NSS    10
#define LMIC_RXTX   LMIC_UNUSED_PIN
#define LMIC_RST    LMIC_UNUSED_PIN
#define LMIC_DIO0   2
#define LMIC_DIO1   7



const lmic_pinmap lmic_pins = {
    .nss = LMIC_NSS,
    .rxtx = LMIC_RXTX,   
    .rst = LMIC_RST,
    .dio = {LMIC_DIO0, LMIC_DIO1},  
}; 

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x09, 0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x25, 0x99, 0x7B, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0xA9, 0x43, 0x65, 0x98, 0x12, 0xDA, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
 
static osjob_t sendjob; 

//Calibration Values fpr Soil Sensor
const int AirValue = 867;   //you need to replace this value
const int WaterValue = 490;  //you need to replace this value
int intervals = (AirValue - WaterValue)/3;   
int soilMoistureValue = 0;
int BattValue = 0;

//Definition of Pins
int SoilOut = A0; //Soil Moisture Sensor INPUT
int BattOut = A1; //Voltage Divider INPUT
int SoilVCC = 9; //Soil Moisture Sensor OUTPUT

void onEvent (ev_t ev) 
{
    Print(os_getTime());
    Print(": ");
    PrintLn(ev);
    switch(ev) 
    {
        case EV_SCAN_TIMEOUT:
            //PrintLn(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            //PrintLn(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            //PrintLn(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            //PrintLn(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            //PrintLn(F("EV_JOINING"));
            break;
        case EV_JOINED:
            //PrintLn(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            //PrintLn(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            //PrintLn(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            //PrintLn(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            PrintLn(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK)
              PrintLn(F("R ACK")); // Received ack
            if (LMIC.dataLen) 
            {
              PrintLn(F("R "));
              PrintLn(LMIC.dataLen);
              PrintLn(F(" bytes")); // of payload
            }            
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            next = true; 
            break;
        case EV_LOST_TSYNC:
            //PrintLn(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            //PrintLn(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            //PrintLn(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            //PrintLn(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            //PrintLn(F("EV_LINK_ALIVE"));
            break;
         default:
            //PrintLn(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        PrintLn(F("OP_TXRXPEND")); //P_TXRXPEND, not sending
    } 
    else 
    {

  digitalWrite(SoilVCC, HIGH);       // sets the digital pin 13 on
  delay(1000);                  // waits for a second
  
  soilMoistureValue = averageAnalogRead(SoilOut);  //put Sensor insert into soil

  digitalWrite(SoilVCC, LOW);

if(soilMoistureValue > WaterValue && soilMoistureValue < (WaterValue + intervals))
{
  Serial.println("Very Wet");
  Serial.println(soilMoistureValue);
}
else if(soilMoistureValue > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals))
{
  Serial.println("Wet");
  Serial.println(soilMoistureValue);
}
else if(soilMoistureValue < AirValue && soilMoistureValue > (AirValue - intervals))
{
  Serial.println("Dry");
  Serial.println(soilMoistureValue);
}
 
BattValue = averageAnalogRead(BattOut);
int battVoltage = ( 3.36 / 1024 * BattValue * 2 * 100 ) - 250;
Serial.println(BattValue);
Serial.println(battVoltage);

     
        unsigned char mydata[3];
        mydata[0] = battVoltage;     
        mydata[1] = soilMoistureValue >> 8;
        mydata[2] = soilMoistureValue & 0xFF;

        
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        PrintLn(F("PQ")); //Packet queued
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

void setup() {

  pinMode(SoilOut, INPUT);
  pinMode(BattOut, INPUT);
  pinMode(SoilVCC, OUTPUT);
  
  Serial.begin(9600); // open serial port, set the baud rate to 9600 bps
      // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    do_send(&sendjob);
}



void loop() 
{ 
  extern volatile unsigned long timer0_overflow_count;

  if (next == false) {

    os_runloop_once();

  } else {

    int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
    Serial.flush(); // give the serial print chance to complete
    for (int i=0; i<sleepcycles; i++) {
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

      // LMIC uses micros() to keep track of the duty cycle, so
      // hack timer0_overflow for a rude adjustment:
      cli();
      timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
      sei();
    }
    next = false;
    // Start job
    do_send(&sendjob);
  }
}
