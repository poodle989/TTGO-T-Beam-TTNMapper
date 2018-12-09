#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>

#define TXPin 12
#define RXPin 15
#define LEDPin 21

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static PROGMEM u1_t NWKSKEY[16] = { 0xF2, 0x2B, 0xC1, 0xCB, 0x10, 0xE1, 0x5A, 0x64, 0x61, 0x7F, 0xED, 0xE0, 0xE9, 0x4B, 0x51, 0x03 };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static u1_t PROGMEM APPSKEY[16] = { 0xCB, 0x32, 0x93, 0xB1, 0x4B, 0xCD, 0x54, 0x8F, 0x11, 0x6B, 0x13, 0xD3, 0xF8, 0x24, 0x60, 0x95 };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26041220 ; // <-- Change this address for every node!

// GPS variables
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint8_t txBuffer[9];

char s[32]; // used to sprintf for Serial output
char t[32]; // used to sprintf for Serial output

static osjob_t sendjob;

const unsigned TX_INTERVAL = 30; // TTN Fair use

TinyGPSPlus gps;
HardwareSerial GPSSerial(2);

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {26, 33, 32},
};

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void InitializeLMIC()
{
  os_init();

  LMIC_reset();

  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly 
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  LMIC_selectSubBand(1);

  LMIC_setLinkCheckMode(0);

  LMIC.dn2Dr = DR_SF9;

  LMIC_setDrTxpow(DR_SF7, 14);

  for (int channel=0; channel<8; ++channel) {
    LMIC_disableChannel(channel);
  }
  for (int channel=9; channel<72; ++channel) {
     LMIC_disableChannel(channel);
  }
}

void setup()
{
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, TXPin, RXPin);

  WiFi.mode(WIFI_OFF);
  btStop();
  
  InitializeLMIC();

  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);
  
  do_send(&sendjob);
}

void loop()
{
  os_runloop_once();
}

void onEvent (ev_t ev) 
{
  switch (ev) 
  {
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
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j)
{ 
  if(LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println("Not Sending");
  }
  else
  {
    if(CheckGPSFix())
    {
      BuildPacket(txBuffer);
      LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
      Serial.println(F("Packet queued"));
      digitalWrite(LEDPin, HIGH);  
      Serial.println(LMIC.freq);
    }
    else
    {
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
    }
  }
}

bool CheckGPSFix()
{
  Encode();
  if(gps.location.isValid()
      && gps.hdop.isValid() 
      && gps.altitude.isValid())
  {
    Serial.println("Valid GPS Fix");
    return true;
  }
  else
  {
    Serial.println("No GPS Fix");
    return false;
  }
}

void Encode()
{
  int data;
  int previousMillis = millis();

  while((previousMillis + 1000) > millis())
  {
    while(GPSSerial.available())
    {
      char data = GPSSerial.read();
      gps.encode(data);
    }
  }
}

void BuildPacket(uint8_t txBuffer[9])
{
  LatitudeBinary = ((gps.location.lat() + 90) / 180.0) * 16777215;
  LongitudeBinary = ((gps.location.lng() + 180) / 360.0) * 16777215;
  
  sprintf(t, "Lat: %f", gps.location.lat());
  Serial.println(t);
  
  sprintf(t, "Lng: %f", gps.location.lng());
  Serial.println(t);
  
  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = gps.altitude.meters();
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = gps.hdop.value()/10;
  txBuffer[8] = hdopGps & 0xFF;
}
