f//used Libraries
//======================================================================================================
#include <Ultrasonic.h>   //distance sensor (Ultrasonic distance sensor) 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
//http://www.netzmafia.de/skripten/hardware/Arduino/Sleep/index.html
//https://www.youtube.com/watch?v=usKaGRzwIMI

//defined Pins
//======================================================================================================
#define TRIG_PIN 12      //ultrasonic sensor trigger pin to arduino pin 5   
#define ECHO_PIN 13      //ultrasonic sensor echo    pin to arduino pin 6
#define BAUDRATE 9600   //for debugging and testing

//defined stuff
#define park_time_threshold 30  //threshold for parking seconds
#define park_out_time_threshold 15  //threshold for parking seconds // has to be lower then park_time_threshold due to logik

#define distance_threshold 50   //threshold for praking distance 

#define timer1_preload 49909    //for 1s
#define timer2_preload 99    //for 10ms
//#define timer2_softload 100    //for 1s //10ms*100 = 1s
//#define sleep_time 60         //upcoming feature

//variables
//======================================================================================================
int distance_cm;

bool park_flag = false;
bool park_status = false;
int park_time = 0;

//bool sleep_flag=false;

//int timer2_softcount=0;
//int timer2_softcount_sec=0;

//LoRa

//=====================================================================================================================================================================================
// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xE7, 0x5C, 0xAA, 0x0A, 0x99, 0xAA, 0x86, 0x5C, 0xDD, 0x14, 0x0D, 0x13, 0xCA, 0x19, 0x83, 0xFF };

// LoRaWAN AppSKey, application session key
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x86, 0x6C, 0x79, 0x98, 0xDA, 0x7A, 0x32, 0x54, 0xE7, 0x83, 0xA6, 0x57, 0x1D, 0x38, 0xEB, 0x15 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011D0B ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "hi";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};


//objects
//======================================================================================================
Ultrasonic parkingsensor(TRIG_PIN, ECHO_PIN);


//Timer1 magic config :P
//for more info check the datasheet http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
//======================================================================================================
void timer1_setup() {
  TCCR1A = 0x00;
  TCCR1B = 0x00;

  //Normal Mode //Prescaler is set to 1024
  TCCR1B |= (1 << CS12);
  TCCR1B |= (1 << CS10);

  //Interrupt enable
  TIMSK1 = 0x00;
  TIMSK1 |= (1 << TOIE1); //enable timer overflow interrupt

  TCNT1 = timer1_preload;  //load value for timer to hit one second precicly   //startvalue of the counter
}


//LoRa functions
//======================================================================================================
void onEvent (ev_t ev) {
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
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
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


void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


//main
//======================================================================================================
void setup() {
  timer1_setup();         //timer1 setup
  //timer2_setup();         //timer2 setup
  Serial.begin(9600);     //for debugging and testing

  //LoRa
  Serial.begin(9600);
  Serial.println(F("Starting"));

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
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

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);
}

//while(1)
//======================================================================================================
void loop() {

  os_runloop_once();

  parkingsensor.measure();      //reads distance to car in cm
  distance_cm = parkingsensor.get_cm();

  if ((distance_cm <= distance_threshold) && (!park_status)) {  //if the distance is under 50cm and the park flag is not set then it starts checking if the car is parking or not.
    park_flag = true;
  }
  if ((distance_cm > distance_threshold) && park_status) { //if the distance is bigger then the threshold it should reset the flag and status
    park_flag = false;
  }

  if (park_status == park_flag) {
    park_time = 0;
  }
  delay(500);
  Serial.print("park_threshcheck: ");
  Serial.print(park_time);
  Serial.print("     ");
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.print("     ");
  Serial.print("Millis: ");
  Serial.println(millis());
}


//======================================================================================================
ISR(TIMER1_OVF_vect) {  //every sec...

  TCNT1 = timer1_preload; //load value for timer to hit one second precicly


  if ((park_time <= park_time_threshold) && park_flag) { //start counting if the slot is taken
    if (distance_cm < distance_threshold) {
      park_time++;
    }
    else {
      park_time = 0;
    }
  }

  if ((park_time == (park_time_threshold)) && park_flag) { //updates parkstatus
    static uint8_t mydata[] = "p1";
    static osjob_t sendjob;
    park_status = true;
    park_time = 0;

    Serial.println("                      Jetzt ist eingeparkt!!!!!!!");
  }

  if ((park_time <= park_out_time_threshold) && (!park_flag)) { //start countig if slot is freed
    if (distance_cm > distance_threshold) {
      park_time++;
    }
    else {
      park_time = 0;
    }
  }

  if ((park_time == (park_out_time_threshold)) && (!park_flag)) { //updates parkstatus
    static uint8_t mydata[] = "p0";
    static osjob_t sendjob;
    park_status = false;
    park_time = 0;

    Serial.println("                      Jetzt ist ausgeparkt!!!!!!!");
  }
}
