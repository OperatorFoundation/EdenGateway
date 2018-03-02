// Beyonce_Bling.pde
// -*- mode: C++ -*-

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiMDNSResponder.h>
#include "Secrets.h"

#define SATELLITE_CAPABLE

#ifdef SATELLITE_CAPABLE
  #include "Serial2.h"
  #include <IridiumSBD.h>

  int signalQuality = -1;  

  IridiumSBD sat(Serial2, A3);  

  #define LIPO_POWER 0
  #define USB_POWER 1
#endif

//#define APMODE
#define TESTING

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;

unsigned int localPort = 2390;      // local port to listen on
char mdnsName[] = "gateway"; // the MDNS name that the board will respond to
char apSSID[] = "EdenGateway";

WiFiMDNSResponder mdnsResponder;
WiFiServer server(localPort);
WiFiClient client;

const int networkWriteBufferMaxLength=1440;
const int networkReadBufferMaxLength=1440;

uint8_t networkWriteBuffer[networkWriteBufferMaxLength];
uint8_t networkReadBuffer[networkReadBufferMaxLength];

size_t networkReadBufferLength=0;
size_t networkWriteBufferLength=0;
size_t networkReadNextMessageLength=0;

enum MessageType {
  announce,
  ping,
  pong,
  message,
  gate,
  repeat
};

struct Node {
  bool seen;
  uint8_t flags;
  char *tac;
  char *call;
};

enum class Flags: uint8_t {
  Repeater  = 0b00000001,
  Internet  = 0b00000010,
  Satellite = 0b00000100,
  Cellular  = 0b00001000,
  User      = 0b00010000,
  Storage   = 0b00100000,
  Voice     = 0b01000000,
  Beacon    = 0b10000000
};

Node peers[255];

void initRandom()
{
  int seed=0;
  int x;

  char *date=__DATE__;
  for(x=0; x<11; x++)
  {
    seed = seed ^ (int)date[x];
  }

  char *timestr=__TIME__;
  for(x=0; x<8; x++)
  {
    seed = seed ^ (int)timestr[x];
  }  

  seed = seed ^ analogRead(A0);

  randomSeed(seed);
}

uint8_t newShortID()
{
  randomSeed(analogRead(A0));   
  return random(2, 254);  
}

uint8_t networkID=0;
uint8_t shortID=0;
const char *tac="BEACON";
const char *call="KF5WVW";

#ifdef REPEATER
  uint8_t flags=(uint8_t)Flags::Repeater | (uint8_t)Flags::Beacon;
#else
  uint8_t flags=(uint8_t)Flags::User;
#endif

uint8_t friendID=0;

bool satReady=false;

unsigned long lastPacketTime=millis();
unsigned int satWaiting=0;

void setup()
{
  initRandom();
  shortID=newShortID();
  
  initSerial();
  initSatellite();

  initWifi();
  initServer();  
  initmDNS();
}

/* Initializers */

void initSerial() {
  Serial.begin(9600);
  
  unsigned long startTime=millis();
  while(!Serial && (startTime+(10*1000) > millis())) {}

  Serial.println("Hello, Operator");
  Serial.println("SatelliteTest");  
}

void initSatellite() {
#ifdef SATELLITE_CAPABLE
  Serial2.begin(19200);  

  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);

  sat.attachConsole(Serial);
  sat.attachDiags(Serial);

  // High-current, meaning powered by a LiPo (not USB)
  sat.setPowerProfile(LIPO_POWER);

  if(sat.begin()==ISBD_SUCCESS)
  {
    Serial.println("satellite found");
    int err = sat.getSignalQuality(signalQuality);
    if (err==ISBD_SUCCESS)
    {
      satReady=true;
      Serial.println("satellite enabled");
      Serial.print("Signal quality is ");
      Serial.println(signalQuality);  

      sat.sleep();
    }
    else
    {
      Serial.print("SignalQuality failed: error ");
      Serial.println(err);
      Serial.println("satellite disabled");
    }    
  } else {
    Serial.println("satellite initialization failed");
  }
#endif
}

void initWifi() {
  WiFi.setPins(8,7,4,2);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED && status != WL_AP_LISTENING ) {    
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    #ifdef APMODE
      Serial.print("Creating access point named: ");
      Serial.println(apSSID);
      // by default the local IP address of will be 192.168.1.1
      // you can override it with the following:
      // WiFi.config(IPAddress(10, 0, 0, 1));
      status = WiFi.beginAP(apSSID);
      if (status != WL_AP_LISTENING) {
        Serial.println("Connecting to WiFi network failed");
        // don't continue
        while (true);
      }
    #else
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);    
      status = WiFi.begin(ssid, pass);
      if (status != WL_CONNECTED) {
        Serial.println("Connecting to WiFi network failed");
        // don't continue
        while (true);
      }
    #endif

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to wifi");
  printWiFiStatus();
}

void initmDNS() {
  // Setup the MDNS responder to listen to the configured name.
  // NOTE: You _must_ call this _after_ connecting to the WiFi network and
  // being assigned an IP address.
  if (!mdnsResponder.begin(mdnsName)) {
    Serial.println("Failed to start MDNS responder!");
    while(1);
  }

  Serial.print("Server listening at tcp://");
  Serial.print(mdnsName);
  Serial.print(".local:");
  Serial.println(localPort);
}

void initServer() {
  server.begin();  
}

void loop()
{  
  handlemDNS();
  handleWifi();

  #ifdef AP_MODE
    if(status == WL_AP_CONNECTED) {
      handleServer();    
    }
  #else
    if(status == WL_CONNECTED) {
      handleServer();    
    }
  #endif
}

/* Handlers */

void handlemDNS() {
  // Call the update() function on the MDNS responder every loop iteration to
  // make sure it can detect and respond to name requests.
  mdnsResponder.poll();  
}

void handleWifi() {
  #ifdef APMODE
    if (status != WiFi.status()) {
      // it has changed update the variable
      status = WiFi.status();
  
      if (status == WL_AP_CONNECTED) {
        byte remoteMac[6];
  
        // a device has connected to the AP
        Serial.print("Device connected to AP, MAC address: ");
        WiFi.APClientMacAddress(remoteMac);
        Serial.print(remoteMac[5], HEX);
        Serial.print(":");
        Serial.print(remoteMac[4], HEX);
        Serial.print(":");
        Serial.print(remoteMac[3], HEX);
        Serial.print(":");
        Serial.print(remoteMac[2], HEX);
        Serial.print(":");
        Serial.print(remoteMac[1], HEX);
        Serial.print(":");
        Serial.println(remoteMac[0], HEX);
      } else {
        // a device has disconnected from the AP, and we are back in listening mode
        Serial.println("Device disconnected from AP");
      }
    }
  #endif  
}

void handleServer() {
    // listen for incoming clients
  client = server.available();
  
  if(client) {
    Serial.println("new client");      
    sat.begin();
    
    while(client.connected()) {
      handleReceiveNetworkMessage();
      handleSatellite();
    }

    Serial.println("client disconnected");        
    sat.sleep();
  }  
}

void handleReceiveNetworkMessage() {  
  int b;
  
  if(networkReadBufferLength > 0) {
    return;
  }
  
  // if there's data available, read a message
  if(client.available()) {
    b = client.read();
    if(b > 0 && b <= networkReadBufferMaxLength) {
      networkReadNextMessageLength=b;
    }
  }

  for(int x=0; x<networkReadNextMessageLength; x++) {
    while(!client.available()) {
      delay(10);
    }
    
    b=client.read();
    if(b != -1) {
      networkReadBuffer[networkReadBufferLength++] = b;
    }
  }
  
  if(networkWriteBufferLength > 0) {
    client.write(networkWriteBufferLength);
    
    for(int x=0; x<networkWriteBufferLength; x++) {
      client.write(networkWriteBuffer[x]);
    }    

    networkWriteBufferLength=0;
  }
}

void handleSatellite() {
  if(networkReadBufferLength > 0) {
    sendSatelliteMessage();
    networkReadBufferLength=0;
  }

  if(satWaiting==0 && networkReadBufferLength==0 && lastPacketTime > 10*60*1000) { // 10 minutes
    Serial.println("Periodically checking for new messages...");
    satWaiting=sat.getWaitingMessageCount();  
  }

  if(satWaiting>0) {
    Serial.print("Messages waiting: ");
    Serial.println(satWaiting);          

    if(networkReadBufferLength==0) {
      sendSatelliteMessage();
    }
  }
}

/* Callbacks */

#ifdef SATELLITE_CAPABLE
bool ISBDCallback()
{
   bool status = (bool)((millis() / 1000) % 2);
   if(status)
   {
//     Serial.print("^");
   }
   
   return true;
}
#endif

/* Utilities */

void(* doReset) (void) = 0; 

// A small helper
void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.print("Port: ");
  Serial.println(localPort);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void sendSatelliteMessage() {
  lastPacketTime=millis();
  
  Serial.print("Sending message, ");
  Serial.print(networkReadBufferLength);
  Serial.println(" bytes");

  const uint8_t *satWriteBuffer=(const uint8_t *)networkReadBuffer;
  size_t satWriteLength=(size_t)networkReadBufferLength;
  uint8_t *satReadBuffer=(uint8_t *)networkWriteBuffer;
  size_t satReadLength=sizeof(satReadBuffer);

  int err = sat.sendReceiveSBDBinary(satWriteBuffer, satWriteLength, satReadBuffer, satReadLength);
  if (err != 0) {
    Serial.print("sendSBDBinary failed: error ");
    Serial.println(err);
    return;
  }

  Serial.println("Hey, it worked!");
  Serial.print("Messages left: ");
  satWaiting=sat.getWaitingMessageCount();  
  Serial.println(satWaiting);  
}

