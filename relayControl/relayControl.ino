//**************************************************************//
//  Name    : relayControl,                                     //
//  Author  : Dimitrios Politis                                 //
//  Date    : 4 May, 2020                                       //
//  Version : 1.0                                               //
//  Notes   : Code for controlling 16 relay board,              //
//          : using two 74HC595 Shift Registers,                //
//          : through serial or udp messages.                   //
//                                                              //
//          : Control messages are:                             //
//          :                                                   //
//          : ON=pin                                            //
//          : OFF=pin                                           //
//          : CHK=pin                                           //
//****************************************************************

/*** Start of relay control definitions ***/
#define EEPROM_LAYOUT_VERSION 0
#define AMOUNT_OF_INDEXES 1
#define INDEX_CONFIGURATION_pinData 0
/*** End of relay control definitions ***/

/*** Start of ethernet shield definitions ***/
// Size of buffer to hold incoming udp packet.
// Udp Command is given in format ON=2, OFF=13, OFF=ALL
// so maxChar=7
#define PACKET_BUFFER_SIZE 7
/*** End of ethernet shield definitions ***/

/*** Start of relay control includes ***/
#include <EEPROMWearLevel.h>
#include <SerialConfigCommand.h>
/*** End of relay control includes ***/

/*** Start of ethernet shield includes ***/
#include <Ethernet.h>
#include <EthernetUdp.h>
/*** End of ethernet shield includes ***/

/*** Start of relay control variables ***/
// Pin connected to SRCLK/SH_CP of 74HC595
int clockPin = 3;
// Pin connected to RCLK/ST_CP of 74HC595
int latchPin = 5;
// Pin connected to OE of 74HC595
int enableOutputPin = 6;
// Pin connected to SER/DS of 74HC595
int dataPin = 7;
// Pin connected to SRCLR/MR of 74HC595
int clearPin = 8;

// Should we Update EEPROM with current setting?
boolean writeEEPROM = true;

// Holders for information you're going to pass to shifting function
int pin;
int pinStatus;
// Pin to actual relay wiring
int pinIndex[16] = {0, 15, 1, 14, 2, 13, 3, 12, 4, 11, 5, 10, 6, 9, 7, 8};
word pinData;

// Define an instance of SerialConfigCommand
SerialConfigCommand scc;
/*** End of relay control variables ***/

/*** Start of ethernet shield variables ***/
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0x00, 0x02, 0x3D, 0x89, 0x40, 0x94};
IPAddress ip(192, 168, 78, 180);

unsigned int localPort = 4444; // Local port to listen on

// Buffers for receiving and sending data
char packetBuffer[PACKET_BUFFER_SIZE]; // Buffer to hold incoming packet
char ReplyERR[] = "ERR"; // An error string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
/*** End of ethernet shield variables ***/

void setup() {
  /*** Start of relay control setup ***/
  // Set pins to output because they are addressed in the loop exec
  // Disable all outputs on powerOn, until outputs settle
  pinMode(enableOutputPin, OUTPUT);
  digitalWrite(enableOutputPin, HIGH);

  // Disable latch pin on powerOn, to disable any input to shift register
  pinMode(latchPin, OUTPUT);
  digitalWrite(latchPin, LOW);

  // Clear spurious output on powerOn, as we use values from EEPROM
  pinMode(clearPin, OUTPUT);
  digitalWrite(clearPin, LOW);
  // Enable shift register normal operation
  digitalWrite(clearPin, HIGH);

  // Clear data input to prepare shift register for bit shifting
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, LOW);
  pinMode(clockPin, OUTPUT);
  digitalWrite(clockPin, LOW);

  // Function that checks all the RELAYs
  // gets passed the number of checks and the pause time
  // Enable all outputs for check
  //digitalWrite(enableOutputPin, LOW);
  //selfCheck(1, 200);

  // Disable all outputs until value from EEPROM is loaded, used if selfCheck is run, only
  //digitalWrite(enableOutputPin, HIGH);

  // Read state from EEPROM and store it in pinData variable
  EEPROMwl.begin(EEPROM_LAYOUT_VERSION, AMOUNT_OF_INDEXES);
  EEPROMwl.get(INDEX_CONFIGURATION_pinData, pinData);
  updateShiftRegister(!writeEEPROM);

  Serial.begin(9600);
  scc.set(200, respond);

  // Enable all outputs
  digitalWrite(enableOutputPin, LOW);
  /*** End of relay control setup ***/

  /*** Start of ethernet shield setup ***/
  // You can use Ethernet.init(pin) to configure the CS pin
  // Ethernet.init(10);  // Most Arduino shields
  // Ethernet.init(5);   // MKR ETH shield
  // Ethernet.init(0);   // Teensy 2.0
  // Ethernet.init(20);  // Teensy++ 2.0
  // Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
  // Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

  // Start the Ethernet
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found. Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // Do nothing, no point running without Ethernet hardware
    }
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp.begin(localPort);
  /*** End of ethernet shield setup ***/
}

void loop() {
  /*** Start of relay control main loop ***/
  // Serial cmd input update
  scc.update();
  /*** End of relay control main loop ***/

  /*** Start of ethernet shield main loop ***/

  // UDP cmd input update
  udpUpdate();
  /*** End of ethernet shield main loop ***/
  
  // Wait before next cmd, until relays settle
  delay(10);
}

/*** Start of relay control functions ***/
void updateShiftRegister(boolean writeEEPROM) {
  // Write pinData to shift register

  if (writeEEPROM) {
    // Write pinData to EEPROM
    EEPROMwl.put(INDEX_CONFIGURATION_pinData, pinData);
  }

  digitalWrite(latchPin, LOW);
  // Shift out lowbyte
  shiftOut(dataPin, clockPin, LSBFIRST, pinData);
  // Shift out highbyte
  shiftOut(dataPin, clockPin, LSBFIRST, (pinData >> 8));
  digitalWrite(latchPin, HIGH);
}

void selfCheck(int blinkCount, int delayPeriod) {
  // Blinks the whole register based on the number of times you want to
  // blink "blinkCount" and the pause between them "delayPeriod".

  pinData = 0xFFFF;
  updateShiftRegister(!writeEEPROM);

  delay(200);

  for (int i = 0; i < blinkCount; i++) {
    pinData = 0;
    updateShiftRegister(!writeEEPROM);
    delay(delayPeriod);

    pinData = 0xFFFF;
    updateShiftRegister(!writeEEPROM);
    delay(delayPeriod);
  }
}

void respond() {

  // Serial Callback function
  // Check for proper input with  scc.hasValue()
  // Command is given in format OFF=2, ON=3, etc
  if (scc.hasValue()) {

    // Sanity check (if command contains "ALL")
    if (scc.getValueS() == "ALL") {
      // Dummy value for debugging
      pin = 255;
    }

    else {
      // Extract pin as substring after "=" sign
      pin = scc.getValueInt();

      //Check if pin is out of bounds
      if (pin < 0 || pin > 15) {
        Serial.println(String(ReplyERR) + "=" + 1);
        return;
      }
    }

    if (strcmp(scc.getCmd(), "CHK") == 0) {
      // Read pin status (relay board has negative logic)
      pinStatus = bitRead(pinData, pinIndex[pin]);

      if (pinStatus == 1) {
        Serial.println(String("OFF") + "=" + pin);
      }

      else {
        Serial.println(String("ON") + "=" + pin);
      }
    }

    else if (strcmp(scc.getCmd(), "OFF") == 0) {
      // Check if we want all relays
      if (pin == 255) {
        pinData = 0xFFFF;

        Serial.println("OFF=ALL");
      }

      else {
        bitSet(pinData, pinIndex[pin]);
        Serial.println(String(scc.getCmdS()) + "=" + pin);
      }
      updateShiftRegister(writeEEPROM);

    }

    else if (strcmp(scc.getCmd(), "ON") == 0) {
      // Check if we want all relays
      if (pin == 255) {
        pinData = 0;

        Serial.println("ON=ALL");
      }

      else {
        bitClear(pinData, pinIndex[pin]);
        Serial.println(String(scc.getCmdS()) + "=" + pin);
      }
      updateShiftRegister(writeEEPROM);

    }

    // Exit if wrong or no command is given
    else {
      Serial.println(String(ReplyERR) + "=" + 3);
      return;
    }
  }

  // Exit if no value is given as argument
  else {
    Serial.println(String(ReplyERR) + "=" + 2);
    return;
  }
}
/*** End of relay control functions ***/

/*** Start of ethernet shield functions ***/
String ip2String(const IPAddress & ipAddr) {
  return String(ipAddr[0]) + "." + ipAddr[1] + "." + ipAddr[2] + "." + ipAddr[3];
}

void udpSendErr(int errorNumber) {
  // Send an ERROR reply to the IP address and port that sent us the packet we received
  String myErrMsgS = String(ReplyERR) + "=" + errorNumber;
  char myErrMsgC[PACKET_BUFFER_SIZE];
  myErrMsgS.toCharArray(myErrMsgC, PACKET_BUFFER_SIZE);

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(myErrMsgC);
  Udp.endPacket();

  Serial.println(myErrMsgS);
}

void udpSendMsg(int pinStatus, int pinNumber) {
  // Send an MSG reply to the IP address and port that sent us the packet we received
  String myPinStatus = "OFF";
  String myMsgS;

  if (pinStatus == 1) {
    myPinStatus = "ON";
  }

  if (pinNumber == 255) {
    myMsgS = myPinStatus + "=" + "ALL";
  }

  else {
    myMsgS = myPinStatus + "=" + pinNumber;
  }

  char myMsgC[PACKET_BUFFER_SIZE];
  myMsgS.toCharArray(myMsgC, PACKET_BUFFER_SIZE);

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(myMsgC);
  Udp.endPacket();

  Serial.println(myMsgS);
}

void udpUpdate() {
  
  // Initialize 3 last chars of array on every loop,
  // to fix possible overflow from erroneous previous run.
  // For example if previous cmd is ON=1333 and next is ON=1
  for (int i = PACKET_BUFFER_SIZE - 3; i < PACKET_BUFFER_SIZE; i++) {
    packetBuffer[i] = '\0';
  }
  
  // If there's data available, read a packet
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    // Parse Remote IP
    IPAddress remote = Udp.remoteIP();
    String myRemoteIP = ip2String(remote);
    
    // Poor man's firewall
    if (myRemoteIP != "192.168.78.37") {
      udpSendErr(4);
      // Exit if packet is received from unknown source
      return;
    }
    
    // Read the packet into packetBuffer
    Udp.read(packetBuffer, PACKET_BUFFER_SIZE);
    String packetBufferS = String(packetBuffer);

    // Exit if no value is given as argument
    if (!(packetBufferS.substring(packetBufferS.indexOf("=") + 1)).length()) {
      udpSendErr(2);
      return;
    }
    
    // Sanity check (if command contains "ALL")
    if (packetBufferS.indexOf("ALL") >= 0) {
      // Dummy value for debugging
      pin = 255;
    }

    else {
      // Extract pin as substring after "=" sign
      pin = (packetBufferS.substring(packetBufferS.indexOf("=") + 1)).toInt();

      // Check if pin is out of bounds
      if (pin < 0 || pin > 15) {
        udpSendErr(1);
        return;
      }
    }

    if (packetBufferS.startsWith("CHK")) {
      // Read pin status (relay board has negative logic)
      pinStatus = bitRead(pinData, pinIndex[pin]);

      // Send a reply to the IP address and port that sent us the packet we received
      udpSendMsg(!pinStatus, pin);
    }

    else if (packetBufferS.startsWith("OFF")) {
      // Check if we want all relays
      if (pin == 255) {
        pinData = 0xFFFF;
      }

      else {
        bitSet(pinData, pinIndex[pin]);
      }

      // Send a reply to the IP address and port that sent us the packet we received
      udpSendMsg(!pinStatus, pin);
      updateShiftRegister(writeEEPROM);
    }

    else if (packetBufferS.startsWith("ON")) {
      // Check if we want all relays
      if (pin == 255) {
        pinData = 0;
      }

      else {
        bitClear(pinData, pinIndex[pin]);
      }

      // Send a reply to the IP address and port that sent us the packet we received
      udpSendMsg(!pinStatus, pin);
      updateShiftRegister(writeEEPROM);
    }

    // Exit if wrong or no command is given
    else {
      udpSendErr(3);
      return;
    }
  }
  
  // Exit if no data is received
  else {
    return;
  }
}
/*** End of ethernet shield functions ***/
