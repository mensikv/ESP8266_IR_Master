/*
  ####
  ## ESP8266 IR Master
  ####
  Board: Lolin(Wemos) D1 R2 & mini
  Flash size: 4M (1M SPIFFS)
  When you want to erase EEPROM: Erase Flash: All Flash Contents

  upload code a then upload files

  use ESP8266 Sketch Data Upload to upload files to ESP8266 SPIFFS: // https://github.com/esp8266/arduino-esp8266fs-plugin
  data\index.html.gz
  data\bootstrap.min.js.gz
  data\jquery-3.4.1.min.js.gz
  data\bootstrap.min.css.gz
  data\favicon.ico

  Tested with versions:
  Arduino IDE 1.8.10

  ESP8266 core 2.5.2

  Libraries:
  ArduinoJSON 6.12.0
  ESP_EEPROM 2.0.0
  IRRemoteESP8266 2.6.6
  WifiManages 0.15.0-beta
  arduinoWebSockets 2.1.1
  PubSubClient 2.7
    you should increase MQTT_MAX_PACKET_SIZE in PubSubClient.h for 5 irCodes is 512 sufficient
      #define MQTT_MAX_PACKET_SIZE 512
*/

#include <Arduino.h>
#include <FS.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal - needed for WifiManager
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal

#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESP_EEPROM.h>           // https://github.com/jwrw/ESP_EEPROM
#include <PinButton.h>            // https://github.com/poelstra/arduino-multi-button
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson
#include <WebSocketsServer.h>     // https://github.com/Links2004/arduinoWebSockets
#include <IRremoteESP8266.h>      // https://github.com/crankyoldgit/IRremoteESP8266
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>

/*
  ## WIFI Manager function
  When your ESP starts up, it sets it up in Station mode and tries to connect to a previously saved Access Point
  if this is unsuccessful (or no previous network saved) it moves the ESP into Access Point mode and spins up a DNS and WebServer (ip 192.168.0.1)
  using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point
  because of the Captive Portal and the DNS server you will either get a 'Join to network' type of popup or get any domain you try to access redirected to the configuration portal
  choose one of the access points scanned, enter password, click save
  ESP will try to connect. If successful, it relinquishes control back to your app. If not, reconnect to AP and reconfigure.
*/

#define ESP_HOSTNAME "IRMASTER001"

#define IR_CODE_COUNT 5                       // Number of IR codes that could be learned
#define IR_CODE_NAME_MAX_LENGTH 20            // Maximum length of IR code name (excluding ending 0)
#define IR_DEFAULT_REPEAT_COUNT 2             // how many times is an IR code fired
#define IR_DEFAULT_REPEAT_INTERVAL_MS 5       // how many milliseconds between IR code firing
#define MAX_REC_IR_WAITING_TIME_MS 5000       // How long to wait for IR code when learning new one

// APIs - if one is not needed, it should be commented
//#define USE_REST_API                          // /api?all - JSON with all ir codes; /api?fire=irCodeIndex - sends ir Code with irCodeIndex index
//#define USE_MQTT                              // all/fire

#define BUTTON_PIN D3                         // Pin where the button is connected
#define KBAUDRATE 115200                      // The Serial connection baud rate.

#define BUTTON_NO_CLICK 0
#define BUTTON_SINGLE_CLICK 1
#define BUTTON_DOUBLE_CLICK 2

#ifdef USE_MQTT
#include <PubSubClient.h>                     // https://github.com/knolleary/pubsubclient
#define MQTT_SERVER "192.168.1.201"
#define MQTT_USERNAME "irmaster"
#define MQTT_PASSWORD "password"
#define MQTT_TOPIC           ESP_HOSTNAME
#define MQTT_CMD_TOPIC       MQTT_TOPIC "/cmd"                  // payload "all", "fire x", where x is irCodeIndex
#define MQTT_ALL_TOPIC       MQTT_TOPIC "/all"                  // response JSON with all ir codes
#define MQTT_FIRE_TOPIC      MQTT_TOPIC "/fire"                 // response JSON with information about success or error
#define MQTT_LAST_WILL_TOPIC MQTT_TOPIC "/live"
#define MQTT_LAST_WILL "OFF"
#endif

// ==================== IRremoteESP8266 TUNEABLE PARAMETERS ====================
// The GPIO an IR detector/demodulator is connected to.
const uint16_t kRecvPin = D4;

// GPIO to use to control the IR LED circuit.
const uint16_t kIrLedPin = D2;

// As this program is a special purpose capture/resender, let's use a larger than expected buffer so we can handle very large IR messages.
const uint16_t kCaptureBufferSize = 1024;  // 1024 == ~511 bits

// kTimeout is the Nr. of milli-Seconds of no-more-data before we consider a message ended.
const uint8_t kTimeout = 50;  // Milli-Seconds

// kFrequency is the modulation frequency all UNKNOWN messages will be sent at.
const uint16_t kFrequency = 38000;  // in Hz. e.g. 38kHz.


// ==================== Types definition ====================
// structure to hold information about an IR code
typedef struct {
  char irCodeName[IR_CODE_NAME_MAX_LENGTH + 1];
  decode_type_t protocol;                   // NEC, SONY, RC5, UNKNOWN
  uint16_t irCodeLength;                    // number of bytes in irCodeState or number of bits in irCodeValue or number of raw data in irCodeRaw
  union {
    uint64_t irCodeValue;                   // Decoded value - a simple message protocol. ie. <= 64 bits
    uint8_t  irCodeState[kStateSizeMax];    // Multi-byte results for AC protocol with states
    uint16_t irCodeRaw[kRawBuf];
  };
} irCodeType;

// Settings
typedef struct {
  uint8_t irCodeCount;                  // 1 byte, Number of learned IR codes
  uint8_t singleClickIRCodeId;          // 1 byte, Index for IR code associated with button single click
  uint8_t doubleClickIRCodeId;          // 1 byte, Index for IR code associated with button double click
} irMasterSettingType;

typedef struct {                        // structure, that is written into the EEPROM memory
  uint32_t crc32;                       // 4 bytes
  irMasterSettingType irMasterSetting;
  irCodeType irCodes[IR_CODE_COUNT];
} irMasterEEPROMType;

// ==================== Global variables ====================
irMasterEEPROMType   irMasterEEPROM;          // contains all IR codes, CRC32 and other informations
irCodeType           *irCodes;                // IR codes the application operates with
irMasterSettingType  *irMasterSetting;        // Will containt info about button, number of learned IR codes and CRC32 of EEPROM
irCodeType irCode;                            // Will contain new IR code when learning

// Strings for storing JSON and IR codes
String strVar;   // reserve for 512 characters
String output;   // reserve for 768 characters

// The IR transmitter.
IRsend irsend(kIrLedPin);

// Web serever will run on port 80
ESP8266WebServer server(80);

// Websocket server will run on port 81
WebSocketsServer websocket_server = WebSocketsServer(81);

// Ticker is used to periodically call an function - we use it to blink the LED during WIFI setup
Ticker ticker;

// Create a new button object, listening on pin BUTTON_PIN = D3.
PinButton myButton(BUTTON_PIN);

#ifdef USE_MQTT
WiFiClient espWifiClient;
PubSubClient mqtt_client(espWifiClient);
#endif


// ==================== WIFI AP configuration ====================
// Function will turn on/off LED on Wemos board
void tick() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// When Wifi Manager could not connecct to WIFI, it will start up its own AP with configuration portal. During configuration we will blink the LED
void startWMConfig(WiFiManager *wmp) {
  // every 200 ms call function tick
  ticker.attach(0.2, tick);
}

// Once WIFI configuration end, we stop blinking
void endWMConfig() {
  ticker.detach();
  digitalWrite(LED_BUILTIN, HIGH);
}


// ==================== ArduinoJSON examples ====================
/*
  //Adding array of objects - {"elem":[{"elem1":1},{"elem2":2}]}
  JsonArray elem = responseJSON.createNestedArray("elem");
  JsonObject elem1 = elem.createNestedObject();
  elem1["elem1"] = 1;
  JsonObject elem2 = elem.createNestedObject();
  elem2["elem2"] = 2;

  // Adding array {"elem":[1, 2]}
  JsonArray elem = responseJSON.createNestedArray("elem");
  elem.add(1);
  elem.add(2);

  // Adding Object {"elem":{"elem1":1,"elem2":2}}
  JsonObject elem = responseJSON.createNestedObject("elem");
  elem["elem1"] = 1;
  elem["elem2"] = 2;
*/

// ==================== JSON responses ====================
void allResponse (bool longFormat = false) {
  // create object for creating JSON
  //biggest JSON object:{"rsT":"all", "rsS":"OK", "rsE":"","rsV":[{"id":0,"name":"TV on", "ir":"123456", "btn":0},{"id":1,"name":"TV off", "ir":"987654", "btn":1},{"id":1,"name":"TV off", "ir":"987654", "btn":2},{"id":1,"name":"TV off", "ir":"987654", "btn":0},{"id":1,"name":"TV off", "ir":"987654", "btn":0}]}
  const int capacityResponseJSON = JSON_ARRAY_SIZE(5) + 6 * JSON_OBJECT_SIZE(4) + 256;
  StaticJsonDocument<capacityResponseJSON> responseJSON;
  //StaticJsonDocument<1024> responseJSON;
  responseJSON["rsT"] = "all";
  responseJSON["rsS"] = "OK";
  responseJSON["rsE"] = "";
  output = "";
  yield();
  //{rsT:"all", rsS:"OK", rsE:"",rsV:[{id:0,name:"TV on", ir:"123456", "btn":0},{id:1,name:"TV off", ir:"987654", "btn":1}]};
  JsonArray rsV = responseJSON.createNestedArray("rsV");
  for (uint8_t i = 0; i < IR_CODE_COUNT; i++) {
    if (irCodes[i].irCodeLength > 0) {
      JsonObject irCodeElem = rsV.createNestedObject();
      irCodeElem["id"] = i;
      irCodeElem["name"] = irCodes[i].irCodeName;
      if (longFormat) {
        irCodeElem["ir"] = getIRCode(&irCodes[i], false);
        if (i == irMasterSetting->singleClickIRCodeId) irCodeElem["btn"] = BUTTON_SINGLE_CLICK;
        else if (i == irMasterSetting->doubleClickIRCodeId) irCodeElem["btn"] = BUTTON_DOUBLE_CLICK;
        else irCodeElem["btn"] = BUTTON_NO_CLICK;
      }
    }
  }
  serializeJson(responseJSON, output);
  yield();
}

void scanResponse (uint8_t scanResult) {
  // create object for creating JSON
  const int capacityResponseJSON = JSON_OBJECT_SIZE(4) + 100;
  StaticJsonDocument<capacityResponseJSON> responseJSON;
  responseJSON["rsT"] = "scan";
  responseJSON["rsS"] = "OK";
  responseJSON["rsE"] = "";
  output = "";
  yield();
  //{"rsT":"scan", "rsS":"OK", "rsE":"","rsV":"123456"}
  if (scanResult == 1) {
    responseJSON["rsV"] = getIRCode(&irCode, false);
  } else {
    responseJSON["rsS"] = "ERROR";
    responseJSON["rsE"] = (scanResult == 0) ? "IR code scan timeout" : "IR code overflow";
  }
  serializeJson(responseJSON, output);
  yield();
}

void fireResponse (uint8_t fireResult, const uint8_t irCodeIndex) {
  // create object for creating JSON
  const int capacityResponseJSON = JSON_OBJECT_SIZE(4) + 60;
  StaticJsonDocument<capacityResponseJSON> responseJSON;
  responseJSON["rsT"] = "fire";
  responseJSON["rsS"] = "OK";
  responseJSON["rsE"] = "";
  responseJSON["rsV"] = irCodeIndex;
  output = "";
  yield();
  //{"rsT":"fire", "rsS":"OK", "rsE":"","rsV":9}
  if (fireResult != 1) {
    responseJSON["rsS"] = "ERROR";
    responseJSON["rsE"] = (fireResult == 0) ? "Error during sending of IR code" : "Non-existing index";
  }
  serializeJson(responseJSON, output);
  yield();
}

void deleteResponse (uint8_t deleteResult, const uint8_t irCodeIndex) {
  // create object for creating JSON
  const int capacityResponseJSON = JSON_OBJECT_SIZE(4) + 60;
  StaticJsonDocument<capacityResponseJSON> responseJSON;
  responseJSON["rsT"] = "delete";
  responseJSON["rsS"] = "OK";
  responseJSON["rsE"] = "";
  responseJSON["rsV"] = irCodeIndex;
  output = "";
  yield();
  //{"rsT":"delete", "rsS":"OK", "rsE":"","rsV":9}
  if (deleteResult != 1) {
    responseJSON["rsS"] = "ERROR";
    responseJSON["rsE"] = (deleteResult == 0) ? "Error during deleting operation" : "Non-existing index";
  }
  serializeJson(responseJSON, output);
  yield();
}

void learnResponse (uint8_t learnResult, const uint8_t irCodeIndex) {
  // create object for creating JSON
  const int capacityResponseJSON = 2 * JSON_OBJECT_SIZE(4) + 60;
  StaticJsonDocument<capacityResponseJSON> responseJSON;
  responseJSON["rsT"] = "learn";
  responseJSON["rsS"] = "OK";
  responseJSON["rsE"] = "";
  output = "";
  yield();
  //{"rsT":"learn", "rsS":"OK", "rsE":"","rsV":{"id":0,"name":"TV on", "ir":"123456", "btn":0}}
  if (learnResult != 1) {
    responseJSON["rsS"] = "ERROR";
    if (learnResult == 0) responseJSON["rsE"] = "Could not save the new IR code to EEPROM.";
    else if (learnResult == 2) responseJSON["rsE"] = "No more IR codes can be learned. Delete some first.";
    else responseJSON["rsE"] = "There is no IR to learn. First use scan.";
  } else {
    JsonObject rsV = responseJSON.createNestedObject("rsV");
    rsV["id"] = irCodeIndex;
    rsV["name"] = irCodes[irCodeIndex].irCodeName;
    rsV["ir"] = getIRCode(&irCodes[irCodeIndex], false);
    rsV["btn"] = BUTTON_NO_CLICK;
  }
  serializeJson(responseJSON, output);
  yield();
}

void clickResponse (uint8_t clickResult, const uint8_t irCodeIndex) {
  // create object for creating JSON
  const int capacityResponseJSON = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(4) + 60;
  StaticJsonDocument<capacityResponseJSON> responseJSON;
  responseJSON["rsT"] = "click";
  responseJSON["rsS"] = "OK";
  responseJSON["rsE"] = "";
  output = "";
  yield();
  //{"rsT":"click", "rsS":"OK", "rsE":"","rsV":{"id":5,"btn":1}}
  if (clickResult != 1) {
    responseJSON["rsS"] = "ERROR";
    if (clickResult == 0) responseJSON["rsE"] = "Could not save to EEPROM.";
    else responseJSON["rsE"] = "Non-existing index.";
  } else {
    JsonObject rsV = responseJSON.createNestedObject("rsV");
    rsV["id"] = irCodeIndex;
    if (irCodeIndex == irMasterSetting->singleClickIRCodeId) rsV["btn"] = BUTTON_SINGLE_CLICK;
    else if (irCodeIndex == irMasterSetting->doubleClickIRCodeId) rsV["btn"] = BUTTON_DOUBLE_CLICK;
    else rsV["btn"] = BUTTON_NO_CLICK;
  }
  serializeJson(responseJSON, output);
  yield();
}

// ==================== Websocket received event processing ====================
// This function is executed when we receive event through Websocket interface
void websocketProcess(uint8_t num, WStype_t type, uint8_t* data, size_t length) {
  digitalWrite(LED_BUILTIN, LOW);    // blick when processing

  if (type == WStype_CONNECTED) {
    IPAddress ip = websocket_server.remoteIP(num);
    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], data);
    //websocket_server.sendTXT(num, "Connected");
  }
  else if (type == WStype_DISCONNECTED) {
    Serial.printf("[%u] Disconnected!\n", num);
  }
  else if (type == WStype_TEXT) {  // when we receive message from a client
    Serial.printf("[%u] get Text: %s\n", num, data);

    // create object for processing
    //biggest JSON object:{"rqT":"click","rqV":{"id":5,"btn":1}}
    const int capacityRequestJSON = 2 * JSON_OBJECT_SIZE(2) + 30;
    StaticJsonDocument<capacityRequestJSON> requestJSON;

    // Request messages
    // {"rqT":"all","rqV":1}
    // {"rqT":"scan","rqV":1}
    // {"rqT":"fire","rqV":id}
    // {"rqT":"delete","rqV":id}
    // {"rqT":"learn","rqV":"name"}
    // {"rqT":"click","rqV":{"id":5,"btn":1}}

    // parse incoming JSON message
    DeserializationError err = deserializeJson(requestJSON, data);
    if (err) {
      Serial.print(F("deserializeJson() failed with code "));
      Serial.println(err.c_str());
      digitalWrite(LED_BUILTIN, HIGH);
      return;
    }
    yield();

    if (requestJSON["rqT"] == "all") { // return all IR codes
      // ###########################
      // ## all
      // ###########################
      Serial.println(F("Received *all* request"));
      //{rsT:"all", rsS:"OK", rsE:"",rsV:[{id:0,name:"TV on", ir:"123456", "btn":0},{id:1,name:"TV off", ir:"987654", "btn":1}]};
      allResponse(true);
      Serial.printf("*all* response: %s\n", output.c_str());
      websocket_server.sendTXT(num, output);

    } else if (requestJSON["rqT"] == "scan") { // start scannig for IR codes and return scanned IR code
      // ###########################
      // ## scan
      // ###########################
      Serial.println(F("Received *scan* request"));
      // {rsT:"scan", rsS:"OK", rsE:"",rsV:"123456"};
      uint8_t scanResult = receiveIRCode();
      scanResponse(scanResult);
      Serial.printf("*scan* response: %s\n", output.c_str());
      websocket_server.sendTXT(num, output);

    } else if (requestJSON["rqT"] == "fire") { // fire (send) ir code and return success or error
      // ###########################
      // ## fire
      // ###########################
      Serial.println(F("Received *fire* request"));
      const uint8_t irCodeIndex = requestJSON["rqV"];
      //{rsT:"fire", rsS:"OK", rsE:"",rsV:9};
      uint8_t fireResult = sendIRCode(irCodeIndex);
      fireResponse(fireResult, irCodeIndex);
      Serial.printf("*fire* response: %s\n", output.c_str());
      websocket_server.sendTXT(num, output);

    } else if (requestJSON["rqT"] == "delete") { // delete ir code and return success or error
      // ###########################
      // ## delete
      // ###########################
      Serial.println(F("Received *delete* request"));
      const uint8_t irCodeIndex = requestJSON["rqV"];
      //{rsT:"delete", rsS:"OK", rsE:"",rsV:9};
      uint8_t deleteResult = deleteIRCode(irCodeIndex);
      deleteResponse(deleteResult, irCodeIndex);
      Serial.printf("*delete* response: %s\n", output.c_str());
      websocket_server.broadcastTXT(output);

    } else if (requestJSON["rqT"] == "learn") { // save last scanned ir code and return it to the client
      // ###########################
      // ## learn
      // ###########################
      Serial.println(F("Received *learn* request"));
      const char* irCodeName = requestJSON["rqV"];
      uint8_t newIRCodeIndex;
      uint8_t learnResult = learnIRCode(irCodeName, &newIRCodeIndex);
      //{rsT:"learn", rsS:"OK", rsE:"",rsV:{id:0,name:"TV on", ir:"123456", "btn":0}}
      learnResponse(learnResult, newIRCodeIndex);
      Serial.printf("*learn* response: %s\n", output.c_str());
      websocket_server.broadcastTXT(output);

    } else if (requestJSON["rqT"] == "click") { // associate button click/double click with an ir code, return succes/error
      // ###########################
      // ## click
      // ###########################
      Serial.println(F("Received *click* request"));
      const uint8_t irCodeIndex = requestJSON["rqV"]["id"];
      const uint8_t clickType = requestJSON["rqV"]["btn"];
      uint8_t clickResult = setButtonClick(irCodeIndex, clickType);
      //{"rsT":"click", "rsS":"OK", "rsE":"","rsV":{"id":5,"btn":1}}
      clickResponse(clickResult, irCodeIndex);
      Serial.printf("*click* response: %s\n", output.c_str());
      websocket_server.sendTXT(num, output);

    } else {
      Serial.println(F("Received not recognised request"));
      websocket_server.sendTXT(num, "ERROR - Received not recognised request");
    }

  } else {
    switch (type) {
      case WStype_ERROR: Serial.println(F("WStype_ERROR")); break;
      case WStype_BIN: Serial.println(F("WStype_BIN")); break;
      case WStype_FRAGMENT_TEXT_START: Serial.println(F("WStype_FRAGMENT_TEXT_START")); break;
      case WStype_FRAGMENT_BIN_START: Serial.println(F("WStype_FRAGMENT_BIN_START")); break;
      case WStype_FRAGMENT: Serial.println(F("WStype_FRAGMENT")); break;
      case WStype_FRAGMENT_FIN: Serial.println(F("WStype_FRAGMENT_FIN")); break;
      case WStype_PING: Serial.println(F("WStype_PING")); break;
      case WStype_PONG: Serial.println(F("WStype_PONG")); break;
    }
  }
  digitalWrite(LED_BUILTIN, HIGH);
}


// ==================== IR codes processing ====================
// Associate button click/double click with and IR code
uint8_t setButtonClick(uint8_t irCodeIndex, uint8_t clickType) {
  Serial.println(F("Set button click action."));
  Serial.printf("For index %i and value %i\n", irCodeIndex, clickType);
  uint8_t result = 1;
  if (!checkIRCodeExists(irCodeIndex)) {
    Serial.printf("Non-existing IR code index %i\n\n", irCodeIndex);
    return 2;
  }
  if ((clickType != BUTTON_NO_CLICK) && (clickType != BUTTON_SINGLE_CLICK) && (clickType != BUTTON_DOUBLE_CLICK)) clickType = BUTTON_NO_CLICK;
  if (clickType == BUTTON_NO_CLICK) {  // delete action but do not save
    if (irCodeIndex == irMasterSetting->singleClickIRCodeId) irMasterSetting->singleClickIRCodeId = IR_CODE_COUNT + 1;
    else if (irCodeIndex == irMasterSetting->doubleClickIRCodeId) irMasterSetting->doubleClickIRCodeId = IR_CODE_COUNT + 1;
  } else {
    if (clickType == BUTTON_SINGLE_CLICK) irMasterSetting->singleClickIRCodeId = irCodeIndex;
    else irMasterSetting->doubleClickIRCodeId = irCodeIndex;

    if (irMasterSetting->singleClickIRCodeId == irMasterSetting->doubleClickIRCodeId) {
      if (clickType == BUTTON_SINGLE_CLICK) irMasterSetting->doubleClickIRCodeId = IR_CODE_COUNT + 1;
      else irMasterSetting->singleClickIRCodeId = IR_CODE_COUNT + 1;
    }
  }
  result = saveEEPROM();
  return result;
}

// Scan/receive new IR code and store the result in variable irCode
uint8_t receiveIRCode() {
  Serial.println(F("Start listening for IR code."));

  IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, false);  // The IR receiver.
  decode_results results;                                        // Somewhere to store the captured message.

  irrecv.enableIRIn();  // Start up the IR receiver.

  uint32_t start_t = millis();
  uint32_t now = start_t;
  do {
    // Check if an IR message has been received.
    if (irrecv.decode(&results)) {  // We have captured something. The capture has stopped at this point.
      yield();
      if (results.overflow) {
        Serial.printf("WARNING: IR code is too big for buffer (>= %d). This result shouldn't be trusted until this is resolved. Edit & increase kCaptureBufferSize.\n", kCaptureBufferSize);
        irrecv.disableIRIn();  // Detach interrupts
        return 2;
      }
      Serial.println(resultToHumanReadableBasic(&results));
      irCode.irCodeName[0] = '\0';
      irCode.protocol = results.decode_type;
      uint16_t size = results.bits;
      // Is it a protocol we don't understand?
      if (irCode.protocol == decode_type_t::UNKNOWN) {  // Yes.
        uint16_t *raw_array = resultToRawArray(&results);                     // Convert the results into an array suitable for sendRaw(). resultToRawArray() allocates the memory we need for the array.
        size = getCorrectedRawLength(&results);                               // Find out how many elements are in the array.
        memcpy(irCode.irCodeRaw, raw_array, size * sizeof(uint16_t));
        irCode.irCodeLength = size;
        delete [] raw_array;                                                  // Deallocate the memory allocated by resultToRawArray()
      } else if (hasACState(irCode.protocol)) {  // Does the message require a state[]?
        // It does, so send with bytes instead.
        memcpy(irCode.irCodeState, results.state, (size / 8) * sizeof(uint8_t));
        irCode.irCodeLength = size / 8;
      } else {  // Anything else must be a simple message protocol. ie. <= 64 bits
        irCode.irCodeValue = results.value;
        irCode.irCodeLength = size;
      }
      // Display a crude timestamp & notification.
      Serial.printf("%06u.%03u: A %d-bit %s message was captured.\n", now / 1000, now % 1000, size, typeToString(irCode.protocol).c_str());
      irrecv.disableIRIn();  // Detach interrupts
      dumpIRCode(&irCode, false);
      return 1;
    }
    now = millis();
    yield();  // Or delay(milliseconds); This ensures the ESP doesn't WDT reset.
  }  while (( start_t + MAX_REC_IR_WAITING_TIME_MS ) > now);
  irrecv.disableIRIn();  // Detach interrupts
  return 0;
}

// Fire (send) IR code
uint8_t sendIRCode(uint8_t irCodeIndex) {
  Serial.print(F("Sending IR command"));
  uint8_t result = 1;
  if (!checkIRCodeExists(irCodeIndex)) {
    Serial.printf("\nNon-existing IR code index %i\n\n", irCodeIndex);
    return 2;
  }
  irCodeType *ir;
  ir = &irCodes[irCodeIndex];
  Serial.printf("Idex %i and name %s \n", irCodeIndex, ir->irCodeName);
  for (uint8_t i = 0; i < IR_DEFAULT_REPEAT_COUNT; i++) {
    if (ir->protocol == decode_type_t::UNKNOWN) {
      irsend.sendRaw(ir->irCodeRaw, ir->irCodeLength, kFrequency);
    } else if (hasACState(ir->protocol)) {  // Does the message require a state[]? It does, so send with bytes instead.
      result = irsend.send(ir->protocol, ir->irCodeState, ir->irCodeLength);
    } else {  // Anything else must be a simple message protocol. ie. <= 64 bits
      result = irsend.send(ir->protocol, ir->irCodeValue, ir->irCodeLength);
    }
    if (!result) break;
    delay(IR_DEFAULT_REPEAT_INTERVAL_MS);
  }
  return result;
}

// Delete saved IR code
uint8_t deleteIRCode(uint8_t irCodeIndex) {
  Serial.printf("Delete IR code %i\n", irCodeIndex);
  if (!checkIRCodeExists(irCodeIndex)) {
    Serial.printf("Non-existing IR code index %i\n\n", irCodeIndex);
    return 2;
  }
  irMasterSetting->irCodeCount--;
  resetIRCode(&irCodes[irCodeIndex]);
  if (irCodeIndex == irMasterSetting->singleClickIRCodeId) irMasterSetting->singleClickIRCodeId = IR_CODE_COUNT + 1;
  else if (irCodeIndex == irMasterSetting->doubleClickIRCodeId) irMasterSetting->doubleClickIRCodeId = IR_CODE_COUNT + 1;
  return saveEEPROM();
}

// Learn (save) new IR code (it is in vaiable irCode)
uint8_t learnIRCode(const char *irCodeName, uint8_t *newIRCodeIndex) {
  Serial.print(F("Learn IR code "));
  uint8_t result = 1;
  if (irMasterSetting->irCodeCount == IR_CODE_COUNT) return 2;
  if (irCode.irCodeLength <= 0) return 3;
  // looking for free index
  uint8_t i = 0;
  while (irCodes[i].irCodeLength != 0) i++;
  Serial.printf("using index %i\n", i);
  strcpy(irCodes[i].irCodeName, irCodeName);
  irCodes[i].protocol = irCode.protocol;
  irCodes[i].irCodeLength = irCode.irCodeLength;
  memcpy(irCodes[i].irCodeRaw, irCode.irCodeRaw, irCode.irCodeLength * sizeof(uint16_t));
  irMasterSetting->irCodeCount++;
  resetIRCode(&irCode);
  result = saveEEPROM();
  *newIRCodeIndex = i;
  return result;
}


// ==================== IR codes print and help functions ====================
void resetIRCode(irCodeType * ir) {
  ir->irCodeName[0] = '\0';
  ir->irCodeLength = 0;
  ir->irCodeValue = 0;
}

// Return string with IR code value
String getIRCode(const irCodeType * ir, bool fullIR) {
  strVar = typeToString(ir->protocol).c_str();
  strVar += ":";

  if ((ir->protocol == decode_type_t::UNKNOWN) || (hasACState(ir->protocol))) {
    strVar += " (";
    strVar += ir->irCodeLength;
    strVar += "x)";
    for (uint16_t i = 0; i < ir->irCodeLength; i++) {
      strVar += " ";
      if (ir->protocol == decode_type_t::UNKNOWN) strVar += ir->irCodeRaw[i];
      else strVar += ir->irCodeState[i];
      if ((strVar.length() > 30) && (!fullIR)) {
        strVar += "...";
        break;
      }
    }
  } else strVar += uint64ToString(ir->irCodeValue);

  return strVar;
}

// Return true/false if IR code with index irCodeIndex exists
uint8_t checkIRCodeExists(uint8_t irCodeIndex) {
  if ((irCodeIndex >= 0) && (irCodeIndex < IR_CODE_COUNT) && (irCodes[irCodeIndex].irCodeLength > 0)) return true;
  else return false;
}

// Write information about an IR code to serial output
void dumpIRCode(const irCodeType * ir, bool fullIR) {
  Serial.print(F("IR code name: "));
  Serial.println(ir->irCodeName);
  Serial.print(F("IR code protocol: "));
  Serial.println(typeToString(ir->protocol).c_str());
  Serial.print(F("IR code length: "));
  Serial.println(ir->irCodeLength);
  Serial.print(F("IR code: "));
  Serial.println(getIRCode(ir, fullIR));
  Serial.println();
}

// Write information about all saved IR codes to serial output
void dumpStoredIRCodes (bool fullIR) {
  Serial.println(F("Dump of all stored IR codes"));
  for (uint8_t i = 0; i < IR_CODE_COUNT; i++) {
    if (irCodes[i].irCodeLength > 0) {
      Serial.print(F("IR code id: "));
      Serial.println(i);
      dumpIRCode(&irCodes[i], fullIR);
    }
  }
}


// ==================== Button click ====================
void processClick(uint8_t clickType) {
  int8_t irCodeIndex = (clickType == BUTTON_SINGLE_CLICK) ? irMasterSetting->singleClickIRCodeId : irMasterSetting->doubleClickIRCodeId;
  if (checkIRCodeExists(irCodeIndex)) {
    Serial.printf("Firing IR after %s click\n", ((clickType == BUTTON_SINGLE_CLICK) ? "single" : "double"));
    Serial.printf("Index %i and name %s \n", irCodeIndex, irCodes[irCodeIndex].irCodeName);
    sendIRCode(irCodeIndex);
  } else {
    Serial.printf("No IR associated with %s click\n", ((clickType == BUTTON_SINGLE_CLICK) ? "single" : "double"));
  }
}

// ==================== Web server help functions ====================
// Return content type of a file
String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

// Load file from SPIFFS and return it through the web server
bool handleFileRead(String path) { // send the right file to the client (if it exists)
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";          // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    if (SPIFFS.exists(pathWithGz))                         // If there's a compressed version available
      path += ".gz";                                         // Use the compressed version
    yield();
    File file = SPIFFS.open(path, "r");                    // Open the file
    size_t sent = server.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again
    Serial.println(String("\tSent file: ") + path);
    return true;
  }
  Serial.println(String("\tFile Not Found: ") + path);
  return false;                                          // If the file doesn't exist, return false
}


// ==================== MQTT API ====================
#ifdef USE_MQTT
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  digitalWrite(LED_BUILTIN, LOW);
  Serial.printf("MQTT message arrived, topic [%s], payload [", topic);
  char payloadMessage[50];
  memcpy(payloadMessage, payload, (length > 49) ? 49 : length);
  payloadMessage[(length > 49) ? 49 : length] = '\0';
  Serial.printf("%s]\n", payloadMessage);
  yield();

  if (strcmp(payloadMessage, "all") == 0) {
    // ###########################
    // ## all
    // ###########################
    Serial.println(F("Received *all* MQTT request"));
    //{rsT:"all", rsS:"OK", rsE:"",rsV:[{id:0,name:"TV on"},{id:1,name:"TV off"}]};
    allResponse(false);
    Serial.printf("*all* MQTT topic [%s] response: %s\n", MQTT_ALL_TOPIC, output.c_str());
    mqtt_client.publish(MQTT_ALL_TOPIC, output.c_str(), false);

  } else if (strncmp(payloadMessage, "fire", 4) == 0) {
    // ###########################
    // ## fire
    // ###########################
    Serial.println(F("Received *fire* MQTT request"));
    uint8_t irCodeIndex = atoi(&payloadMessage[5]);
    //sscanf(&payloadMessage[5], "%d", &irCodeIndex);
    //{rsT:"fire", rsS:"OK", rsE:"",rsV:9};
    uint8_t fireResult = sendIRCode(irCodeIndex);
    fireResponse(fireResult, irCodeIndex);
    Serial.printf("*fire* MQTT topic [%s] response: %s\n", MQTT_FIRE_TOPIC, output.c_str());
    mqtt_client.publish(MQTT_FIRE_TOPIC, output.c_str(), false);

  } else {
    Serial.println(F("Received *UNKNOWN* MQTT request"));
    mqtt_client.publish(MQTT_LAST_WILL_TOPIC, "ERROR - Unknown MQTT command, supported: all;fire x", false);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void mqttConnect() {
  mqtt_client.setServer(MQTT_SERVER, 1883);
  mqtt_client.setCallback(mqttCallback);

  // Loop until we're reconnected
  uint8_t i = 0;
  while (!mqtt_client.connected()) {
    i++;
    // Attempt to connect
    //boolean connect(const char* id, const char* user, const char* pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage);
    if (!mqtt_client.connect(ESP_HOSTNAME, MQTT_USERNAME, MQTT_PASSWORD, MQTT_LAST_WILL_TOPIC, 0, 1, MQTT_LAST_WILL)) {
      Serial.printf("failed, rc=%s\n", mqtt_client.state());
      if (i <= 10) {
        Serial.println(F(" try again in 500ms"));
        delay(500); // Wait 500ms before retrying
      } else {
        ESP.reset();
      }
    }
  }

  mqtt_client.publish(MQTT_LAST_WILL_TOPIC, "ON", false);
  mqtt_client.subscribe(MQTT_CMD_TOPIC);
}
#endif


// ==================== EEPROM helpers ====================
uint8_t saveEEPROM() {
  Serial.println(F("EEPROM saving"));
  irMasterEEPROM.crc32 =  calculateCRC32( ((uint8_t*)&irMasterEEPROM) + 4, sizeof( irMasterEEPROM ) - 4 );

  EEPROM.put(0, irMasterEEPROM);
  uint8_t result = EEPROM.commit();
  yield();
  Serial.println((result) ? F("Commit OK") : F("Commit failed"));
  return result;
}

void resetEEPROM() {
  for (uint8_t i = 0; i < IR_CODE_COUNT; i++) {
    resetIRCode(&irCodes[i]);
  }
  irMasterSetting->irCodeCount = 0;
  irMasterSetting->singleClickIRCodeId = IR_CODE_COUNT + 1;
  irMasterSetting->doubleClickIRCodeId = IR_CODE_COUNT + 1;

  saveEEPROM();
}

uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while ( length-- ) {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if ( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if ( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}


// ==================== SETUP ====================
void setup() {
  Serial.begin(KBAUDRATE, SERIAL_8N1);
  while (!Serial) delay(50); // Wait for the serial connection to be establised.

  Serial.printf("\n\nESP8266 IR Master - %s\n\n", ESP_HOSTNAME);

  // get possible reset reason
  Serial.printf("Start: %s\n\n", ESP.getResetReason().c_str());

  // Name and MAC address
  Serial.printf("Hostname: %s\n", ESP_HOSTNAME);
  Serial.printf("MAC address: %s\n\n", WiFi.macAddress().c_str());

  // Set the pin that controls the LED on Wemos as OUTPUT and set its output HIGH (the LED shines on LOW)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // reserve memory for long strings
  strVar.reserve(512);
  output.reserve(768);

  Serial.print(F("Loading from EEPROM... "));
  irMasterSetting = &irMasterEEPROM.irMasterSetting;
  irCodes = &irMasterEEPROM.irCodes[0];

  EEPROM.begin(sizeof(irMasterEEPROMType));
  if (EEPROM.percentUsed() >= 0) {
    Serial.printf("EEPROM has data from a previous run.%i%% of ESP flash space currently used\n", EEPROM.percentUsed());

    EEPROM.get(0, irMasterEEPROM);
    uint32_t crc = calculateCRC32( ((uint8_t*) &irMasterEEPROM) + 4, sizeof( irMasterEEPROM ) - 4 );
    if ( crc == irMasterEEPROM.crc32 ) {
      Serial.print("loaded\n\n");

      Serial.printf("Count of saved IR codes: %i\n", irMasterSetting->irCodeCount);
      uint8_t irCodeIndex = irMasterSetting->singleClickIRCodeId;
      if (irCodeIndex < (IR_CODE_COUNT + 1)) Serial.printf("Single click associated with IR code index %i name %s\n", irCodeIndex, irCodes[irCodeIndex].irCodeName);
      irCodeIndex = irMasterSetting->doubleClickIRCodeId;
      if (irCodeIndex < (IR_CODE_COUNT + 1)) Serial.printf("Double click associated with IR code index %i name %s\n", irCodeIndex, irCodes[irCodeIndex].irCodeName);

      for (uint8_t i = 0; i < IR_CODE_COUNT; i++) {
        if (irCodes[i].irCodeLength > 0) {
          dumpIRCode(&irCodes[i], true);
        }
      }
    } else {
      Serial.print("CRC32 error, resetting EEPROM\n\n");
      resetEEPROM();
    }
  } else {
    Serial.println(F("EEPROM size changed - EEPROM data zeroed - commit() to make permanent"));
    resetEEPROM();
  }

  // Start WiFiManager, that handles connection to the WIFI network
  Serial.println(F("Connecting to WIFI"));
  WiFi.hostname(ESP_HOSTNAME);
  WiFiManager wm;
  wm.setAPCallback(startWMConfig);           // Use this if you need to do something when your device enters configuration mode on failed WiFi connection attempt. Before autoConnect()
  wm.setSaveConfigCallback(endWMConfig);     // This gets called when custom parameters have been set AND a connection has been established.
  // IP parameters of WIFI network when WIfi Manager starts in AP mode - custom ip /gateway /subnet configuration
  wm.setAPStaticIPConfig(IPAddress(192, 168, 0, 1), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0)); // This will set your captive portal to a specific IP should you need/want such a feature. Add the following snippet before autoConnect()
  // set IP after logon to wifi (instead of DHCP) - custom ip /gateway /subnet configuration
  //wifiManager.setSTAStaticIPConfig(IPAddress(192,168,0,99), IPAddress(192,168,0,1), IPAddress(255,255,255,0));

  // Connect to WIFI
  // If there are no WIFI access information in the memory or the board is out of reach of saved WIFI
  // start own AP and the user must connect to this AP and sets up new WIFI connection
  if (!wm.autoConnect(ESP_HOSTNAME)) { // name of the AP WIFI, wifiManager.autoConnect("AutoConnectAP", "password")
    Serial.print(F("Could not connect to WIFI"));
    ESP.reset();                       // If something fails, restart ESP8266
    delay(1000);
  }
  Serial.print(F("Connected as: "));
  Serial.print(WiFi.localIP());
  Serial.printf("\n\n");

  // Initialize SPIFFS
  Serial.print(F("Mounting SPIFFS"));
  if (!SPIFFS.begin()) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.reset();                       // If something fails, restart ESP8266
    delay(1000);
  }
  Serial.printf("... mounted\n\n");

  // Start web server that will send the client documents from SPIFFS
  Serial.print(F("Starting webserver"));

  // ==================== REST API - all/fire ====================
#ifdef USE_REST_API
  server.on("/api", []() {
    digitalWrite(LED_BUILTIN, LOW);
    if (server.args() > 0) {
      int16_t response_code = 200;
      // 200 OK
      // 204 No Content
      // 400 Bad Request
      yield();

      if (server.hasArg("all")) {
        // ###########################
        // ## all
        // ###########################
        Serial.println(F("Received *all* GET request"));
        //{rsT:"all", rsS:"OK", rsE:"",rsV:[{id:0,name:"TV on"},{id:1,name:"TV off"}]};
        allResponse(false);
        if (irMasterSetting->irCodeCount == 0) response_code = 204;
        Serial.printf("*all* GET response: %s\n", output.c_str());
        server.send(response_code, "application/json", output);

      } else if (server.hasArg("fire") && server.arg("fire") != NULL) {
        // ###########################
        // ## fire
        // ###########################
        Serial.println(F("Received *fire* GET request"));
        uint8_t irCodeIndex = server.arg("fire").toInt();
        //{rsT:"fire", rsS:"OK", rsE:"",rsV:9};
        uint8_t fireResult = sendIRCode(irCodeIndex);
        fireResponse(fireResult, irCodeIndex);
        if (fireResult != 1)response_code = 204;
        Serial.printf("*fire* GET response: %s\n", output.c_str());
        server.send(response_code, "application/json", output);

      } else {
        Serial.println(F("Received *UNKNOWN* GET request"));
        response_code = 400;
        server.send(response_code, "text/html", "ERROR - Unknown API command, supported: ?all;?fire=x"); // otherwise, respond with a 404 (Not Found) error
      }
    }
    digitalWrite(LED_BUILTIN, HIGH);
  });
#endif

  server.onNotFound([]() {                              // If the client requests any URI
    digitalWrite(LED_BUILTIN, LOW);
    if (!handleFileRead(server.uri()))                  // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
    digitalWrite(LED_BUILTIN, HIGH);
  });
  server.begin();
  Serial.printf("... started\n\n");

  // Start websocket process
  Serial.print(F("Websocket starting"));
  // when someone connect to the websocket, the function websocketProcess will handle the request
  websocket_server.onEvent(websocketProcess);
  websocket_server.begin();
  Serial.printf("... started\n\n");

#ifdef USE_MQTT
  Serial.print(F("MQTT starting"));
  mqttConnect();
  Serial.printf("... started\n\n");
#endif


  Serial.print(F("Set up IR sending"));
  irsend.begin();       // Start up the IR sender.
  Serial.printf(" ... done\n");

  // initialize global variable
  resetIRCode(&irCode);
}


// ==================== LOOP ====================
void loop() {
  // Handle HTTP communication
  server.handleClient();
  yield();
  // Hanlde websocket communication
  websocket_server.loop();
  yield();

#ifdef USE_MQTT
  if (!mqtt_client.connected()) {
    mqttConnect();
  }
  mqtt_client.loop();
#endif

  // Read hardware pin, convert to click events
  myButton.update();
  if ((myButton.isSingleClick()) || (myButton.isDoubleClick())) {
    processClick((myButton.isSingleClick()) ? BUTTON_SINGLE_CLICK : BUTTON_DOUBLE_CLICK);
  }

  yield();
}
/*
  unsigned long last_10sec = 0;
  unsigned int counter = 0;

  void loop() {
    unsigned long t = millis();
    webSocket.loop();
    server.handleClient();

    if((t - last_10sec) > 10 * 1000) {
        counter++;
        bool ping = (counter % 2);
        int i = webSocket.connectedClients(ping);
        USE_SERIAL.printf("%d Connected websocket clients ping: %d\n", i, ping);
        last_10sec = millis();
    }
  }
*/
