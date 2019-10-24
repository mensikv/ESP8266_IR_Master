# ESP8266 IR Master
The project is for ESP8266 boards and utilizes IR LEDs and IR receiver. Thanks to the IR receiver, you can scan various IR codes (from tv remotes, air-conditionings, lights...), save them to EEPROM on the ESP8266 and later send them through the IR LEDs.
Webserver runs on ESP8266 that provides interface for scanning and sending of IR codes. REST and MQTT API can also be enabled.
Project can be used in home automation systems for devices, that are controlled with IR remotes.
**WIFI networks is required**.

This project was created for workshop on [Mini Maker Fair Brno 2019](https://brno.makerfaire.com/).

## Prerequisites
### Hardware
It is possible to use any ESP8266 board and some IR LEDs and IR receiver, but I would recommend following combination:
* [Wemos (Lolin) D1 Mini](https://lolin.aliexpress.com/store/group/D1-D1-mini-Boards/1331105_505460007.html?spm=a2g0o.detail.0.0.57c7637a1P3kJY) (non-affiliated link)
* [Wemos (Lolin) IR shield](https://www.aliexpress.com/item/32891173618.html?spm=2114.12010612.8148356.1.32522730sTPRit) (non-affiliated link)

### Software
Project was tested with following versions:
* Arduino IDE 1.8.10
* ESP8266 core 2.5.2
* [ESP8266 Sketch Data Upload](https://github.com/esp8266/arduino-esp8266fs-plugin)

Libraries:
* [ArduinoJSON 6.12.0](https://github.com/bblanchon/ArduinoJson)
* [ESP_EEPROM 2.0.0](https://github.com/jwrw/ESP_EEPROM)
* [IRRemoteESP8266 2.6.6](https://github.com/crankyoldgit/IRremoteESP8266)
* [WifiManages 0.15.0-beta](https://github.com/tzapu/WiFiManager)
* [arduinoWebSockets 2.1.1](https://github.com/Links2004/arduinoWebSockets)
* [PubSubClient 2.7](https://github.com/knolleary/pubsubclient)
* [PinButton](https://github.com/poelstra/arduino-multi-button)

## Sending IR codes with a button
You can use a button to send selected IR codes, single click and double click is supported.
If you use Wemos D1 Mini with [IR shield](https://wiki.wemos.cc/products:d1_mini_shields:ir_controller_shield), use these steps:
1. Solder a button on the IR shield between pins D3 and GND.
1. Since D3 is by default used for IR LEDs, you must cut a trace on the back of the IR shield and join two solder pads - I have used pin D2 for IR LEDs.

## Setup
1. Download content of this repository to your Arduino folder (probably `~\Documents\Arduino\ESP8266_IR_Master`, folder should contain one `*.ino` file and `data` folder) and open `ESP8266_IR_Master.ino`.
1. Set pin for sending IR signals `const uint16_t kIrLedPin = D2;`
1. Set pin for receiving IR signals `const uint16_t kRecvPin = D4;`
1. You can set a hostname by changing `#define ESP_HOSTNAME "IRMASTER001"`
1. If you want to use button, set the pin, where button is connected: `#define BUTTON_PIN D3`
1. If you want to enable REST API (sending IR codes with GET requests), uncomment `//#define USE_REST_API`
1. If you want to enable MQTT API (sending IR codes with MQTT messages), uncomment `//#define USE_MQTT`
   1. In section `#ifdef USE_MQTT` set connection to your MQTT broker and topics, you want to use.
   1. You should increase `MQTT_MAX_PACKET_SIZE` in PubSubClient.h. For 5 IR codes is 512 sufficient - `#define MQTT_MAX_PACKET_SIZE 512`
1. If you want to assign static IP address to the board in your WIFI network, uncomment and set following line `//wifiManager.setSTAStaticIPConfig(IPAddress(192,168,0,99), IPAddress(192,168,0,1), IPAddress(255,255,255,0));`
1. Choose the right board and select Flash size with at least 128kB SPIFFS, on Wemos D1 Mini I have used 4M (1M SPIFFS)
1. Upload the code.
1. Run ESP8266 Sketch Data Upload to upload content of `/data` folder to SPIFFS on ESP8266. Great article is [Chapter 11 - SPIFFS](https://tttapa.github.io/ESP8266/Chap11%20-%20SPIFFS.html). ESP8266 Sketch Data Upload plug-in to load data on SPIFFS can be downloaded from [here](https://github.com/esp8266/arduino-esp8266fs-plugin). `/data` folder contains html code, CSS styles and JavaScript code for the webserver.
   1. Before stating upload to SPIFFS make sure, that Serial monitor window is not opened, otherwise the upload will fail.
1. Now the ESP8266 will start in AP mode (onboard LED blinks) and creates WIFI network with the name you set as hostname. Connect to this network and you should be redirected to URL 192.168.0.1 (if not, enter this address in your browser). On this page set up WIFI connection to your network. ESP8266 will try to connect to it, if it succeeds the onboard LED stops blinking.
1. Either check your router or Serial monitor to find out, what IP address the ESP8266 got. Enter this address in your browser.
1. You enter a web interface with 3 tabs, 1st tab is used to send IR codes (it is empty, until you scan and save some IR codes). 2nd tab is used to scan new IR codes (click Scan button and then you have 5seconds to fire IR signal on the IR receiver. If successful, you can name it and save), delete some IR codes and also assign some codes to click or double click of the button. 3rd tab is short help.

## REST API
If you enable REST API, then the ESP8266 will provide two new URLs:
* http://IPADDRESS/api?all
  * returns JSON with saved IR codes and their IDs
* http://IPADDRESS/api?fire=IR_ID
  * sends IR signal with id IR_ID and returns JSON with success or error

## MQTT API
If you enable MQTT API, then the ESP8266 will wait for commands in topic:
* MQTT_TOPIC/cmd with payloads
  * "all" - returns JSON with saved IR codes in topic MQTT_TOPIC/all
  * "fire IR_ID" - sends IR signal with id IR_ID and returns JSON with success or error in topic MQTT_TOPIC/fire

## STL for case
In folder `case` you will find `*.stl` files for your 3D printer, so you can print nice case for Wemos D1 Mini with IR shield.
More information about the case and photos can be found on [Thingiverse](https://www.thingiverse.com/thing:3930823).
