//#include <WiFiType.h>
//#include <WiFiSTA.h>
//#include <WiFiScan.h>
//#include <WiFiMulti.h>
//#include <WiFiGeneric.h>
//#include <WiFiAP.h>
#include <ESP32Servo.h>
#include <WiFi.h>
//#include <ETH.h>
#include <HardwareSerial.h>
#include <Ticker.h>
//#include <WiFiUdp.h>
//#include <WiFiServer.h>
//#include <WiFiClientSecure.h>
//#include <WiFiClient.h>



/*
 * luckylaker.ino
 *
 * Created: 8/26/2018 6:18:12 PM
 * Author: User
 */
#define DEBUGPRINT(...)				Serial.print(__VA_ARGS__)
#define DEBUGPRINTLN(...)			Serial.println(__VA_ARGS__)
#define DEBUGPRINTF(...)			Serial.printf(__VA_ARGS__)

#define PPM_PIN		15	//PPM input pin
#define LED			2	//Builti in LED

#define PPM_CHANNELS_NUMBER		10 //number PPM of channels to read
#define PPM_BUFFER_LEN			PPM_CHANNELS_NUMBER * 2 + 1
#define PPM_SEPARATOR_DURATION	9000	//PPM frame separator duration

#define PPM_NEUTRAL_POS			1500
#define PPM_MAX_POS				2050
#define PPM_MIN_POS				950
#define PPM_TOLERANCE			10

#define CAM_WINCH_CHANNEL		5
#define CAM_WINCH_PIN			5
#define CAM_WINCH_UP			110
#define	CAM_WINCH_DOWN			80
#define	CAM_WINCH_STOP			90
#define CAM_ENDSTOP_PIN			4

#define NETWORK_NAME "FishFinder1"
#define NETWORK_PASSWORD "12345678"
#define TCP_HOST "10.10.100.254"
#define TCP_PORT 8899
#define SENSITIVITY 5

#define START_BYTE 0x68
#define START_BYTE_BROKEN_DATA 0xFF
#define END_BYTE 0xED
#define HANDSHAKE_BYTE 0x01
#define REQUEST_DATA_BYTE 0x02
#define ACTIVATE_BYTE 0X03
#define SET_SENSITIVITY_BYTE 0X04
 //#define ACTIVE_RESPONSE_LEN 
#define INACTIVE_RESPONSE_LEN 6


typedef struct __attribute__((__packed__)) {
	boolean deviceActive;
	float depth;
	float temperature;
	float largeFishDepth;
	float mediumFishDepth;
	float smallFishDepth;
	uint8_t batteryLevel;
} FishFinderData;

//WiFiEventHandler connectHandler, disconnectHandler;
WiFiClient tcpClient;

Ticker blinker;
Ticker statusReporter;
//SoftwareSerial softwareSerial(SW_SERIAL_UNUSED_PIN, 15);

boolean flagWiFiConnected = false;
boolean flagWiFiConnecting = false;
boolean flagTcpClientConnected = false;
boolean flagActivated = false;
boolean flagReportStatus = false;

byte mac[6];
FishFinderData fishFinderData;

//PPM variables
unsigned long pulseStart, pulseEnd;
volatile unsigned int tmp[PPM_BUFFER_LEN], channels[PPM_CHANNELS_NUMBER];
uint8_t tmpPos = 0;

volatile boolean camAtTop = true;

Servo servoCamWinch;

void printArray(byte* arr, byte size) {
	for (int i = 0; i < size; i++) {
		DEBUGPRINTF("%02X ", arr[i]);
	}
}

String getMacString() {
	String s = WiFi.macAddress();
	DEBUGPRINT(s); DEBUGPRINT(" -> ");
	String mac = s.substring(0, 2) + s.substring(3, 5) + s.substring(6, 8) + s.substring(9, 11) + s.substring(12, 14) + s.substring(15, 17);
	DEBUGPRINTLN(mac);
	return mac;
}

void sendCommand(byte cmd, byte* payload, byte payloadLen, byte* response, byte* responseLen) {
	byte command[32];
	command[0] = START_BYTE;
	command[1] = cmd;
	command[2] = payloadLen;
	memcpy(command + 3, payload, payloadLen);
	command[3 + payloadLen] = calculateCrc(command, payloadLen + 3); //calculate from bytes 1, 2 and payload
	command[4 + payloadLen] = END_BYTE;
	DEBUGPRINT("Sending: "); printArray(command, 5 + payloadLen); DEBUGPRINTLN();

	tcpClient.setNoDelay(true);
	tcpClient.write((const uint8_t*)command, payloadLen + 5);
	//tcpClient.flush();


	while (tcpClient.available() == 0) {
		yield();
	}

	int pos = 0;
	while (int a = tcpClient.available()) {
		DEBUGPRINTF("Read %d bytes\n", a);
		tcpClient.read(response + pos, a);
		pos += a;
	}
	*responseLen = pos;
	DEBUGPRINT("Response: "); printArray(response, pos); DEBUGPRINTLN("\n");
	byte c = calculateCrc(response, *responseLen - 2);
	if (c != response[*responseLen - 2]) {
		DEBUGPRINTLN("CRC mismatch");
		response[0] = START_BYTE_BROKEN_DATA;
	}
}

byte calculateCrc(byte* data, byte dataLen) {
	byte crc = 0;
	for (int i = 1; i < dataLen; i++) { //exclude first byte
		crc = (byte)(crc + data[i]);
	}
	return crc;
}

boolean parseResponse(byte* response, byte responseLen, byte* payload, byte* payloadLen) {
	if (response[0] != START_BYTE || response[responseLen - 1] != END_BYTE) {
		DEBUGPRINTLN("Incorrect response format");
		return false;
	}
	*payloadLen = response[2];
	if (responseLen != *payloadLen + 5) {
		DEBUGPRINTLN("Incorrect payload length");
		return false;
	}
	byte crc = response[3 + *payloadLen];
	if (crc != calculateCrc(response, responseLen - 2)) {
		DEBUGPRINTLN("CRC mismatch");
		return false;
	}
	memcpy(payload, response + 3, *payloadLen);
	return true;
}

void activateFishFinder() {
	DEBUGPRINTLN("Activate:");
	byte response[32];
	byte responseLen = 0;
	sendCommand(ACTIVATE_BYTE, mac, 6, response, &responseLen);
}

void handshakeFishFInder() {
	DEBUGPRINTLN("Handshake:");
	byte response[32];
	byte responseLen = 0;
	sendCommand(HANDSHAKE_BYTE, mac, 6, response, &responseLen);
}

void requestFishFinderData() {
	DEBUGPRINTLN("Request data:");
	byte request[1] = { 1 };
	byte response[32];
	byte responseLen = 0;
	sendCommand(REQUEST_DATA_BYTE, request, 1, response, &responseLen);
	parseFishFinderData(response, responseLen);
}

bool setSensitivity(byte sensitivity) {
	DEBUGPRINTLN("Set sensitivity: ");
	if (sensitivity < 1 || sensitivity > 5) {
		DEBUGPRINTLN("Incorrect sensitivity. Must be 1..6");
		return false;
	}
	byte response[32];
	byte responseLen = 0;
	sendCommand(SET_SENSITIVITY_BYTE, &sensitivity, 1, response, &responseLen);
	byte payload[16];
	byte payloadLen = 0;
	parseResponse(response, responseLen, payload, &payloadLen);
	if (payload[0] != sensitivity) {
		DEBUGPRINTLN("Failed");
		return false;
	}
	else {
		DEBUGPRINTLN("Success");
		return true;
	}
}

void blink() {
	boolean state = digitalRead(LED);
	digitalWrite(LED, !state);
	DEBUGPRINT(".");
}

void reportFishFinderData() {
	DEBUGPRINTF("Sizeof struct: %d\n", sizeof(fishFinderData));
	//Serial2.println("Report");
	Serial2.write((uint8_t*)&fishFinderData, sizeof(fishFinderData));
	Serial2.write('\r');
	//Serial2.println("======");
}

void parseFishFinderData(byte* response, uint8_t responseLen) {
	if (response[0] == START_BYTE_BROKEN_DATA) {
		return;
	}

	if (responseLen == INACTIVE_RESPONSE_LEN || (response[3] == 0x7D && response[4] == 0x07 && response[5] == 0xD0)) { //Device not in water(?)
		fishFinderData.deviceActive = false;
		fishFinderData.depth = 0;
		fishFinderData.temperature = 0;
		fishFinderData.largeFishDepth = 0;
		fishFinderData.mediumFishDepth = 0;
		fishFinderData.smallFishDepth = 0;
		fishFinderData.batteryLevel = 0;
	}
	else {
		fishFinderData.deviceActive = true;
		fishFinderData.depth = (((int)response[3] << 4) + ((int)response[4] >> 4)) / 10.00f;
		fishFinderData.temperature = ((((int)((byte)(response[4] << 4))) << 4) + response[5]) / 10.00f;
		fishFinderData.largeFishDepth = response[6] * fishFinderData.depth / 255.00f;
		fishFinderData.mediumFishDepth = response[7] * fishFinderData.depth / 255.00f;
		fishFinderData.smallFishDepth = response[8] * fishFinderData.depth / 255.00f;
		fishFinderData.batteryLevel = response[9];
	}

	DEBUGPRINTF("Active: %d\n", fishFinderData.deviceActive);
	DEBUGPRINTF("Depth: %03X -> ", ((int)response[3] << 4) + ((int)response[4] >> 4)); DEBUGPRINTLN(fishFinderData.depth, 2);
	DEBUGPRINTF("Temperature: %03X -> ", (((int)((byte)(response[4] << 4))) << 4) + response[5]); DEBUGPRINTLN(fishFinderData.temperature, 2);
	DEBUGPRINTF("Large fish depth: %02X -> ", response[6]); DEBUGPRINTLN(fishFinderData.largeFishDepth, 2);
	DEBUGPRINTF("Medium fish depth: %02X -> ", response[7]); DEBUGPRINTLN(fishFinderData.mediumFishDepth, 2);
	DEBUGPRINTF("Small fish depth: %02X -> ", response[8]); DEBUGPRINTLN(fishFinderData.smallFishDepth, 2);
	DEBUGPRINTF("Battery level: %d", fishFinderData.batteryLevel);
	DEBUGPRINTLN("\n");

	reportFishFinderData();
}

void onWiFiConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
	DEBUGPRINTLN();
	DEBUGPRINTLN("WIFI connected");
	flagWiFiConnected = true;
	flagWiFiConnecting = false;
	blinker.detach();
}

void onWiFIDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
	if (!flagWiFiConnecting) {
		DEBUGPRINTLN("WIFI disconnected");
		flagWiFiConnected = false;
		flagTcpClientConnected = false;
		flagActivated = false;
		digitalWrite(LED, HIGH);
		ESP.restart();
	}
}

void wifiConnect() {
	DEBUGPRINT("WIFI connecting");
	WiFi.disconnect(true);
	flagWiFiConnecting = true;
	blinker.attach_ms(500, blink);
	WiFi.begin(NETWORK_NAME, NETWORK_PASSWORD);
}


void getMac() {
	String localMac = WiFi.macAddress();
	byte pos = 0;
	byte i = 0;
	while (pos < localMac.length()) {
		//DEBUGPRINTLN(localMac.substring(pos, pos+2));
		mac[i] = strtol(localMac.substring(pos, pos + 2).c_str(), 0, 16);
		//DEBUGPRINTF("%02X\n", mac[i]);
		pos += 3;
		i++;
	}
	//stringToByteArray(WiFi.macAddress().c_str(), mac);
	//DEBUGPRINTLN(WiFi.macAddress());
	DEBUGPRINT("Local MAC: "); printArray(mac, 6); DEBUGPRINTLN();
}

void readSerial() {
	if (Serial.available()) {
		char c;
		c = Serial.read();
		executeCommand(c);
	}
}

void executeCommand(char c) {
	DEBUGPRINTLN("ExecuteCommand");
	switch (c) {
	case 's': //report status
		break;
	case 'd':
		requestFishFinderData();
	}
}

void IRAM_ATTR ppm_handler() {
	unsigned int duration = 0;
	pulseEnd = micros();
	duration = pulseEnd - pulseStart;
	pulseStart = pulseEnd;

	tmp[tmpPos] = duration;

	if (duration > PPM_SEPARATOR_DURATION && tmpPos > PPM_CHANNELS_NUMBER) { //found frame separator and there is enough data before it
		for (uint8_t i = 0; i < PPM_CHANNELS_NUMBER; i++) { //copy data to channels array
			channels[i] = tmp[tmpPos - PPM_CHANNELS_NUMBER + i];
		}
		tmpPos = 0;
	}
	else {
		tmpPos++;
	}

	if (tmpPos == PPM_BUFFER_LEN) {
		tmpPos = 0;
	}
}

void IRAM_ATTR cam_endstop_handler() {
	if (digitalRead(CAM_ENDSTOP_PIN) == LOW) {
		camAtTop = true;
		//stop_cam();
	}
	else {
		camAtTop = false;
	}
}

void print_ppm() {
	for (uint8_t i = 0; i < PPM_CHANNELS_NUMBER; i++){
		DEBUGPRINT(channels[i]);DEBUGPRINT("   ");
	}
	DEBUGPRINTLN();
}

void process_cam_winch() {
	if ((channels[CAM_WINCH_CHANNEL] >= PPM_NEUTRAL_POS - PPM_TOLERANCE) && (channels[CAM_WINCH_CHANNEL] <= PPM_NEUTRAL_POS + PPM_TOLERANCE)) {
		servoCamWinch.write(CAM_WINCH_STOP);
	} else if (channels[CAM_WINCH_CHANNEL] >= PPM_MAX_POS - PPM_TOLERANCE) { //switch is DOWN, lower cam
		servoCamWinch.write(CAM_WINCH_DOWN);
	} else if (channels[CAM_WINCH_CHANNEL] <= PPM_MIN_POS + PPM_TOLERANCE) { //switch is UP, raise cam
		if (camAtTop) {
			servoCamWinch.write(CAM_WINCH_STOP);
		}
		else {
			servoCamWinch.write(CAM_WINCH_UP);
		}
	}
}

void setup()
{
	Serial.begin(115200);
	Serial2.begin(115200, SERIAL_8N1);
	DEBUGPRINTLN();
	DEBUGPRINTLN("Starting...");
	//Serial2.println("Starting...");

	pinMode(LED, OUTPUT);
	pinMode(PPM_PIN, INPUT_PULLDOWN);
	pinMode(CAM_ENDSTOP_PIN, INPUT_PULLUP);
	camAtTop = !digitalRead(CAM_ENDSTOP_PIN); //LOW = at top
	attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppm_handler, RISING);
	attachInterrupt(digitalPinToInterrupt(CAM_ENDSTOP_PIN), cam_endstop_handler, CHANGE);

	servoCamWinch.attach(CAM_WINCH_PIN);

	WiFi.persistent(false);
	WiFi.onEvent(onWiFiConnected, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
	WiFi.onEvent(onWiFIDisconnected, WiFiEvent_t::SYSTEM_EVENT_STA_LOST_IP);
	WiFi.mode(WIFI_STA);
	getMac();
}

void loop()
{
	process_cam_winch();
	yield();

	if (!flagWiFiConnected) {
		if (!flagWiFiConnecting) {
			wifiConnect();
		}
	}
	else {
		if (!flagTcpClientConnected) {
			if (tcpClient.connect(TCP_HOST, TCP_PORT)) {
				DEBUGPRINTLN("TCP client connected to server");
				flagTcpClientConnected = true;
				digitalWrite(LED, LOW);
				delay(5000);
				activateFishFinder();
				//delay(1000);
				handshakeFishFInder();
				//delay(1000);
				flagActivated = setSensitivity(SENSITIVITY);
			}
			else {
				DEBUGPRINTLN("TCP client failed to connect to server");
				flagTcpClientConnected = false;
			}
		}
	}

	if (flagActivated) {
		readSerial();
	}
}