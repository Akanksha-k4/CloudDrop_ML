/*
 * ESP32 MAVLink to WiFi Bridge
 * * This sketch creates a WiFi Access Point and forwards MAVLink data
 * between the Pixhawk's serial port and a computer connected via WiFi UDP.
 * * It's a transparent bridge, so it's very fast.
 */

#include "WiFi.h"
#include <WiFiUdp.h>

// --- Settings to Change ---
const char *ssid = "DroneBridge";     // The name of the WiFi network to create
const char *password = "dronebridge"; // The password for the WiFi network

IPAddress apIP(192, 168, 4, 1);       // Static IP of the ESP32
IPAddress netMsk(255, 255, 255, 0);

const int udp_port_local = 14550;   // Port your GCS (Pymavlink) will listen on
const int udp_port_remote = 14550;  // Port your GCS will send to
// --- End of Settings ---

// We will use Hardware Serial 2 (GPIO 16 & 17)
// RX2 = GPIO 16
// TX2 = GPIO 17
#define PIXHAWK_SERIAL Serial2

const uint32_t PIXHAWK_BAUD = 921600; // High baud rate for fast MAVLink v2
// Note: You must set your Pixhawk's TELEM2 port (e.g., SERIAL2_BAUD) to 921600 as well!

WiFiUDP udp;
bool udp_connected = false;

void setup() {
  // Start serial for debugging
  Serial.begin(115200);
  Serial.println("\nStarting MAVLink to WiFi Bridge...");

  // Start the serial port to the Pixhawk
  // (RX, TX, mode, buffer)
  PIXHAWK_SERIAL.begin(PIXHAWK_BAUD, SERIAL_8N1, 16, 17);

  // Start WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(ssid, password);

  Serial.print("WiFi AP '");
  Serial.print(ssid);
  Serial.println("' started.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start listening for UDP packets
 if (udp.begin(udp_port_local)) {
  Serial.print("UDP server started on port ");
  Serial.println(udp_port_local);
  udp_connected = true;
} else {
  Serial.println("UDP server failed to start!");
}

//  TEST PACKET: send once at startup to confirm UDP link
const char *test = "hello from ESP32";
udp.beginPacket("192.168.4.2", 14550); // your PC's IP
udp.write((const uint8_t*)test, strlen(test));
udp.endPacket();
Serial.println("Test UDP packet sent");
  }

void loop() {
  // 1. Check for data from Pixhawk (Serial) and send to WiFi (UDP)
 while (PIXHAWK_SERIAL.available()) {
  uint8_t buffer[256];
  int len = PIXHAWK_SERIAL.readBytes(buffer, sizeof(buffer));

  // Send it to the remote UDP port
  if (len > 0 && udp_connected) {
    udp.beginPacket(udp.remoteIP(), udp_port_remote);
    udp.write(buffer, len);
    udp.endPacket();

    }
  }

  // 2. Check for data from WiFi (UDP) and send to Pixhawk (Serial)
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    uint8_t packetBuffer[256];
    int len = udp.read(packetBuffer, sizeof(packetBuffer));

    if (len > 0) {
      PIXHAWK_SERIAL.write(packetBuffer, len);
    }
  }
}