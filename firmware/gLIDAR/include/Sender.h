#ifndef SENDER_H
#define SENDER_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiUdp.h>
#include <vector>

class Sender {
    std::vector<IPAddress> clients;
    WiFiUDP udpClient;
    int port;
public:
    Sender();
    void begin(String ssid, String password, int WiFiport);
    void checkIPs();
    void sendParameters(std::vector<uint8_t> data);
};

#endif