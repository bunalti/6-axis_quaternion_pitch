#include <Arduino.h>

//#include "WifiConfig.h"

#include <ESPNtpClient.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#ifndef WIFI_CONFIG_H

#define YOUR_WIFI_SSID "SUPERONLINE_WiFi_7815"
#define YOUR_WIFI_PASSWD "NP9HFHXHJH7M"
#endif // !WIFI_CONFIG_H

#define SHOW_TIME_PERIOD 1000

String formattedDate;
String dayStamp;
String timeStamp;


void setup() {
    Serial.begin (115200);
    Serial.println ();
    WiFi.begin (YOUR_WIFI_SSID, YOUR_WIFI_PASSWD);
    NTP.setTimeZone (TZ_Europe_Istanbul );
    NTP.begin ();
}

void loop() {
    
    formattedDate = NTP.getTimeDateStringUs();
    int splitT = formattedDate.indexOf(" ");
    timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-4);
    Serial.println (timeStamp);
    
}
