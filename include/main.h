#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <Servo.h>
#include <PID_v1.h>

// WiFi Definitions
const char *ssid = "TTNET_TP-LINK_B58D";
const char *password = "rsQTaSXY";
IPAddress server(192, 168, 1, 110); // Set the rosserial socket server IP address
const uint16_t serverPort = 11411;  // Set the rosserial socket server port

// Stop both motors with 0 pwm
void stop(void)
{
    analogWrite(D1, 0);
    analogWrite(D6, 0);
}

// Stop both motors with braking
void hardStop(void)
{
    digitalWrite(D2, HIGH);
    digitalWrite(D3, HIGH);
    digitalWrite(D4, HIGH);
    digitalWrite(D5, HIGH);
}

void setupWiFi()
{
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

#endif