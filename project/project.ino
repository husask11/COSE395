#include <Debug.h>
#include <JSN270.h>

#include <Debug.h>
#include <JSN270.h>
#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>


#define SSID      "ArduinoTest"      // your wifi network SSID
#define KEY       "HelloArduino"      // your wifi network password
#define AUTH       "WPA2"       // your wifi network security (NONE, WEP, WPA, WPA2)
#define USE_DHCP_IP 1
#define SERVER_PORT    80
#define PROTOCOL       "TCP"

#if !USE_DHCP_IP
#define MY_IP          "192.168.1.133"
#define SUBNET         "255.255.255.0"
#define GATEWAY        "192.168.1.254"
#endif

// PINS
int pin_servo = 9;
int pin_touch = 10;
int pin_LED_G = 12;
int pin_LED_R = 13;

// Values
int led_status = 0; // 0 = RED OFF, GREEN ON / 1 = RED ON, GREEN OFF
int door_status = 0;  // 0 = closed / 1 = open
String currentLine = "";  // make a String to hold incoming data from the client
int get_http_request = 0;
int touch_status = LOW; 

Servo servo;
SoftwareSerial mySerial(3, 2); // RX, TX
JSN270 JSN270(&mySerial);

void setup() {
   mySerial.begin(9600);
   Serial.begin(9600);

  pinMode(pin_touch, INPUT);
  pinMode(pin_servo, OUTPUT);
  pinMode(pin_LED_G, OUTPUT);
  pinMode(pin_LED_R, OUTPUT);

  digitalWrite(pin_servo, LOW);
  digitalWrite(pin_LED_G, LOW);
  digitalWrite(pin_LED_R, LOW);
  
  servo.attach(pin_servo);
  servo.write(0);
  delay(150);

   Serial.println("--------- KU Term Project --------");
  init_JSN();

  digitalWrite(pin_LED_R, HIGH);
}

void loop() {
   if (!JSN270.server(SERVER_PORT, PROTOCOL)) {
      Serial.println("Failed connect ");
      Serial.println("Restart System");
   } else {
      Serial.println("Waiting for connection...");
   }
      
   currentLine = "";                // make a String to hold incoming data from the client
   get_http_request = 0;

   while (1) {
      if (mySerial.overflow()) {
         Serial.println("SoftwareSerial overflow!");
      }
   
      if (JSN270.available() > 0) {
         char c = JSN270.read();
         Serial.print(c);

      // Main Page
         if (c == '\n') {                    // if the byte is a newline character
            if (currentLine.length() == 0) {
               if (get_http_request) {
            touch_status = digitalRead(pin_touch);  // Get Touch Status
                  Serial.println("new client");
                  JSN270.sendCommand("at+exit\r");
                  delay(100);
           
                  JSN270.println("HTTP/1.1 200 OK");
                  JSN270.println("Content-type:text/html");
                  JSN270.println();
            if(touch_status == HIGH)
              JSN270.print("TOUCHED!!<br>");
            else
              JSN270.print("NOT TOUCHED!!<br>");
                  JSN270.print("Click <a href=\"/H\">here</a> Open the Door<br>");
                  JSN270.print("Click <a href=\"/L\">here</a> Close the Door<br>");
                  JSN270.println();

                  // wait until all http response data sent:
                  delay(1000);

                  // Enter command mode:
                  JSN270.print("+++");
                  delay(100);
                  
                  // break out of the while loop:
                  break;
               }
            }
            // if you got a newline, then clear currentLine:
            else {                
               // Check the client request:
               if (currentLine.startsWith("GET / HTTP")) {
                  Serial.println("HTTP REQUEST");
                  get_http_request = 1;
               }
          // door Open
               else if (currentLine.startsWith("GET /H")) {
                  get_http_request = 1;
                  led_status = 1;
            door_status = 1;
            digitalWrite(pin_LED_G, HIGH);
            digitalWrite(pin_LED_R, LOW);
            servo.write(90);
               }
          // door Close
               else if (currentLine.startsWith("GET /L")) {
                  get_http_request = 1;
                  led_status = 0;
            door_status = 0;
            digitalWrite(pin_LED_G, LOW);
            digitalWrite(pin_LED_R, HIGH);
            servo.write(0);
               }
               currentLine = "";
            }
         }
         else if (c != '\r') {    // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
         }
      }
   }
   
   // close the connection
   JSN270.sendCommand("at+nclose\r");
   Serial.println("client disonnected");
}

void init_JSN(){
  char c;
  
  // wait for initilization of JSN270
  delay(5000);
  //JSN270.reset();
  delay(1000);

  //JSN270.prompt();
  JSN270.sendCommand("at+ver\r");
  delay(5);
  while(JSN270.receive((uint8_t *)&c, 1, 1000) > 0) {
    Serial.print((char)c);
  }
  delay(1000);

#if USE_DHCP_IP
  JSN270.dynamicIP();
#else
  JSN270.staticIP(MY_IP, SUBNET, GATEWAY);
#endif    
    
  if (JSN270.join(SSID, KEY, AUTH)) {
    Serial.println("WiFi connect to " SSID);
  }
  else {
    Serial.println("Failed WiFi connect to " SSID);
    Serial.println("Restart System");

    return;
  }    
  delay(1000);

  JSN270.sendCommand("at+wstat\r");
  delay(5);
  while(JSN270.receive((uint8_t *)&c, 1, 1000) > 0) {
    Serial.print((char)c);
  }
  delay(1000);        

  JSN270.sendCommand("at+nstat\r");
  delay(5);
  while(JSN270.receive((uint8_t *)&c, 1, 1000) > 0) {
    Serial.print((char)c);
  }
  delay(1000);
}