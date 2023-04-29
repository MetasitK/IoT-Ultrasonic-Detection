// ----------------- Import Library --------------------
#include <WiFi.h>
#include <time.h>
#include <DS3231.h>
#include <Wire.h>
#include <PubSubClient.h>

#include "config.h"

// Config format
/*
    // NodeRED MQTT server
    const char* noderedServer = "";
    const int noderedMQTTPort = 1883;
    
    // NetPie MQTT Server 
    const char* netpieServer = "broker.netpie.io";
    const int netpiePort = 1883;
    const char* netpieClientID = ""; //Client ID
    const char* netpieUser = ""; //Token
    const char* netpiePassword = ""; //Secret
    
    // (WiFi) Variables
    char ssid[] = ""; 
    char pass[] = ""; 
*/

// -----------------------------------------------------

// ------------------- DEFINE PIN ----------------------
#define LED 2

#define ULTRA_ECHO 19
#define ULTRA_TRIG 23 

#define PUSH_BUTTON 13
// -----------------------------------------------------

// ------------------- For Wifi -----------------------
bool wifiConnected = true;
// -----------------------------------------------------

// ------------------- For RTC ---------------------------
const char* ntpServer = "th.pool.ntp.org";
const long  gmtOffset_sec = 3600 * 7; //UTC +7.00
const int   daylightOffset_sec = 0; //0 means no DST observed; otherwise, 3600.

DS3231  rtc;

bool h12Format;
bool ampm;
bool centuryRollover;

struct tm timeinfo;
String dateTimeString;
// -----------------------------------------------------

// ------------------- For MQTT -----------------------
WiFiClient noderedClient;
PubSubClient noderedclient(noderedClient);

WiFiClient netpieClient;
PubSubClient netpieclient(netpieClient);
bool netpieFrag = false;
long now, lastMSG = 0;
// -----------------------------------------------------

// -------------------- For FSM -----------------------
#define STATE_OFF 0
#define STATE_ON 1

#define STATE_DETECTING 0
#define STATE_DETECTED 1
#define STATE_ERROR 2

#define STATE_CHANGE_ALLOWED_ERROR_CM 2
#define STATE_CHANGE_THRESHOLD_CM 10
float duration_us, distance_cm;
float initialUltrasonicDistanceCM;

int mainState = STATE_OFF; 
int ultrasonicState = STATE_DETECTING;

// toggleMainState: For detemining whether the ultrasonic should be active or not
//   Purpose: Reduce power consumption
void IRAM_ATTR toggleMainState()
{
  mainState = (mainState == STATE_ON)?STATE_OFF:STATE_ON;
  digitalWrite(LED, LOW);
  ultrasonicState = STATE_DETECTING;
}
// -----------------------------------------------------

void setup() {

  Serial.begin(115200);
  // ------------------------ Set-up Wifi -----------------------
  Wire.begin();
  //connect to WiFi
  Serial.printf("Connecting to %s ", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println(" CONNECTED");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  // ------------------------------------------------------------------
  
  // ------------------------ Set-up RTC -----------------------
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  rtc.enableOscillator(true, true, 1);
  rtc.setClockMode(h12Format); //24-h format
  rtc.setDoW(timeinfo.tm_wday);
  rtc.setHour(timeinfo.tm_hour);
  rtc.setMinute(timeinfo.tm_min);
  rtc.setSecond(timeinfo.tm_sec);
  rtc.setDate(timeinfo.tm_mday);
  rtc.setMonth(timeinfo.tm_mon + 1); //Month from NTP starts from zero
  rtc.setYear(timeinfo.tm_year - 100); //Year from NTP is an offset from 1900
  // ------------------------------------------------------------------
  
  // --------------------------- Set-up Pins---------------------------
  pinMode(LED, OUTPUT);
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  
  pinMode(ULTRA_ECHO, INPUT);
  pinMode(ULTRA_TRIG, OUTPUT);
  // ------------------------------------------------------------------
  
  // -------------------------- Set-up Interupt------------------------
  attachInterrupt(PUSH_BUTTON, toggleMainState, FALLING);
  // ------------------------------------------------------------------
  
  // -------------------------- Set-up MQTT---------------------------
  noderedclient.setServer(noderedServer, noderedMQTTPort); //mqtt server and port
  netpieclient.setServer(netpieServer, netpiePort); //mqtt server and port
  // ------------------------------------------------------------------
  
  // ------------------ Initiate Ultrasonic Distance ------------------
  digitalWrite(ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG, LOW);
      
  duration_us = pulseIn(ULTRA_ECHO, HIGH);
  initialUltrasonicDistanceCM = 0.017 * duration_us;
  if(initialUltrasonicDistanceCM < STATE_CHANGE_THRESHOLD_CM){
      ultrasonicState = STATE_ERROR;
  }
  // ------------------------------------------------------------------
}

void loop() {

  if (!noderedclient.connected()) {
    reconnectNodeRed();
  }
  noderedclient.loop();

  if (!netpieclient.connected()){
    netpieReconnect();
  }
  netpieclient.loop();
  
  dateTimeString = "20" + (String)rtc.getYear() + "/" + (String)rtc.getMonth(centuryRollover) + "/" + (String)rtc.getDate() + "---" + (String)rtc.getHour(h12Format, ampm) + "-" + (String)rtc.getMinute() + "-" + (String)rtc.getSecond();

  switch(mainState){
    case(STATE_OFF):
      break;
    case(STATE_ON):   
      Serial.print("Current Ultrasonic State: ");
      if(ultrasonicState == STATE_DETECTING){
        Serial.println("Detecting");
      }
      else if(ultrasonicState == STATE_DETECTED){
        Serial.println("Detected");
      }
      else if(ultrasonicState == STATE_ERROR){
        Serial.println("Error");
      }
      
      Serial.print("Initial distance: " );
      Serial.print(initialUltrasonicDistanceCM);
      Serial.println(" cm");
    
      digitalWrite(ULTRA_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(ULTRA_TRIG, LOW);
      
      duration_us = pulseIn(ULTRA_ECHO, HIGH);
      distance_cm = 0.017 * duration_us;
      
      Serial.print("distance: " );
      Serial.print(distance_cm);
      Serial.println(" cm");
      
      switch(ultrasonicState){
        case(STATE_DETECTING):
          digitalWrite(LED,LOW);
          if(distance_cm < initialUltrasonicDistanceCM - STATE_CHANGE_THRESHOLD_CM){
            changeUltrasonicState(STATE_DETECTED);
          }
          else if(distance_cm > initialUltrasonicDistanceCM + STATE_CHANGE_ALLOWED_ERROR_CM){
            changeUltrasonicState(STATE_ERROR);
          }
          break;
        case(STATE_DETECTED):
          digitalWrite(LED,HIGH);
          if(distance_cm >= initialUltrasonicDistanceCM - STATE_CHANGE_ALLOWED_ERROR_CM && distance_cm <= initialUltrasonicDistanceCM + STATE_CHANGE_ALLOWED_ERROR_CM){
            changeUltrasonicState(STATE_DETECTING);
          }
          else if(distance_cm > initialUltrasonicDistanceCM + STATE_CHANGE_ALLOWED_ERROR_CM){
            changeUltrasonicState(STATE_ERROR);
          }
          break;  
        case(STATE_ERROR):
          digitalWrite(LED,HIGH);
          delay(100);
          digitalWrite(LED,LOW);
          delay(100);
          if(distance_cm >= initialUltrasonicDistanceCM - STATE_CHANGE_ALLOWED_ERROR_CM && distance_cm <= initialUltrasonicDistanceCM + STATE_CHANGE_ALLOWED_ERROR_CM){
            changeUltrasonicState(STATE_DETECTING);
          }
          else if(distance_cm < initialUltrasonicDistanceCM - STATE_CHANGE_THRESHOLD_CM){
            changeUltrasonicState(STATE_DETECTED);
          }
          break;  
      }
      break;  
  }
  delay(100);
}

void changeUltrasonicState(int state){
  ultrasonicState = state;
  String message;
  String netpieOutput;
  switch(state){
    case STATE_DETECTING:
      message = "Detecting";
      break; 
    case STATE_DETECTED:
      message = "Detected";
      break; 
    case STATE_ERROR:
      message = "Error";
      break; 
  }
  netpieOutput = "{\"data\": {\"status\":\"" + message + "\", \"dateTime\":\"" + dateTimeString + "\"}}";
  Serial.println(netpieOutput);
  int lenNetPie = netpieOutput.length() + 1;
  char bufNetPie[lenNetPie]; 
  netpieOutput.toCharArray(bufNetPie, lenNetPie);
  netpieclient.publish("@shadow/data/update", bufNetPie);  
    
  message += "|" + dateTimeString;
  
  int lenNodeRED = message.length() + 1;
  char bufNodeRED[lenNodeRED]; 
  message.toCharArray(bufNodeRED,lenNodeRED);
  
  Serial.println(message);

  noderedclient.publish("P2/StateChange", bufNodeRED);
}

void reconnectNodeRed() {
  // Loop until we're reconnected
  while (!noderedclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (noderedclient.connect("esp32 Client")) {
      Serial.println("connected");
      // Subscribe
    } else {
      Serial.print("failed, rc=");
      Serial.print(noderedclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void netpieReconnect(){
  while(!netpieclient.connected()){
    Serial.println("Connecting to NetPie...");
    if (netpieclient.connect(netpieClientID, netpieUser, netpiePassword )) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(netpieclient.state());
      delay(2000);
    }
  }
}
