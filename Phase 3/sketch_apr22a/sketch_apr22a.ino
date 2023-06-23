//Includes :
#include <DHT.h>
#include <DHT_U.h>
//WebServer :
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

//Definitions : 
//For Motor :
#define enca D2
#define encb D1 
#define IN2 D3
#define IN1 D5
const int PWMreal = 12;
volatile int POS = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//For DHT22 And WebServer :
const int refresh = 3; //WebServers Refreshes for New Data Every 3 Seconds.
#define DHTPIN D4
#define DHTTYPE DHT22
DHT dht(DHTPIN , DHTTYPE); //Introducing The DHT
float tValue; //Short for Temperature Value

//Enabling ESP8266:
#ifndef SSIDWiFi
#define SSIDWiFi "dcl_pi3" //WiFi SSID
#define PSWWiFi "dcl_pi_3" //WiFi PassWord
#endif

//Enabling WebServer:
ESP8266WebServer server(80); //We Use HTTP Port = 80

//Sending Temprature To the WebServer;
void SetTemp(){
  String page = "<!DOCTYPE html>\n\n";
  page += "<meta http-equiv='refresh' content'";
  page += String(refresh); //Remember The Refresh We Described Earlier ??!!
  page += "'/>\n";
  page += "<html>\n";
  page += "<body>\n";
  page += "<p style=\"font-size:50px;\">Temperature<br/>\n";
  page += "<p style=\"color:red; font-size:50px;\">";
  page += String(tValue , 2); //2 : 2 Decimal Place
  page += "</p>\n";
  page += "</body>\n";
  page += "</html>\n";
server.send(200 , "text/html" , page);
}

//Reading From Encoder : Updating Position
void ICACHE_RAM_ATTR readEncoder(){
  int b = digitalRead(encb);
  if(b>0){
    POS++;
  }
  else{
    POS--;
  }
}

//Control The Motor 
void SetMotor(int dir , int pwmVal , int pwm , int in1 , int in2){
  analogWrite(pwm , pwmVal);
  if(dir == 1){ //If dir = 1 Motor rotates one way
    digitalWrite(IN1 , HIGH);
    digitalWrite(IN2 , LOW);
    
  }
  else if(dir == -1){ // and if dir = -1 Motor rotates another way
    digitalWrite(IN1 , LOW);
    digitalWrite(IN2 , HIGH);
  }
  else{ //if dir is 0 Motor doesn't rotate (IN1 and IN2 could both be high for this purpose)
    digitalWrite(IN1 , LOW);
    digitalWrite(IN2 , LOW);
  }
}

//Main Setup
void setup(){
  //Setting Baud Rate
  Serial.begin(9600);
  //Updating Position:
  ICACHE_RAM_ATTR;
  attachInterrupt(digitalPinToInterrupt(enca) , readEncoder , RISING);
  //DHT : Enabling DHT
  dht.begin();
  //Enabling WiFi:
  WiFi.mode(WIFI_STA); //Wifi Station
  WiFi.begin(SSIDWiFi ,PSWWiFi);
  //Pin Modes:
  pinMode(enca , INPUT);
  pinMode(encb , INPUT);
  pinMode(IN2 , OUTPUT);
  pinMode(IN1 , OUTPUT);

  //Wait For Connection :
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected To : ");
  Serial.println(SSIDWiFi);
  Serial.print("IP Address : ");
  Serial.println(WiFi.localIP());
  //Enabling MDNS:
  if(MDNS.begin("RezaProject")){
    Serial.println("MDNS responder Started");
  }

  server.on("/" , SetTemp);
  server.begin();
  Serial.println("HTTP Server Started");
  
  
 
  
}




void loop(){
  server.handleClient();
  MDNS.update();
  float c = dht.readTemperature();
  float h = dht.readHumidity();
  tValue = c;
  //Serial.print("Temprature is : ");
  //Serial.println(c);
  //delay(300);
 
  //Set a Target :
  int target = 1200;

  //Set PID Variables:
  float kp = 1;
  float kd = 0;
  float ki = 0;

  long currT = micros();

  float deltaT = (float)(currT - prevT)/1.0e6;
  prevT = currT;

  int e = POS - target;

  float dedt = (e-eprev)/(deltaT);


 eintegral = eintegral + e*deltaT;

 //float u = target;
  float u = kp*e + kd*dedt + ki*eintegral;

 float pwr = fabs(u);
 if(pwr >255){
  pwr = 255;
 }

 int dir = 1;
 if(u<0){
  dir = -1;
 }

 SetMotor(dir,pwr,PWMreal,IN1,IN2);

 eprev = e;

 Serial.print(target);
 Serial.print(" ");
 Serial.print(POS);
 Serial.print(" ");
 Serial.print(pwr);
 Serial.println();
 

 

  
  
}
