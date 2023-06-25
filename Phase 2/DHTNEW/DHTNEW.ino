#include <DHT.h>
#include <DHT_U.h>


#define DHTPIN 2

#define DHTTYPE DHT22

DHT dht(DHTPIN , DHTTYPE);




void setup() {
  Serial.begin(9600);


  dht.begin();

}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if(isnan(h) || isnan(t)){
    Serial.println("Failed to read from DHT");
  }

  else{
    Serial.print("Humidity : ");
    Serial.print(h);
    Serial.print("% ");
    Serial.print("Temperature : ");
    Serial.print(t);
    Serial.println(" Degrees Celcius");
  }

}
