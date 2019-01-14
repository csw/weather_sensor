// -*- c++ -*-

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15

#define SEALEVELPRESSURE_HPA (1013.25)

struct bme_reading
{
    float temperature;
    float pressure;
    float humidity;
};

//Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
WiFiUDP     udp;

const char* ssid       = "aje-csw";
const char* passphrase = "celeryairshipomitted";
char        host_string[16];
const int   local_port = 4000;

const char* service = "weather";
IPAddress   svc_addr;
uint16_t    svc_port;

bool        acked;
bme_reading reading;

char packet[256];

unsigned long delayTime = 20*1000;

void setup()
{
    Serial.begin(9600);
    Serial.println(F("BME280 test"));

    sensor_init();
    wifi_init();
    udp.begin(local_port);

    discover();

    Serial.println(F("Sensor node ready.\n"));
}


void loop()
{
    read_sensor();
    send_reading();
    Serial.print(packet);

    delay(delayTime);
}

void run_unacked()
{
    discover();
}

void wifi_init()
{
    WiFi.begin(ssid, passphrase);

    Serial.print(F("Connecting to WiFi"));
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    Serial.print(F("Connected, IP address: "));
    Serial.println(WiFi.localIP());

    sprintf(host_string, "ESP_%06X", ESP.getChipId());
    Serial.print("Hostname: ");
    Serial.println(host_string);

    if (!MDNS.begin(host_string)) {
        Serial.println("Error setting up MDNS responder!");
    }
    Serial.println("mDNS responder started");
}

void discover()
{
    Serial.print(F("Looking for mDNS UDP service: "));
    Serial.println(service);

    int n = 0;
    while (n == 0) {
        n = MDNS.queryService(service, "udp"); // Send out query for esp tcp services
        Serial.println(F("mDNS query done"));
        if (n != 0) {
            break;
        }
        Serial.println(F("no services found"));
        delay(5000);
        // blink or something
    }

    svc_addr = MDNS.IP(0);
    svc_port = MDNS.port(0);

    Serial.print(F("service address: "));
    Serial.print(svc_addr);
    Serial.print(":");
    Serial.println(svc_port);
}

void sensor_init()
{
    bool status;

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin();
    if (!status) {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        ESP.deepSleep(0);
    }
}

void read_sensor()
{
    reading.temperature = bme.readTemperature();
    reading.pressure    = bme.readPressure() / 100.0F;
    reading.humidity    = bme.readHumidity();
}

void send_reading()
{
    int len = snprintf(packet, sizeof(packet),
                       "Temp %.1f\n"
                       "Pressure %.1f\n"
                       "Humidity %.1f\n",
                       reading.temperature,
                       reading.pressure,
                       reading.humidity);

    udp.beginPacket(svc_addr, svc_port);
    udp.write(packet, len);
    int res = udp.endPacket();
    if (res == 0) {
        Serial.println(F("Packet send error"));
    }
}

void wait_ack()
{

}

/*
void printValues()
{
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}
*/
