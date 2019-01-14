// -*- c++ -*-

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>


#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "env.pb.h"

#define BME_SCK 14
#define BME_MISO 12
#define BME_MOSI 13
#define BME_CS 15

#define SEALEVELPRESSURE_HPA (1013.25)

//Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
WiFiUDP         udp;

const char* ssid       = "aje-csw";
const char* passphrase = "celeryairshipomitted";
char        host_string[16];
const int   local_port = 4000;

const char* service = "weather";
IPAddress   svc_addr;
uint16_t    svc_port;

bool        acked = false;

Report   report;
Response response;

unsigned char sndbuf[256];
unsigned char rcvbuf[256];

char log_msg[256];

unsigned long delayTime = 20*1000;

void setup()
{
    Serial.begin(9600);
    Serial.println(F("BME280 test"));

    sensor_init();
    wifi_init();
    udp.begin(local_port);

    report.which_payload = Report_bme280_tag;

    Serial.println(F("Sensor node ready.\n"));
}


void loop()
{
    if (!acked) {
        discover();
    }
    read_sensor();
    send_and_await();
    //Serial.print(packet);
    Serial.println();

    delay(delayTime);
}

void send_and_await()
{
    send_reading();
    dump_report();
    wait_ack();
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
    report.payload.bme280.temperature = bme.readTemperature();
    report.payload.bme280.pressure    = bme.readPressure() / 100.0F;
    report.payload.bme280.humidity    = bme.readHumidity();
}

void send_reading()
{
    auto pb_ostream = pb_ostream_from_buffer(sndbuf, sizeof(sndbuf));
    bool ok = pb_encode(&pb_ostream, Report_fields, &report);
    if (!ok) {
        Serial.print(F("Packet encoding error: "));
        Serial.println(PB_GET_ERROR(&pb_ostream));
    }

    udp.beginPacket(svc_addr, svc_port);
    udp.write(reinterpret_cast<char*>(sndbuf), pb_ostream.bytes_written);
    int res = udp.endPacket();
    if (res == 0) {
        Serial.println(F("Packet send error"));
    }
}

void wait_ack()
{
    Serial.println(F("Waiting for ack."));
    acked = false;
    for (int i = 0; !acked && i < 10; ++i) {
        int size = udp.parsePacket();
        if (size == 0) {
            delay(500);
            continue;
        }
        Serial.print(F("Received UDP packet, size="));
        Serial.println(size);
        udp.read(rcvbuf, sizeof(rcvbuf));
        auto pb_istream = pb_istream_from_buffer(rcvbuf, sizeof(rcvbuf));
        auto ok = pb_decode(&pb_istream, Response_fields, &response);
        if (!ok) {
            Serial.print(F("Packet decoding error: "));
            Serial.println(PB_GET_ERROR(&pb_istream));
            continue;
        }
        Serial.println(F("Decoded packet OK."));
        if (response.which_payload == Response_ack_tag) {
            acked = true;
            break;
        }
    }
    if (acked) {
        Serial.println(F("Acknowledged."));
    }
}

void dump_report()
{
    Serial.print("Temperature = ");
    Serial.print(report.payload.bme280.temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(report.payload.bme280.pressure / 100.0F);
    Serial.println(" hPa");

    /*
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    */

    Serial.print("Humidity = ");
    Serial.print(report.payload.bme280.humidity);
    Serial.println(" %");
}
