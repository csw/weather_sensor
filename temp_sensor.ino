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

struct rst_info* rsti;
uint32_t         wake_n = 0;

const char* ssid       = "aje-csw";
const char* passphrase = "celeryairshipomitted";
char        host_string[16];
const int   LOCAL_PORT = 4000;

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

const static int LOC_WAKE_N = 66;
const static int LOC_SVC_ADDR = 67;
const static int LOC_SVC_PORT = 68;
const static int LOC_ACKED = 69;

const static int CALIBRATE_EVERY = 100;

class WifiSleepTypeCtx
{
public:
    WifiSleepTypeCtx(sleep_type_t st);
    ~WifiSleepTypeCtx();
private:
    sleep_type_t _saved;
};

void setup()
{
    Serial.begin(9600);
    Serial.println(F("\nBME280 sensor node"));

    rsti = system_get_rst_info();

    Serial.print(F("init wake_n: "));
    Serial.println(wake_n);
    load_rtc();
    Serial.print(F("loaded wake_n: "));
    Serial.println(wake_n);

    Serial.print(F("Reset reason: "));
    Serial.println(rsti->reason);

    if (is_cold_start()) {
        // boot; could be 0 (cold boot) or 6 (reset)
        wake_n = 0;
    } else {
        // wake from deep sleep
        ++wake_n;
    }
    system_rtc_mem_write(LOC_WAKE_N, &wake_n, sizeof(wake_n));
    Serial.print(F("Wakeup number: "));
    Serial.println(wake_n);

    sensor_init();
    wifi_init();
    yield();
    udp.begin(LOCAL_PORT);

    report.which_payload = Report_bme280_tag;

    Serial.println(F("Sensor node ready.\n"));
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("Reconnecting."));
        int tries = 0;
        do {
            delay(200);
        } while (WiFi.status() != WL_CONNECTED
                 && tries++ < 20);
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println(F("Reconnected."));
        } else {
            Serial.println(F("Failed to connect to wifi."));
            // sleep for a minute, do RF calibration in case that's what's needed
            ESP.deepSleep(60e6, WAKE_RFCAL);
        }
    }

    if (!acked) {
        discover();
    }
    read_sensor();
    send_and_await();
    //Serial.print(packet);
    Serial.println();

    //delay(delayTime);
    ESP.deepSleep(delayTime*1000, WAKE_NO_RFCAL);
}

void send_and_await()
{
    send_reading();
    dump_report();
    wait_ack();
}

void load_rtc()
{
    system_rtc_mem_read(LOC_WAKE_N, &wake_n, sizeof(wake_n));
    system_rtc_mem_read(LOC_SVC_ADDR, &svc_addr, sizeof(svc_addr));
    system_rtc_mem_read(LOC_SVC_PORT, &svc_port, sizeof(svc_port));
    system_rtc_mem_read(LOC_ACKED, &acked, sizeof(acked));
}

bool is_cold_start()
{
    return rsti->reason != 5;
}

void wifi_init()
{
    WiFi.persistent(false);
    WiFi.begin(ssid, passphrase);
    auto sc = WifiSleepTypeCtx(LIGHT_SLEEP_T);

    Serial.print(F("Connecting to wifi"));
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries++ < 20) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("Failed to connect, sleeping."));
        ESP.deepSleep(delayTime*1000, WAKE_NO_RFCAL);
    }

    Serial.print(F("Connected, IP address: "));
    Serial.println(WiFi.localIP());

    sprintf(host_string, "ESP_%06X", ESP.getChipId());
    Serial.print("Hostname: ");
    Serial.println(host_string);

}

void discover()
{
    WifiSleepTypeCtx sc(LIGHT_SLEEP_T);

    if (!MDNS.begin(host_string)) {
        Serial.println("Error setting up MDNS responder!");
    }
    Serial.println("mDNS responder started");

    Serial.print(F("Looking for mDNS UDP service: "));
    Serial.println(service);

    int n = 0;
    int tries = 0;
    while (n == 0 && tries++ < 10) {
        n = MDNS.queryService(service, "udp"); // Send out query for esp tcp services
        Serial.println(F("mDNS query done"));
        if (n != 0) {
            break;
        }
        Serial.println(F("no services found"));
        delay(2000);
        // blink or something
    }

    if (n == 0) {
        // discovery failed, sleep and try again
        Serial.println(F("Service discovery failed, sleeping."));
        ESP.deepSleep(delayTime*1000, WAKE_NO_RFCAL);
    }

    svc_addr = MDNS.IP(0);
    svc_port = MDNS.port(0);

    system_rtc_mem_write(LOC_SVC_ADDR, &svc_addr, sizeof(svc_addr));
    system_rtc_mem_write(LOC_SVC_PORT, &svc_port, sizeof(svc_port));

    Serial.print(F("service address: "));
    Serial.print(svc_addr);
    Serial.print(":");
    Serial.println(svc_port);
    Serial.flush();
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
    } else {
        Serial.println(F("Timed out waiting for ack."));
    }
    system_rtc_mem_write(LOC_ACKED, &acked, sizeof(acked));
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

WifiSleepTypeCtx::WifiSleepTypeCtx(sleep_type_t st)
    : _saved(wifi_get_sleep_type())
{
    wifi_set_sleep_type(st);
}

WifiSleepTypeCtx::~WifiSleepTypeCtx()
{
    wifi_set_sleep_type(_saved);
}
