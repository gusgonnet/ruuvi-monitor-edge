#include "Ruuvi.h"

Logger ruuviLog("app.ruuvi");

// NUS (Nordic UART Service)
// https://docs.ruuvi.com/communication/bluetooth-connection/nordic-uart-service-nus
const char *serviceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";

Ruuvi *Ruuvi::_instance;

// [static]
Ruuvi &Ruuvi::instance()
{
    if (!_instance)
    {
        _instance = new Ruuvi();
    }
    return *_instance;
}

Ruuvi::Ruuvi()
{
}

Ruuvi::~Ruuvi()
{
}

void Ruuvi::setup()
{
    ruuviDataEnhanced.timestamp = -1;
    BLE.on();
}

void Ruuvi::loop()
{
    static unsigned long lastTime = millis();
    unsigned long now = millis();
    if ((now - lastTime) >= 10000)
    {
        lastTime = now;

        ruuviLog.info("-----------------------------------------");
        ruuviLog.info("About to scan for Ruuvi tags at %s", Time.timeStr().c_str());

        BleScanFilter filter;
        filter.serviceUUID(serviceUuid);
        Vector<BleScanResult> scanResults = BLE.scanWithFilter(filter);

        ruuviLog.info("Tags found %u", scanResults.size());

        // print out the scan results
        for (int i = 0; i < scanResults.size(); i++)
        {
            uint8_t buf[BLE_MAX_SCAN_REPORT_BUF_LEN];
            size_t len;
            len = scanResults[i].advertisingData().get(buf, BLE_MAX_SCAN_REPORT_BUF_LEN);

            RuuviData ruuviData;
            if (!parseRuuviAdvertisement(buf, len, ruuviData))
            {
                ruuviLog.info("Parsing of the Ruuvi tag advertisement failed");
                continue;
            }

            ruuviLog.info("temperature=%.2f humidity=%.2f pressure=%.2f", ruuviData.temperature, ruuviData.humidity, ruuviData.pressure);
            ruuviLog.info("format=%u accelX=%.2f accelY=%.2f accelZ=%.2f battery=%.2f txPower=%.2f movements=%u sequence=%lu mac=%s", ruuviData.format, ruuviData.accelerationX, ruuviData.accelerationY, ruuviData.accelerationZ, ruuviData.batteryVoltage, ruuviData.txPower, ruuviData.movementCounter, ruuviData.measurementSequenceNumber, ruuviData.mac);

            // publish the first tag found to the cloud via ruuviDataEnhanced
            // if you have more than one tag, modify this code to your needs
            if (i == 0)
            {
                // copy the data to the enhanced structure
                ruuviDataEnhanced.ruuviData = ruuviData;

                // add the timestamp
                ruuviDataEnhanced.timestamp = Time.now();
            }
        }
    }
}

// The Ruuvi data is in this format:
// https://docs.ruuvi.com/communication/bluetooth-advertisements/data-format-5-rawv2
// example: 0201061bff99040516142d8fc4c7fc9401c8fffc8ed6f6cba8fc4c2295c482
bool Ruuvi::parseRuuviAdvertisement(const uint8_t *buf, size_t len, RuuviData &ruuviData)
{
    if (len < 31)
    {
        ruuviLog.info("len is less than 31, too short for a Ruuvi tag, skipping");
        return false;
    }

    // Flags (0x02):
    // 02 indicates that this is an advertising packet.
    if (buf[0] != 0x02)
    {
        ruuviLog.info("not an advertising packet, skipping");
        return false;
    }

    int index = 0;

    // Data Length (0x01):
    // indicates the length of the following data. Skip this byte and the following data.
    //  0  1  2  3
    // 02 01 06 1b
    index = 2 + buf[1];

    // Length (not counting length byte)
    size_t payloadLength = buf[index];
    index++;

    // validate the payload length + 4 (flags+length+type+length)
    if (payloadLength + 4 < len)
    {
        ruuviLog.info("payload length is less than len, skipping");
        return false;
    }

    // Type: 0xFF = Manufacturer specific data
    if (buf[index] != 0xFF)
    {
        ruuviLog.info("data type is not 0xFF, skipping");
        return false;
    }
    index++;

    // Manufacturer ID, least significant byte first: 0x0499 = Ruuvi Innovations Ltd
    uint16_t manufacturerId = (buf[index + 1] << 8) | buf[index];
    if (manufacturerId != 0x0499)
    {
        ruuviLog.info("manufacturer ID is not 0x0499 - Ruuvi Innovations Ltd, skipping");
        return false;
    }
    index += 2;

    // Payload data
    // example: 0516142d8fc4c7fc9401c8fffc8ed6f6cba8fc4c2295c482
    // 05 Data format (8bit)
    ruuviData.format = buf[index];
    index++;

    // next 1-2 bytes are the Temperature in 0.005 degrees Celsius
    ruuviData.temperature = (buf[index] << 8 | buf[index + 1]) * 0.005;
    index += 2;

    // next 2 bytes are the Humidity (16bit unsigned) in 0.0025% (0-163.83% range, though realistically 0-100%)
    ruuviData.humidity = (buf[index] << 8 | buf[index + 1]) * 0.0025;
    index += 2;

    // next 2 bytes are the Pressure (16bit unsigned) in 1 Pa units, with offset of -50 000 Pa
    ruuviData.pressure = (buf[index] << 8 | buf[index]) + 50000;
    index += 2;

    // next 2 bytes are the Acceleration-X (Most Significant Byte first)
    ruuviData.accelerationX = (buf[index] << 8 | buf[index + 1]);
    index += 2;

    // next 2 bytes are the Acceleration-Y (Most Significant Byte first)
    ruuviData.accelerationY = (buf[index] << 8 | buf[index + 1]);
    index += 2;

    // next 2 bytes are the Acceleration-Z (Most Significant Byte first)
    ruuviData.accelerationZ = (buf[index] << 8 | buf[index + 1]);
    index += 2;

    // next 2 bytes are the Power info (11+5bit unsigned), first 11 bits is the battery voltage above 1.6V, in millivolts (1.6V to 3.646V range).
    // Last 5 bits unsigned are the TX power above -40dBm, in 2dBm steps. (-40dBm to +20dBm range)
    ruuviData.batteryVoltage = (((buf[index] << 8 | buf[index + 1]) >> 5) + 1600) / 1000.0;
    ruuviData.txPower = (buf[index] << 8 | buf[index + 1]) & 0x1F;
    index += 2;

    // next byte is the Movement counter (8 bit unsigned), incremented by motion detection interrupts from accelerometer
    ruuviData.movementCounter = buf[index];
    index++;

    // next 2 bytes are the Measurement sequence number (16 bit unsigned), each time a measurement is taken, this is incremented by one, used for measurement de-duplication. Depending on the transmit interval, multiple packets with the same measurements can be sent, and there may be measurements that never were sent.
    ruuviData.measurementSequenceNumber = (buf[index] << 8 | buf[index + 1]);
    index += 2;

    // next 6 bytes are the MAC address (48bit)
    snprintf(ruuviData.mac, sizeof(ruuviData.mac), "%02x%02x%02x%02x%02x%02x",
             buf[index], buf[index + 1], buf[index + 2], buf[index + 3], buf[index + 4], buf[index + 5]);
    index += 6;

    return true;
}

// This is a callback from the Tracker class to add information to the location publish
void Ruuvi::locationGenCallback(JSONWriter &writer)
{
    if (-1 == ruuviDataEnhanced.timestamp)
    {
        ruuviLog.info("No Ruuvi data available");
        return;
    }

    ruuviLog.info("Adding Ruuvi data to the location publish");
    writer.name("ruuvi");
    writer.beginObject();
    writer.name("timestamp").value(ruuviDataEnhanced.timestamp);
    writer.name("temperature").value(ruuviDataEnhanced.ruuviData.temperature);
    writer.name("humidity").value(ruuviDataEnhanced.ruuviData.humidity);
    writer.name("pressure").value(ruuviDataEnhanced.ruuviData.pressure);
    writer.name("accelerationX").value(ruuviDataEnhanced.ruuviData.accelerationX);
    writer.name("accelerationY").value(ruuviDataEnhanced.ruuviData.accelerationY);
    writer.name("accelerationZ").value(ruuviDataEnhanced.ruuviData.accelerationZ);
    writer.name("batteryVoltage").value(ruuviDataEnhanced.ruuviData.batteryVoltage);
    writer.name("txPower").value(ruuviDataEnhanced.ruuviData.txPower);
    writer.name("movementCounter").value(ruuviDataEnhanced.ruuviData.movementCounter);
    writer.name("measurementSequenceNumber").value(ruuviDataEnhanced.ruuviData.measurementSequenceNumber);
    writer.name("mac").value(ruuviDataEnhanced.ruuviData.mac);
    writer.endObject();
}
