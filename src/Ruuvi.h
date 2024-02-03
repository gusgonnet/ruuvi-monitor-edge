#ifndef __RUUVI_H
#define __RUUVI_H

#include "Particle.h"

struct RuuviData
{
    uint8_t format;                     // byte 0: Data format (8bit)
    float temperature;                  // bytes 1-2: Temperature in 0.005 degrees Celsius
    float humidity;                     // bytes 3-4: Humidity (16bit unsigned) in 0.0025% (0-163.83% range, though realistically 0-100%)
    float pressure;                     // bytes 5-6: Pressure (16bit unsigned) in 1 Pa units, with offset of -50 000 Pa
    float accelerationX;                // bytes 7-8: Acceleration-X (Most Significant Byte first)
    float accelerationY;                // bytes 9-10: Acceleration-Y (Most Significant Byte first)
    float accelerationZ;                // bytes 11-12: Acceleration-Z (Most Significant Byte first)
    float batteryVoltage;               // bytes 13-14: Power info (11+5bit unsigned), first 11 bits is the battery voltage above 1.6V, in millivolts (1.6V to 3.646V range).
    float txPower;                      // bytes 13-14: Last 5 bits unsigned are the TX power above -40dBm, in 2dBm steps. (-40dBm to +20dBm range)
    uint16_t movementCounter;           // byte 15: Movement counter (8 bit unsigned), incremented by motion detection interrupts from accelerometer
    uint32_t measurementSequenceNumber; // bytes 16-17: Measurement sequence number (16 bit unsigned), each time a measurement is taken, this is incremented by one, used for measurement de-duplication. Depending on the transmit interval, multiple packets with the same measurements can be sent, and there may be measurements that never were sent.
    char mac[18];                       // bytes 18-23: MAC address (48bit)
};

struct RuuviDataEnhanced
{
    RuuviData ruuviData;
    time32_t timestamp;
};

class Ruuvi
{
public:
    static Ruuvi &instance();
    void setup();
    void loop();
    void locationGenCallback(JSONWriter &writer);

private:
    bool parseRuuviAdvertisement(const uint8_t *buf, size_t len, RuuviData &ruuviData);
    RuuviDataEnhanced ruuviDataEnhanced;

protected:
    Ruuvi();
    virtual ~Ruuvi();
    Ruuvi(const Ruuvi &) = delete;
    Ruuvi &operator=(const Ruuvi &) = delete;
    static Ruuvi *_instance;
};
#endif /* __RUUVI_H */
