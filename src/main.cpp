#include "Particle.h"
#include "edge.h"
#include "Ruuvi.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#if EDGE_PRODUCT_NEEDED
PRODUCT_ID(EDGE_PRODUCT_ID);
#endif // EDGE_PRODUCT_NEEDED
PRODUCT_VERSION(EDGE_PRODUCT_VERSION);

STARTUP(
    Edge::startup(););

SerialLogHandler logHandler(115200, LOG_LEVEL_INFO,
                            {
                                {"app.gps.nmea", LOG_LEVEL_WARN},
                                {"app.gps.ubx", LOG_LEVEL_WARN},
                                {"ncp.at", LOG_LEVEL_WARN},
                                {"net.ppp.client", LOG_LEVEL_WARN},
                                {"app.ruuvi", LOG_LEVEL_INFO},
                            });

// add ruuvi tags information to the Monitor One publish
void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{
    Ruuvi::instance().locationGenCallback(writer);
}

void setup()
{
    Edge::instance().init();
    Ruuvi::instance().setup();

    // Callback to add information to the location publish
    // 
    Tracker::instance().location.regLocGenCallback(myLocationGenerationCallback);
}

void loop()
{
    Edge::instance().loop();
    Ruuvi::instance().loop();
}
