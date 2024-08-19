#include <SPI.h> //Needed for SPI to GNSS
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include "gps.h"

SFE_UBLOX_GNSS_SPI myGNSS; // SFE_UBLOX_GNSS_SPI uses SPI. For I2C or Serial, see Example1 and Example2

#define myCS 15 // Define the GPIO pin number for the SPI Chip Select. Change this if required
#define myClockSpeed 1000000 // Define what SPI clock speed to use. Change this if required

SPIClass spiPort(HSPI);
TrackerDataGPS gpsData;
void gpsInitialize()
{

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  while (myGNSS.begin(spiPort, myCS, myClockSpeed) == false) // Connect to the u-blox module using the port, chip select pin and speed defined above
  {
    delay(1000);
  }

  myGNSS.setSPIOutput(COM_TYPE_UBX); // Set the SPI port to output UBX only (turn off NMEA noise)

  // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR
}

TrackerDataGPS getTrackerGPS()
{
  return gpsData;
}

void gpsTask(void *parameters)
{
  for (;;)
  {
    // Request (poll) the position, velocity and time (PVT) information.
    // The module only responds when a new position is available. Default is once per second.
    // getPVT() returns true when new data is received.
    if (myGNSS.getPVT() == true)
    {
      gpsData.latitude = myGNSS.getLatitude();
      gpsData.longitude = myGNSS.getLongitude();
      gpsData.altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
      gpsData.fixType = myGNSS.getFixType();
      gpsData.satCount = myGNSS.getSIV();
    }
  }
}