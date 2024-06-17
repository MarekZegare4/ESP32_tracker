void gpsInitialize();
void gpsTask(void *parameters);

class TrackerDataGPS {
    public:
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        byte fixType;
        int8_t satCount;
};

TrackerDataGPS getTrackerGPS();