struct __attribute__((packed)) FloatData
{
    uint8_t header;
    uint8_t teamID;
    uint8_t packageCounter;
    uint8_t status;
    uint8_t tiltAngle;
    uint16_t altitude;
    float latitude, longitude;
    float gpsAltitude;
    int16_t gY;
    uint8_t AngleY;
    uint8_t checksum;
};