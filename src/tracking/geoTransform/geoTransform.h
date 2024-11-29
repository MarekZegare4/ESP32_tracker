#pragma once

class Wgs84Coord {
    public:
        int32_t lat;
        int32_t lon;
        int32_t alt;
};

class CartCoord {
  public:
      double x;
      double y;
      double z;
};

class DMSCoord {
  public:
      int degrees;
      int minutes;
      double seconds;
};

class AngleValues {
  public:
      double azimuth;
      double elevation;
      double distance;
};

CartCoord cartTransform(Wgs84Coord &coord);
AngleValues distAziElev(Wgs84Coord &c1, Wgs84Coord &c2);
DMSCoord decimalToDMS(double decimalDegrees);
double convertHeading(double heading, double rotationAngle);