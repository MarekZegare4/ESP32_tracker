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
class AngleValues {
  public:
      float azimuth;
      float elevation;
      float distance;
};

CartCoord CartTransform(Wgs84Coord &coord);
AngleValues DistAziElev(Wgs84Coord &c1, Wgs84Coord &c2);