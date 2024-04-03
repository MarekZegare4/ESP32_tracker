#pragma once

class wgs84_coord {
    public:
        int32_t lat;
        int32_t lon;
        int32_t alt;
};

class cart_coord {
  public:
      double x;
      double y;
      double z;
};
class angleValues {
  public:
      float azimuth;
      float elevation;
      float distance;
};
cart_coord cart_transform(wgs84_coord &coord);
angleValues dist_azi_elev(wgs84_coord &c1, wgs84_coord &c2);