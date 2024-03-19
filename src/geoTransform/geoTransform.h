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

cart_coord cart_transform(wgs84_coord &coord);

std::tuple<float, float, float> dist_azi_elev(wgs84_coord &c1, wgs84_coord &c2);