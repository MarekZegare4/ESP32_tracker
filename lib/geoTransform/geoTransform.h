#pragma once

class wgs84_coord {
    public:
        float lat;
        float lon;
        float alt;
};

class cart_coord {
  public:
      double x;
      double y;
      double z;
};

cart_coord cart_transform(wgs84_coord &coord);

std::tuple<float, float, float> dist_azi_elev(wgs84_coord &c1, wgs84_coord &c2);