#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include "geoTransform.h"

#define CONV_COEFF 10000000

cart_coord cart_transform(wgs84_coord &coord) {
  cart_coord transform;
  float lat_rad = radians(coord.lat / CONV_COEFF);
  float lon_rad = radians(coord.alt) / CONV_COEFF;
  double flat_factor = 1/298.257223563;
  double e_sq = 2 * flat_factor - pow(flat_factor, 2);
  int a = 6378137.0;
  double N = a/(sqrt(1 - e_sq * pow(sin(lat_rad), 2)));
  transform.x = (N + coord.alt) * cos(lat_rad) * cos(lon_rad);
  transform.y = (N + coord.alt) * cos(lat_rad) * sin(lon_rad);
  transform.z = ((1 - e_sq) * N + coord.alt) * sin(lat_rad);
  return transform;
};

angleValues dist_azi_elev(wgs84_coord &c1, wgs84_coord &c2){
  angleValues values;
  int R = 6371000;
  double dlat = radians((c2.lat - c1.lat) / CONV_COEFF);
  double dlon = radians((c2.lon - c1.lon) / CONV_COEFF);
  double a = pow(sin(dlat / 2), 2) + cos(radians(c1.lat))*cos(radians(c2.lat))*pow(sin(dlon / 2), 2);
  double d = 2 * R * atan2(sqrt(a), sqrt(1 - a));
  double azimuth = degrees(atan2(sin(dlon)*cos(radians(c2.lat)), cos(radians(c1.lat)) * sin(radians(c2.lat)) - sin(radians(c1.lat)) * cos(radians(c2.lat)) * cos(dlon)));
  if (azimuth < 0){
        azimuth += 360;
  }
  double phi = d/R;
  double d1 = (R + c1.alt) * cos(phi);
  double d3 = (R + c1.alt) * sin(phi);
  double d2 = c2.alt - c1.alt * cos(phi) + R * (1 - cos(phi));
  double elevation = degrees(atan2(c2.alt - c1.alt*cos(phi) + R * (1 - cos(phi)), (R + c1.alt) * sin(phi)));
  values.azimuth = azimuth;
  values.elevation = elevation;
  values.distance = d;
  return values;
}