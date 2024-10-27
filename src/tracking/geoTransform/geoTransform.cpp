#include <Arduino.h>
#include <stdio.h>
#include <math.h>
#include "geoTransform.h"

#define CONV_COEFF 10000000

// http://www.movable-type.co.uk/scripts/latlong.html
// https://stackoverflow.com/questions/29858543/elevation-angle-between-positions

/**
 * @brief Function to convert WGS84 coordinates to cartesian coordinates
 * @param coord Wgs84Coord structure with latitude, longitude and altitude
 */
CartCoord CartTransform(Wgs84Coord &coord) {
  CartCoord transform;
  float lat_rad = radians(coord.lat / CONV_COEFF);
  float lon_rad = radians(coord.alt / CONV_COEFF);
  double flat_factor = 1/298.257223563;
  double e_sq = 2 * flat_factor - pow(flat_factor, 2);
  int a = 6378137.0;
  double N = a/(sqrt(1 - e_sq * pow(sin(lat_rad), 2)));
  transform.x = (N + coord.alt) * cos(lat_rad) * cos(lon_rad);
  transform.y = (N + coord.alt) * cos(lat_rad) * sin(lon_rad);
  transform.z = ((1 - e_sq) * N + coord.alt) * sin(lat_rad);
  return transform;
};

/**
 * @brief Function to calculate distance, azimuth and elevation between two WGS84 coordinates
 * @param c1 Wgs84Coord structure with latitude, longitude and altitude
 * @param c2 Wgs84Coord structure with latitude, longitude and altitude
 * @param coeff Coefficient used to convert latitude and longitude integer values to floats
 */
AngleValues DistAziElev(Wgs84Coord &c1, Wgs84Coord &c2, int coeff = CONV_COEFF){
  AngleValues values;
  int R = 6371000;
  double dlat = radians((c2.lat - c1.lat) / coeff);
  double dlon = radians((c2.lon - c1.lon) / coeff);
  double a = pow(sin(dlat / 2), 2) + cos(radians(c1.lat))*cos(radians(c2.lat))*pow(sin(dlon / 2), 2);
  double d = 2 * R * atan2(sqrt(a), sqrt(1 - a));
  double azimuth = degrees(atan2(sin(dlon)*cos(radians(c2.lat)), cos(radians(c1.lat)) * sin(radians(c2.lat)) - sin(radians(c1.lat)) * cos(radians(c2.lat)) * cos(dlon)));
  if (azimuth < 0){
        azimuth += 360;
  }
  double b = R + c1.alt;
  double c = R + c2.alt;
  double phi = d / R;
  double elevation = degrees(-asin(b - c * cos(phi))/(sqrt(b*b + c*c - 2 * b * c * cos(phi))));
  values.azimuth = azimuth;
  values.elevation = elevation;
  values.distance = d;
  return values;
}