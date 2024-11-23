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
CartCoord cartTransform(Wgs84Coord &coord) {
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
 */
AngleValues distAziElev(Wgs84Coord &c1, Wgs84Coord &c2){
  AngleValues values;
  int R = 6371008;
  double dlat = radians((c2.lat - c1.lat) / CONV_COEFF);
  double dlon = radians((c2.lon - c1.lon) / CONV_COEFF);
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

/**
 * @brief Function to convert decimal degrees to degrees, minutes, and seconds
 * @param decimalDegrees The coordinate in decimal degrees
 * @return A structure containing degrees, minutes, and seconds
 */
DMSCoord decimalToDMS(double decimalDegrees) {
  DMSCoord dms;
  dms.degrees = (int)decimalDegrees;
  double fractional = fabs(decimalDegrees - dms.degrees);
  dms.minutes = (int)(fractional * 60);
  dms.seconds = (fractional * 60 - dms.minutes) * 60;
  return dms;
}

/**
 * @brief Function to convert compass heading from -180 to 180 degrees to 0 to 360 degrees and rotate by a given angle
 * @param heading The compass heading in degrees from -180 to 180
 * @param rotationAngle The angle to rotate the heading by, in degrees
 * @return The compass heading in degrees from 0 to 360, rotated by the given angle
 */
double convertHeading(double heading, double rotationAngle) {
  double newHeading = heading + rotationAngle;
  if (newHeading < 0) {
    newHeading += 360;
  } else if (newHeading >= 360) {
    newHeading -= 360;
  }
  return newHeading;
}