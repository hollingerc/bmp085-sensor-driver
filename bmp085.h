/*
 * File:       bmp085.h
 * Author:     Craig Hollinger
 *
 * This file is the public interface for bosch085.cpp.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
#ifndef _BMP085_H_
#define _BMP085_H_ 1

#include <Arduino.h>

class BMP085
{
  public:
    BMP085();
    boolean getBoschRawTemperature(void);
    boolean getBoschRawPressure(void);
    int calculateBoschTemperature(void);
    long calculateBoschPressure(void);

};
#endif // _BMP085_H_