/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/**********Adafruit_MAX31865 modified by Sylvain Boyer 17/02/2021 (https://github.com/sylvanoMTL/Adafruit_MAX31865)
 * 
 * 17/02/2021 - added some code from Jack Davies and J-M-L (arduino forum)
 *  https://forum.arduino.cc/index.php?topic=703346.msg4772457#msg4772457
 *    added: readRTDAsync() and temperatureAsync
 *    this allow to keep the code running rather than being stuck by the delays
 *  17/02/2021 - Fork from https://github.com/budulinek/Adafruit_MAX31865.git
 *    the fork contains code improvement to allow continuous measurement rather than single shot measurement
 *    this will improve reading rate but will cause self heating.
 *    the code also allow the use of the 50Hz filter with the appropriate delay.
 */

/**
 * Adaption of the library for the Arduino Portenta Machine Control
 * and dropped the initial lib from Arduino (which actually used
 * a lot of code from the original Adafruit lib!)
 */

#ifndef MAX31865_H
#define MAX31865_H

#include <Arduino.h>
#include <mbed.h>
#include <SPI.h>

#define MAX31865_CONFIG_REG 0x00
#define MAX31865_CONFIG_BIAS 0x80
#define MAX31865_CONFIG_MODEAUTO 0x40
#define MAX31865_CONFIG_MODEOFF 0x00
#define MAX31865_CONFIG_1SHOT 0x20
#define MAX31865_CONFIG_3WIRE 0x10
#define MAX31865_CONFIG_24WIRE 0x00
#define MAX31865_CONFIG_FAULTSTAT 0x02
#define MAX31865_CONFIG_FILT50HZ 0x01
#define MAX31865_CONFIG_FILT60HZ 0x00

#define MAX31865_RTDMSB_REG 0x01
#define MAX31865_RTDLSB_REG 0x02
#define MAX31865_HFAULTMSB_REG 0x03
#define MAX31865_HFAULTLSB_REG 0x04
#define MAX31865_LFAULTMSB_REG 0x05
#define MAX31865_LFAULTLSB_REG 0x06
#define MAX31865_FAULTSTAT_REG 0x07

#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

enum conversion_state_t : byte {IDLE, SETTLING, CONVERTING}; // for asynchronous mode

typedef enum {
  MAX31865_FAULT_NONE = 0,
  MAX31865_FAULT_AUTO,
  MAX31865_FAULT_MANUAL_RUN,
  MAX31865_FAULT_MANUAL_FINISH
} max31865_fault_cycle_t;

class MAX31865Class {
public:
  MAX31865Class(PinName cs = PA_6);

  bool begin(max31865_numwires_t wires = MAX31865_2WIRE);
  void resume(void);
  void pause(void);
  void end(void);

  void setThresholds(uint16_t lower_threshold, uint16_t upper_threshold);
  uint16_t getLowerThreshold(void);
  uint16_t getUpperThreshold(void);

  void setWires(max31865_numwires_t wires);
  void setAutoConvert(bool enabled);
  void set50HzFilter(bool enabled);
  void setBias(bool enabled);

  uint8_t readFault(max31865_fault_cycle_t fault_cycle = MAX31865_FAULT_AUTO);
  void clearFault(void);

  uint16_t readRTD(void);
  bool readRTDAsync(uint16_t& rtd_value_raw); //added by JD modified by Sylvain Boyer

  float readTemperature(float rtd_nominal_value, float ref_res_value);

  float calculateTemperature(uint16_t rtd_value_raw, float rtd_nominal_value, float ref_res_value);

private:
  uint8_t readByte(uint8_t addr);
  uint16_t readWord(uint8_t addr);
  void writeByte(uint8_t addr, uint8_t data);

  PinName _cs;
  SPIClass& _spi;

  bool _begun;
  bool _paused;
  bool _continuous_mode_enabled; // continuous conversion
  bool _50hz_filter_enabled; // 50Hz filter
  bool _bias_voltage_enabled; // bias voltage
  uint32_t _async_timer; // timer for asynchronous mode, added Sylvain Boyer
  conversion_state_t _async_state; // state for asynchronous mode //Does static matter?
};

#endif
