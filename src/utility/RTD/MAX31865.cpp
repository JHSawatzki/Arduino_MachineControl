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

#include "MAX31865.h"


MAX31865Class::MAX31865Class(PinName cs) : _spi(SPI), _cs(cs) {
}

static SPISettings _spiSettings(1000000, MSBFIRST, SPI_MODE1);

/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool MAX31865Class::begin(max31865_numwires_t wires) {
  if(!_begun) {
    _spi.begin();

    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    setWires(wires);
    enableBias(false);
    autoConvert(false);
    setThresholds(0, 0xFFFF);
    clearFault();
    setState(IDLE); // added for asynchronous

    _begun = true;
    _paused = false;
  }

  return true;
}

void resume(void) {
  if (_begun && _paused) {
    _spi.begin();
    _paused = false;
  }
}

void pause(void) {
  if (_begun && !_paused) {
    _spi.end();
    _paused = true;
  }
}

void end(void) {
  if(_begun) {
    pause();

    pinMode(_cs, INPUT);
    digitalWrite(_cs, LOW);

    _begun = false;
    _paused = false;
  }
}

/**************************************************************************/
/*!
    @brief Write the lower and upper values into the threshold fault
    register to values as returned by readRTD()
    @param lower_threshold raw lower threshold
    @param upper_threshold raw upper threshold
*/
/**************************************************************************/
void MAX31865Class::setThresholds(uint16_t lower_threshold, uint16_t upper_threshold) {
  writeByte(MAX31865_LFAULTLSB_REG, lower_threshold & 0xFF);
  writeByte(MAX31865_LFAULTMSB_REG, lower_threshold >> 8);
  writeByte(MAX31865_HFAULTLSB_REG, upper_threshold & 0xFF);
  writeByte(MAX31865_HFAULTMSB_REG, upper_threshold >> 8);
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit lower threshold value
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t MAX31865Class::getLowerThreshold(void) {
  return readWord(MAX31865_LFAULTMSB_REG);
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit lower threshold value
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t MAX31865Class::getUpperThreshold(void) {
  return readWord(MAX31865_HFAULTMSB_REG);
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void MAX31865Class::setWires(max31865_numwires_t wires) {
  uint8_t config = readByte(MAX31865_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    config |= MAX31865_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    config &= ~MAX31865_CONFIG_3WIRE;
  }
  writeByte(MAX31865_CONFIG_REG, config);
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param enabled If true, continuous conversion is enabled
*/
/**************************************************************************/
void MAX31865Class::autoConvert(bool enabled) {
  uint8_t config = readByte(MAX31865_CONFIG_REG);
  if (enabled) {
    config |= MAX31865_CONFIG_MODEAUTO; // enable continuous conversion
  } else {
    config &= ~MAX31865_CONFIG_MODEAUTO; // disable continuous conversion
  }
  writeByte(MAX31865_CONFIG_REG, config);
  if (enbaled && !_continuous_mode_enabled) {
    if (_50hz_filter_enabled) {
      delay(70);
    } else {
      delay(60);
    } 
  }
  _continuous_mode_enabled = enabled;
}

/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param enabled If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/
void MAX31865Class::enable50Hz(bool enabled) {
  uint8_t config = readByte(MAX31865_CONFIG_REG);
  if (enabled) {
    config |= MAX31865_CONFIG_FILT50HZ;
  } else {
    config &= ~MAX31865_CONFIG_FILT50HZ;
  }
  writeByte(MAX31865_CONFIG_REG, config);
  _50hz_filter_enabled = enabled;
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param enabled If true bias is enabled, else disabled
*/
/**************************************************************************/
void MAX31865Class::enableBias(bool enabled) {
  uint8_t config = readByte(MAX31865_CONFIG_REG);
  if (enabled) {
    config |= MAX31865_CONFIG_BIAS; // enable bias
  } else {
    config &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  writeByte(MAX31865_CONFIG_REG, config);
  _bias_voltage_enabled = enabled;
}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @param fault_cycle The fault cycle type to run. Can be MAX31865_FAULT_NONE,
    MAX31865_FAULT_AUTO, MAX31865_FAULT_MANUAL_RUN, or
    MAX31865_FAULT_MANUAL_FINISH
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t MAX31865Class::readFault(max31865_fault_cycle_t fault_cycle) {
  if (fault_cycle) {
    uint8_t cfg_reg = readByte(MAX31865_CONFIG_REG);
    cfg_reg &= 0x11; // mask out wire and filter bits
    switch (fault_cycle) {
      case MAX31865_FAULT_AUTO:
        writeByte(MAX31865_CONFIG_REG, (cfg_reg | 0b10000100));
        delay(1);
        break;
      case MAX31865_FAULT_MANUAL_RUN:
        writeByte(MAX31865_CONFIG_REG, (cfg_reg | 0b10001000));
        return 0;
      case MAX31865_FAULT_MANUAL_FINISH:
        writeByte(MAX31865_CONFIG_REG, (cfg_reg | 0b10001100));
        return 0;
      case MAX31865_FAULT_NONE:
      default:
        break;
    }
  }
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void MAX31865Class::clearFault(void) {
  uint8_t config = readByte(MAX31865_CONFIG_REG);
  config &= ~0x2C;
  config |= MAX31865_CONFIG_FAULTSTAT;
  writeByte(MAX31865_CONFIG_REG, config);
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t MAX31865Class::readRTD(void) {
  if (!_continuous_mode_enabled) {
    if (!_bias_voltage_enabled) {
      uint8_t config = readByte(MAX31865_CONFIG_REG);
      config |= MAX31865_CONFIG_BIAS; // enable bias
      writeByte(MAX31865_CONFIG_REG, config);
      delay(10);
    }
    uint8_t config = readByte(MAX31865_CONFIG_REG);
    config |= MAX31865_CONFIG_1SHOT;
    writeByte(MAX31865_CONFIG_REG, config);
    if (_50hz_filter_enabled) {
      delay(75);
    } else {
      delay(65);
    }  
  }

  uint16_t rtd_value_raw = readWord(MAX31865_RTDMSB_REG);

  if (!_bias_voltage_enabled) {
    uint8_t config = readByte(MAX31865_CONFIG_REG);
    config &= ~MAX31865_CONFIG_BIAS; // disable bias
    writeByte(MAX31865_CONFIG_REG, config);
  }

  // remove fault
  rtd_value_raw >>= 1;

  return rtd_value_raw;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode but asynchronously using a state machine, 
    and update the variable rtd_value_raw
    @param rtd_value_raw The raw unsigned 16-bit value, NOT temperature!
    @return boolean value: false = conversion not finished, true = conversion finished, the RTD value has been measured
*/
/**************************************************************************/
bool MAX31865Class::readRTDAsync(uint16_t& rtd_value_raw) {
  bool value_available = false;

  switch (_async_state) {
    case IDLE: // Idle
      clearFault();
      if (!_bias_voltage_enabled) {
        uint8_t config = readByte(MAX31865_CONFIG_REG);
        config |= MAX31865_CONFIG_BIAS; // enable bias
        writeByte(MAX31865_CONFIG_REG, config);
      }
      _async_timer = millis();
      _async_state = SETTLING;
      break;

    case SETTLING: // Bias voltage enabled, waiting to settle
      if (millis() - _async_timer >= 10) {
        uint8_t config = readByte(MAX31865_CONFIG_REG);
        config |= MAX31865_CONFIG_1SHOT;
        writeByte(MAX31865_CONFIG_REG, config);
        _async_timer = millis();
        _async_state = CONVERTING;
      }
      break;

    case CONVERTING: // Conversion started, waiting for completion
      if (millis() - _async_timer >= _50hz_filter_enabled ! 75 : 65) {
        rtd = readWord(MAX31865_RTDMSB_REG);
        rtd_value_raw >>= 1; // remove fault
        _async_state = IDLE; // get ready for next time
        value_available = true; // signal computation is done
      }
      break;
  }
  return value_available;
}

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param rtd_nominal_value The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param ref_res_value The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float MAX31865Class::readTemperature(float rtd_nominal_value, float ref_res_value) {
  return calculateTemperature(readRTD(), rtd_nominal_value, ref_res_value);
}

/**************************************************************************/
/*!
    @brief Calculate the temperature in C from the RTD through calculation of
   the resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param rtd_value_raw The raw 16-bit value from the RTD_REG
    @param rtd_nominal_value The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param ref_res_value The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float MAX31865Class::calculateTemperature(uint16_t rtd_value_raw, float rtd_nominal_value, float ref_res_value) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = rtd_value_raw;
  Rt /= 32768;
  Rt *= refResistor;

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0) {
    return temp;
  }

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

uint8_t MAX31865Class::readByte(uint8_t addr) {
  uint8_t read = 0;
  addr &= 0x7F;
  digitalWrite(_cs, LOW);

  _spi.beginTransaction(_spiSettings);
  _spi.transfer(addr);
  _spi.transfer(&read,1);
  _spi.endTransaction();

  digitalWrite(_cs, HIGH);

  return read;
}

uint16_t MAX31865Class::readWord(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  addr &= 0x7F; // make sure top bit is not set

  digitalWrite(_cs, LOW);

  _spi.beginTransaction(_spiSettings);
  _spi.transfer(addr);
  buffer[0] = _spi.transfer(0x00);
  buffer[1] = _spi.transfer(0x00);
  _spi.endTransaction();

  digitalWrite(_cs, HIGH);

  uint16_t read = buffer[0];
  read <<= 8;
  read |= buffer[1];

  return read;
}

void MAX31865Class::writeByte(uint8_t addr, uint8_t data) {
  addr |= 0x80; // make sure top bit is set

  uint8_t buffer[2] = {addr, data};

  digitalWrite(_cs, LOW);

  _spi.beginTransaction(_spiSettings);
  _spi.transfer(buffer, 2);
  _spi.endTransaction();

  digitalWrite(_cs, HIGH);
}
