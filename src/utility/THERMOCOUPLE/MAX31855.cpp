/*
  This file is part of the MachineControl library.
  Copyright (c) 2020 Arduino SA.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "MAX31855.h"

//#include <Arduino_MachineControl.h>
//using namespace machinecontrol;

const double MAX31855Class::Jm210_760[];
const double MAX31855Class::J760_1200[];

const double MAX31855Class::Km270_0[];
const double MAX31855Class::K0_1372[];

const double MAX31855Class::Tm270_0[];
const double MAX31855Class::T0_400[];

const double MAX31855Class::InvJ_neg[];
const double MAX31855Class::InvJ0_760[];
const double MAX31855Class::InvJ760_1200[];

const double MAX31855Class::InvK_neg[];
const double MAX31855Class::InvK0_500[];
const double MAX31855Class::InvK500_1372[];

const double MAX31855Class::InvT_m200_0[];
const double MAX31855Class::InvT0_400[];

const MAX31855Class::coefftable MAX31855Class::CoeffJ[];
const MAX31855Class::coefftable MAX31855Class::CoeffK[];
const MAX31855Class::coefftable MAX31855Class::CoeffT[];

const MAX31855Class::coefftable MAX31855Class::InvCoeffJ[];
const MAX31855Class::coefftable MAX31855Class::InvCoeffK[];
const MAX31855Class::coefftable MAX31855Class::InvCoeffT[];

MAX31855Class::MAX31855Class(PinName cs) : _spi(SPI), _cs(cs), _coldOffset(2.10f) {
}

static SPISettings _spiSettings(4000000, MSBFIRST, SPI_MODE0);

int MAX31855Class::begin() {
  uint32_t rawword;

  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  _spi.begin();

  rawword = readSensor();
  if (rawword == 0xFFFFFF) {
    //comm_protocols.rs485.beginTransmission();
    //comm_protocols.rs485.println("TC begin error");
    //comm_protocols.rs485.endTransmission();
    end();

    return 0;
  }

  return 1;
}

void MAX31855Class::end() {
  pinMode(_cs, INPUT);
  digitalWrite(_cs, LOW);
  _spi.end();
}

uint32_t MAX31855Class::readSensor() {
  uint32_t read = 0x00;

  digitalWrite(_cs, LOW);
  delayMicroseconds(1);

  _spi.beginTransaction(_spiSettings);

  for (int i = 0; i < 4; i++) {
    read <<= 8;
    read |= _spi.transfer(0);
  }

  _spi.endTransaction();

  digitalWrite(_cs, HIGH);
  return read;
}

double MAX31855Class::polynomial(double value, int tableEntries, coefftable const (*table)) {
  double output = 0;
  double valuePower = 1;
  for (int i = 0; i < tableEntries; i++) {
    if (value < table[i].max) {
      if (table[i].size == 0) {
        return NAN;
      } else {
        output = 0;
        for (int j = 0; j < table[i].size; j++) {
          output += valuePower * table[i].coeffs[j];
          valuePower *= value;
        }
        return output;
      }
    }
  }
  return NAN;
}

double MAX31855Class::tempTomv(int type, double temp) {
  coefftable const (*table);
  int tableEntries;
  double voltage;

  switch (type) {
    case PROBE_J:
      table =  CoeffJ;
      tableEntries = sizeof(CoeffJ) / sizeof(coefftable);
    break;
    case PROBE_K:
      table = CoeffK;
      tableEntries = sizeof(CoeffK) / sizeof(coefftable);
    break;
    case PROBE_T:
      table = CoeffT;
      tableEntries = sizeof(CoeffT) / sizeof(coefftable);
    break;
  }
  voltage = polynomial(temp, tableEntries, table);
  // special case... for K probes in temperature range 0-1372 we need
  // to add an extra term
  if (type == PROBE_K && temp > 0) {
    voltage += 0.118597600000E+00 * exp( -0.118343200000E-03 * pow(temp - 0.126968600000E+03, 2));
  }
  return voltage;
}

double MAX31855Class::mvtoTemp(int type, double voltage) {
  coefftable const (*table);
  int tableEntries;

  switch (type) {
    case PROBE_J:
      table =  InvCoeffJ;
      tableEntries = sizeof(InvCoeffJ) / sizeof(coefftable);
    break;
    case PROBE_K:
      table = InvCoeffK;
      tableEntries = sizeof(InvCoeffK) / sizeof(coefftable);
    break;
    case PROBE_T:
      table = InvCoeffT;
      tableEntries = sizeof(InvCoeffT) / sizeof(coefftable);
    break;
  }
  return polynomial(voltage, tableEntries, table);
}

double MAX31855Class::readVoltage(int type) {
  uint32_t rawword;
  int32_t measuredTempInt;
  int32_t measuredColdInt;
  double measuredTemp;
  double measuredCold;
  double measuredVolt;

  //comm_protocols.rs485.beginTransmission();
  //comm_protocols.rs485.println("Read TC");
  //comm_protocols.rs485.endTransmission();
  rawword = readSensor();

  _lastFault = rawword & _faultMask;
  // Check for reading error
  if (_lastFault) {
    //comm_protocols.rs485.beginTransmission();
    //comm_protocols.rs485.println("TC reading error");
    //comm_protocols.rs485.endTransmission();
    return NAN; 
  }
  // The cold junction temperature is stored in the last 14 word's bits 
  // whereas the ttermocouple temperature (non linearized) is in the topmost 18 bits
  // sent by the Thermocouple-to-Digital Converter

  // sign extend thermocouple value
  if (rawword & 0x80000000) {
    // Negative value, drop the lower 18 bits and explicitly extend sign bits.
    measuredTempInt = 0xFFFC0000 | ((rawword >> 18) & 0x00003FFFF);
  } else {
    // Positive value, just drop the lower 18 bits.
    measuredTempInt = rawword >> 18;
  }

  // convert it to degrees
  measuredTemp = measuredTempInt * 0.25f;
  //comm_protocols.rs485.beginTransmission();
  //comm_protocols.rs485.print("Measured Temp: ");
  //comm_protocols.rs485.println(measuredTemp);
  //comm_protocols.rs485.endTransmission();

  // sign extend cold junction temperature
  measuredColdInt = (rawword >>4) & 0xfff;
  if (measuredColdInt & 0x800) {
    // Negative value, sign extend
    measuredColdInt |= 0xfffff000;
  }

  // convert it to degrees
  measuredCold = (measuredColdInt / 16.0f);
  //comm_protocols.rs485.beginTransmission();
  //comm_protocols.rs485.print("Measured CJC: ");
  //comm_protocols.rs485.println(measuredCold);
  //comm_protocols.rs485.endTransmission();

  // now the tricky part... since MAX31855K is considering a linear response 
  // and is trimemd for K thermocouples, we have to convert the reading back
  // to mV and then use NIST polynomial approximation to determine temperature
  // we know that reading from chip is calculated as:
  // temp = chip_temperature + thermocouple_voltage/0.041276f
  // 
  // convert temperature to mV is accomplished converting the chip temperature
  // to mV using NIST polynomial and then by adding the measured voltage 
  // calculated inverting the function above
  // this way we calculate the voltage we would have measured if cold junction 
  // was at 0 degrees celsius
  measuredVolt = tempTomv(type, measuredCold - _coldOffset) + (measuredTemp - measuredCold) * 0.041276f;

  return measuredVolt;
}

double MAX31855Class::readTemperature(int type) {
  double measuredVolt = readVoltage(type);

  // finally from the cold junction compensated voltage we calculate the temperature
  // using NIST polynomial approximation for the thermocouple type we are using
  return mvtoTemp(type, measuredVolt);
}

double MAX31855Class::readReferenceTemperature(int type) {
  uint32_t rawword;
  double reference;

  rawword = readSensor();

  // ignore first 4 FAULT bits
  rawword >>= 4;

  // The cold junction reference temperature is stored in the first 11 word's bits
  // sent by the Thermocouple-to-Digital Converter
  rawword = rawword & 0x7FF;
  // check sign bit  and convert to negative value.
  if (rawword & 0x800) {
    reference = (0xF800 | (rawword & 0x7FF)) * 0.0625f;
  } else {
    // multiply for the LSB value
    reference = rawword * 0.0625f;
  }

  return reference;
}

void MAX31855Class::setColdOffset(double offset) {
  _coldOffset = offset;
}

double MAX31855Class::getColdOffset() {
  return _coldOffset;
}

uint8_t MAX31855Class::getLastFault() {
  uint8_t tempLastFault = _lastFault;
  _lastFault = 0;
  return tempLastFault;
}

/**************************************************************************/
/*!
    @brief  Set the faults to check when reading temperature. If any set
    faults occur, temperature reading will return NAN.
    @param faults Faults to check. Use logical OR combinations of preset
    fault masks: TC_FAULT_OPEN, TC_FAULT_SHORT_GND,
    TC_FAULT_SHORT_VCC. Can also enable/disable all fault checks
    using: TC_FAULT_ALL or TC_FAULT_NONE.
*/
/**************************************************************************/
void MAX31855Class::setFaultChecks(uint8_t faults) {
  _faultMask = faults & TC_FAULT_ALL;
}