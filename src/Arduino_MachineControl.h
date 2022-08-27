#ifndef __MACHINE_CONTROL_H__
#define __MACHINE_CONTROL_H__

#include "utility/RTD/MAX31865.h"
#include "utility/THERMOCOUPLE/MAX31855.h"
#include <ArduinoRS485.h>
#include "utility/QEI/QEI.h"
#include "utility/ioexpander/ArduinoIOExpander.h"
#include "utility/RTC/PCF8563T.h"
#include "utility/RTC/PCF8563T.h"

#include <Arduino.h>
#include <pinDefinitions.h>
#include <mbed.h>

namespace machinecontrol {

enum temperature_sensor_t : byte {TC, RTD};

/**
 * The TemperatureProbesClass allows enabling and selecting the different temperature sensor inputs
 * (RTD and thermocouples)
 */
class TemperatureProbesClass {
public:
	/**
	*  Select the input channel to be read (3 channels available)
	*  
	*  @param channel (0-2)
	*/
	void selectChannel(int channel, temperature_sensor_t temperature_sensor) {
		if (_current_channel != channel) {
			for (int i = 0; i < 3; i++) {
				_ch_sel[i] = (i == channel ? 1 : 0);
			}
			uint8_t switch_delay;
			switch (temperature_sensor) {
				case TC:
					switch_delay = 150;
					break;
				
				case RTD:
					switch_delay = 75;
					break;
				
				default:
					switch_delay = 150;
					break;
			}
			delay(switch_delay);
			_current_channel = channel;
		}
	}

	/**
	*  Select the input channel to be read (3 channels available)
	*  
	*  @param channel (0-2)
	*/
	bool selectChannelAsync(int channel, temperature_sensor_t temperature_sensor) {
		bool channel_selected = false;

		if (channel >= 0 && channel <= 2) {
			if (_current_channel != channel) {
				for (int i = 0; i < 3; i++) {
					_ch_sel[i] = (i == channel ? 1 : 0);
				}
				_current_channel = channel;
				_async_timer = millis();
				_async_state = SWITCHING;
			} else {
				switch (_async_state) {
					case SWITCHING: // Channel switching
						uint8_t switch_delay;
						switch (temperature_sensor) {
							case TC:
								switch_delay = 150;
								break;
							
							case RTD:
								switch_delay = 75;
								break;
							
							default:
								switch_delay = 150;
								break;
						}
						if (millis() - _async_timer >= switch_delay) {
							_async_state = SELECTED;
							channel_selected = true;
						}
						break;
					case SELECTED: // Channel selected
						channel_selected = true;
						break;
				}
			}
		}
		
		return channel_selected;
	}

	/**
	*  Disable the bias / force line
	*/   
	void enableTC() {
		_rtd_th = 0;
	}

	/**
	*  Enable the bias / force line
	*/   
	void enableRTD() {
		_rtd_th = 1;
	}

	/**
	*  Disable Chip select for both RTD and thermocouple digital converters. 
	*  
	*/   
	void disableCS() {
		digitalWrite(PA_7, HIGH);
		digitalWrite(PA_6, HIGH);
	}

	enum channel_select_state_t : byte {SWITCHING, SELECTED}; // for asynchronous mode

	MAX31865Class rtd = MAX31865Class(PA_6);
	MAX31855Class tc = MAX31855Class(PA_7);

private:
	mbed::DigitalOut _ch_sel[3] = { mbed::DigitalOut(PD_6), mbed::DigitalOut(PI_4), mbed::DigitalOut(PG_10)};
	mbed::DigitalOut _rtd_th = mbed::DigitalOut(PC_15);
	int _current_channel = -1;
	uint32_t _async_timer; // timer for asynchronous mode
	channel_select_state_t _async_state = SELECTED; // state for asynchronous mode
};

extern TemperatureProbesClass temp_probes;

/**
 * The COMMClass is used to initialize the CAN and RS485 LEDs and 
 * establish the power mode of the CAN bus. 
 */
class COMMClass {
public:
	// to be tested: check if can be made a big pin initialization

	 /**
	 * Shutdown RS485 and CAN LEDs
	 */
	void init() {
		//SHUTDOWN OF RS485 LEDS
		digitalWrite(PA_0, LOW);
		digitalWrite(PI_9, LOW);
		//SHUTDOWN OF CAN LEDS
		digitalWrite(PB_8, HIGH);
		digitalWrite(PH_13, HIGH);

		// SET DEFAULTS for RS485
		rs485Enable(false);
		rs485ModeRS232(false);
		rs485FullDuplex(false);
		rs485YZTerm(false);
		rs485ABTerm(false);
		rs485Slew(false);
	}

	 /**
	 * Set the CAN transceiver in Normal mode. In this mode, the transceiver 
	 * can transmit and receive data via the bus lines CANH and CANL.
	 */
	void enableCAN() {
		can_disable = 0;
	}

	 /**
	 * Set the CAN transceiver in standby (low power) mode. In this mode the
	 * transceiver will not be able to transmit or correctly receive data via the bus lines.
	 * The wake-up filter on the output of the low-power receiver does not latch bus dominant states,
	 * but ensures that only bus dominant and bus recessive states that persist longer than tfltr(wake)
	 * bus are reflected on pin RXD.
	 */
	void disableCAN() {
		can_disable = 1;
	}

	arduino::UART _UART4_ {PA_0, PI_9, NC, NC};
	mbed::CAN can {PB_8, PH_13};

	RS485Class rs485 {_UART4_, PA_0, PI_13, PI_10};

	void rs485Enable(bool enable) 		{ digitalWrite(PG_9,  enable ? 	HIGH : LOW); }
	void rs485ModeRS232(bool enable) 	{ digitalWrite(PA_10, enable ? 	LOW : HIGH); }
	void rs485YZTerm(bool enable) 		{ digitalWrite(PI_15, enable ? 	HIGH : LOW); }
	void rs485ABTerm(bool enable) 		{ digitalWrite(PI_14, enable ? 	HIGH : LOW); }
    void rs485Slew(bool enable)  		{ digitalWrite(PG_14, enable ? 	LOW : HIGH); }
	void rs485FullDuplex(bool enable) 	{
		digitalWrite(PA_9, enable ? LOW : HIGH);
		if (enable) {
			// RS485 Full Duplex require YZ and AB 120 Ohm termination enabled
			rs485YZTerm(true);
			rs485ABTerm(true);
		}
	}

private:
	mbed::DigitalOut can_disable = mbed::DigitalOut(PA_13, 0);
};

extern COMMClass comm_protocols;

#define ch0_in1		ch_in[0]
#define ch0_in2		ch_in[1]
#define ch0_in3		ch_in[2]
#define ch0_in4		ch_in[3]
#define ch1_in1		ch_in[4]
#define ch1_in2		ch_in[5]
#define ch1_in3		ch_in[6]
#define ch1_in4		ch_in[7]
#define ch2_in1		ch_in[8]
#define ch2_in2		ch_in[9]
#define ch2_in3		ch_in[10]
#define ch2_in4		ch_in[11]

/**
 * The AnalogInClass is used to set the resistor configuration for the right type of analog sensor
 * i.e. NTC sensors, 4-10mA or 0-10V. 
 */
class AnalogInClass {
public:

	 /**
	 * read the sampled voltage from the selected channel
	 * @param  channel integer for selecting the analog input (0, 1 or 2)
	 * @return the analog value between 0.0 and 1.0 normalized to a 16-bit value (uint16_t)
	 */
	uint16_t read(int channel) {
        uint16_t value = 0;
        switch (channel) {
            case 0:
                value = in_0.read_u16();
                break;
            case 1:
                value =  in_1.read_u16();
                break;
            case 2:
                value =  in_2.read_u16();
                break;
            default:
                break;
            }
        return value;
    }

	 /**
	 * Configure the input resistor dividers to have a ratio of 0.28.
	 * Maximum input voltage is 10V. 
	 */
	void set0_10V() {
		ch0_in1 = 1;
		ch0_in2 = 1;
		ch0_in3 = 0;
		ch0_in4 = 1;

		ch1_in1 = 1;
		ch1_in2 = 1;
		ch1_in3 = 0;
		ch1_in4 = 1;

		ch2_in1 = 1;
		ch2_in2 = 1;
		ch2_in3 = 0;
		ch2_in4 = 1;
	}

	 /**
	 * Enable a 120 ohm resistor to GND to convert the 4-20mA sensor currents to voltage. 
	 * Note: 24V are available from the carrier to power the 4-20mA sensors.   
	 */
	void set4_20mA() {
		ch0_in1 = 1;
		ch0_in2 = 0;
		ch0_in3 = 1;
		ch0_in4 = 0;

		ch1_in1 = 1;
		ch1_in2 = 0;
		ch1_in3 = 1;
		ch1_in4 = 0;

		ch2_in1 = 1;
		ch2_in2 = 0;
		ch2_in3 = 1;
		ch2_in4 = 0;
	}

	 /**
	 * Enable a 100K resistor in series with the reference voltage.	 
	 * The voltage sampled is the voltage division between the 100k resistor and the input resistor (NTC/PTC)  
	 */
	void setNTC() {
		ch0_in1 = 0;
		ch0_in2 = 0;
		ch0_in3 = 1;
		ch0_in4 = 1;

		ch1_in1 = 0;
		ch1_in2 = 0;
		ch1_in3 = 1;
		ch1_in4 = 1;

		ch2_in1 = 0;
		ch2_in2 = 0;
		ch2_in3 = 1;
		ch2_in4 = 1;
	}

	mbed::AnalogIn& operator[](int index) {
		switch (index) {
			case 0:
				return in_0;
			case 1:
				return in_1;
			case 2:
				return in_2;
		}
	}

	mbed::DigitalOut ch_in[12] = { 
		mbed::DigitalOut(PD_4), mbed::DigitalOut(PD_5), mbed::DigitalOut(PE_3), mbed::DigitalOut(PG_3),
		mbed::DigitalOut(PD_7), mbed::DigitalOut(PH_6), mbed::DigitalOut(PJ_7), mbed::DigitalOut(PH_15),
		mbed::DigitalOut(PH_10), mbed::DigitalOut(PA_4), mbed::DigitalOut(PA_8), mbed::DigitalOut(PC_6) 
	};

private:
	mbed::AnalogIn in_0 = mbed::AnalogIn(PC_3C);
	mbed::AnalogIn in_1 = mbed::AnalogIn(PC_2C);
	mbed::AnalogIn in_2 = mbed::AnalogIn(PA_1C);
};

extern AnalogInClass analog_in;

class AnalogOutClass {
public:

	 /**
	 * Set output voltage value (PWM)
	 * @param  index select channel
	 * @param  voltage desired output voltage (max 10.5V)
	 */
	void write(int index, float voltage) {
		if (voltage < 0) {
			voltage = 0;
		}

		switch (index) {
			case 0:
				out_0.write(voltage / 10.5);
				break;
			case 1:
				out_1.write(voltage / 10.5);
				break;
			case 2:
				out_2.write(voltage / 10.5);
				break;
			case 3:
				out_3.write(voltage / 10.5);
				break;
		}
	}

	 /**
	 * Set the PWM period (frequency)
	 * @param  index select channel
	 * @param  period integer for selecting the period in ms
	 */
	void period_ms(int index, uint8_t period) {
		switch (index) {
			case 0:
				out_0.period_ms(period);
				break;
			case 1:
				out_1.period_ms(period);
				break;
			case 2:
				out_2.period_ms(period);
				break;
			case 3:
				out_3.period_ms(period);
				break;
		}
	}
	mbed::PwmOut& operator[](int index) {
		switch (index) {
			case 0:
				return out_0;
			case 1:
				return out_1;
			case 2:
				return out_2;
			case 3:
				return out_3;
		}
	}
private:
	mbed::PwmOut out_0 = mbed::PwmOut(PJ_11);
	mbed::PwmOut out_1 = mbed::PwmOut(PK_1);
	mbed::PwmOut out_2 = mbed::PwmOut(PG_7);
	mbed::PwmOut out_3 = mbed::PwmOut(PC_7);
};

extern AnalogOutClass analog_out;


/* 
  TODO: writeme 
  Use QEI library for mbed since it implements index pin
*/
 /**
 * The EncoderClass is a wrapper for manipulating Quadrature Encoder Interface devices.
 */
class EncoderClass {
public:
  /**
	 * returns the encoder variable depending on the index
	 * @param  index integer for selecting the encoder (0 or 1)
	 * @return enc_0 for index = 0, enc_1 for index = 1
	 */
	EncoderClass()
		: enc_0{PJ_8, PH_12, PH_11, 0}
		, enc_1{PC_13, PI_7, PJ_10, 0} {};


	QEI& operator[](int index) {
		switch (index) {
			case 0:
				return enc_0;
			case 1:
				return enc_1;
		}
	}
private:
	QEI enc_0;
	QEI enc_1;
};

extern EncoderClass encoders;

/* 
  using gpio expander class https://www.i2cdevlib.com/devices/tca6424a#source
  Ask Giampaolo for proper porting
  Expander interrupt is PI_5
  prog_latch_retry (AKA TERM ? ) is PH_14

  TODO: check if Wire and address are correct
*/


/**
 * The ProgrammableDIOClass is used to initialize the IOExpanders and configure the 
 * thermal shutdown mode of the high side switches.
 */
class ProgrammableDIOClass : public ArduinoIOExpanderClass {
public:

	/**
	 * Test connection with the IOExpander and set all the pins to the default mode. 
	 * @return true if OK, false if fault
	 */
	bool init() {
		return begin(IO_ADD);
	}

	/**
	 * Configures the thermal shutdown of the high-side switches (TPS4H160) to operate in latch mode. 
	 * The output latches off when thermal shutdown occurs. 
	 */
	void setLatch() {
		prog_latch_retry = 0;
	}

	 /**
	 * Configures the thermal shutdown of the high-side switches (TPS4H160) to operate in auto-retry mode. 
	 * The output automatically recovers when TJ < T(SD) – T(hys), but the current is limited to ICL(TSD) 
	 * to avoid repetitive thermal shutdown. 
	 */
	void setRetry() {
		prog_latch_retry = 1;
	}
private:
	mbed::DigitalOut prog_latch_retry = mbed::DigitalOut(PH_14);
};

extern ProgrammableDIOClass digital_programmables;


/**
 * The DigitalOutputClass is used to interface with the IO Expander and
 * set the digital outputs.
 */
class DigitalOutputsClass {
public:

	/**
	 * Set all digital outputs at the same time. 
	 * @param val 8 bit integer to set all 8 channels. e.g:
	 * Set all to HIGH -> val = 255 (0b11111111)
	 * Set all to LOW  -> val = 0   (0b00000000)
	 */
	void setAll(uint8_t val) {
		for (int i = 0; i < 8; i++) {
			out[i] = val & 0x1;
			val = val >> 1;
		}
	}

	 /**
	 * Set a particular digital output
	 * @param index digital output to be set
	 * @param val set value (HIGH/LOW)
	 */
	void set(int index, bool val) {
			out[index] = val;
	}

	/**
	 * Configures the thermal shutdown of the high-side switches (TPS4H160) to operate in latch mode. 
	 * The output latches off when thermal shutdown occurs. 
	 */
	void setLatch() {
		dig_out_latch_retry = 0;
	}

	 /**
	 * Configures the thermal shutdown of the high-side switches (TPS4H160) to operate in auto-retry mode. 
	 * The output automatically recovers when TJ < T(SD) – T(hys), but the current is limited to ICL(TSD) 
	 * to avoid repetitive thermal shutdown. 
	 */
	void setRetry() {
		dig_out_latch_retry = 1;
	}
	mbed::DigitalOut& operator[](int index) {
		return out[index];
	}
private:
	mbed::DigitalOut dig_out_latch_retry = mbed::DigitalOut(PB_2);
	mbed::DigitalOut out[8] = {
		mbed::DigitalOut(PI_6), mbed::DigitalOut(PH_9), mbed::DigitalOut(PJ_9), mbed::DigitalOut(PE_2),
		mbed::DigitalOut(PI_3), mbed::DigitalOut(PI_2), mbed::DigitalOut(PD_3), mbed::DigitalOut(PA_14)
	};
};

extern DigitalOutputsClass digital_outputs;

class DigitalInputsClass : public ArduinoIOExpanderClass {
public:
	/**
	 * Test connection with the IOExpander and set all the pins to the default mode. 
	 * @return true if OK, false if fault
	 */
	bool init() {
		return begin(DIN_ADD);
	}
};

extern DigitalInputsClass digital_inputs;

/**
 * The RtcControllerClass is a wrapper for the PCF8563TClass() that is used to 
 * set and get the time to/from the PCF8563T RTC.
 *  
 */
class RtcControllerClass : public PCF8563TClass {
public:
	mbed::DigitalIn int_pin = mbed::DigitalIn(PB_9,PullUp);
private:

};

extern RtcControllerClass rtc_controller;


/**
 * The USB Class is used to enable/disable the power of the USBA (Host) and configure
 * the callbacks for the different host types (i.e. Keyboard, mouse, storage device etc). 
 */
class USBClass {
public:
	USBClass()
		: _power{PB_14, 0}
		, _usbflag{PB_15}
		{};
		
	/**
	 * Enable power to USBA VBUS. 
	 */
	void powerEnable() {
		_power = 0;
	}

	/**
	 * Disable power to USBA VBUS.  
	 */
	void powerDisable() {
		_power = 1;
	}

	/**
	 * Flag to indicate overcurrent, overtemperature, or reverse−voltage conditions on the USBA VBUS. 	 
	 * Active−low open−drain output.
	 * @return true if OK, false if fault
	 */
	bool vflagRead() {
		return _usbflag;
	}

private:
	mbed::DigitalOut _power;
	mbed::DigitalIn _usbflag;
};


extern USBClass usb_controller;


}
#endif
