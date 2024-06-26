#include "Arduino_MachineControl.h"

namespace machinecontrol {

TemperatureProbesClass temp_probes;
COMMClass comm_protocols;
AnalogInClass analog_in;
AnalogOutClass analog_out;
EncoderClass encoders;
DigitalOutputsClass digital_outputs;
DigitalInputsClass digital_inputs;
ProgrammableDIOClass digital_programmables;
RtcControllerClass rtc_controller;
USBClass usb_controller;
}
