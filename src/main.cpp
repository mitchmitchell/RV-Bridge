/*********************************************************************************
 *  MIT License
 *  
 *  Copyright (c) 2023 Randy Ubillos
 *  
 *  https://github.com/rubillos/RV-Bridge
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *  
 ********************************************************************************/
 
////////////////////////////////////////////////////////////////
//                                                            //
//    RV-Bridge: A HomeKit to RV-C interface for the ESP32    //
//                                                            //
////////////////////////////////////////////////////////////////

#include "Arduino.h"

#include <algorithm>
#include <vector>

#include "elapsedMillis.h"
#include "HomeSpan.h" 
#include "ESP32CAN.h"
#include "CAN_config.h"

#include "config.h"

//////////////////////////////////////////////

constexpr const char* versionString = "v1.1.0";

constexpr uint8_t indicatorPinR = 2;
constexpr uint8_t indicatorPinG = 15;
constexpr uint8_t indicatorPinB = 4;

constexpr gpio_num_t canTxPin = GPIO_NUM_25;
constexpr gpio_num_t canRxPin = GPIO_NUM_26;

// A busy RV-C bus carries a few hundred frames per second and homeSpan.poll() can take
// several milliseconds, so the receive queue needs real headroom or status frames get dropped.
constexpr uint16_t receiveQueueSize = 64;
constexpr uint16_t maxPacketsPerLoop = 32;
constexpr uint16_t sendQueueSize = 8;

constexpr uint32_t sendPacketIntervalmS = 50;
constexpr uint32_t minSendPacketIntervalmS = 5;

constexpr uint32_t packetBlinkTime = 25;
constexpr uint32_t heatbeatRate = 3000;
constexpr uint32_t heartbeatBlinkTime = 10;

elapsedMillis lastPacketSendTime = 1000;
elapsedMillis lastPacketRecvTime = 1000;

CAN_device_t CAN_cfg;               	// CAN Config

//////////////////////////////////////////////

enum {
	packetPrintNo = 0,
	packetPrintYes,
	packetPrintIfUnknown,
	packetPrintIfKnown
};

void displayPacket(CAN_frame_t *packet, uint8_t printPacket);
uint8_t packetPrintMode = packetPrintNo;

//////////////////////////////////////////////
// RV-C dgn numbers

constexpr uint32_t DATE_TIME_STATUS = 0x1FFFF;
constexpr uint32_t DC_DIMMER_COMMAND_2 = 0x1FEDB;
constexpr uint32_t DC_DIMMER_STATUS_3 = 0x1FEDA;
constexpr uint32_t THERMOSTAT_AMBIENT_STATUS = 0x1FF9C;
constexpr uint32_t THERMOSTAT_STATUS_1 = 0x1FFE2;
constexpr uint32_t THERMOSTAT_COMMAND_1 = 0x1FEF9;

constexpr uint32_t AIR_CONDITIONER_STATUS = 0x1FFE1;
constexpr uint32_t AIR_CONDITIONER_COMMAND = 0x1FFE0;
constexpr uint32_t GENERIC_INDICATOR_COMMAND = 0x1FED9;
constexpr uint32_t GENERIC_INDICATOR_STATUS = 0x1FED7;
constexpr uint32_t GENERIC_CONFIGURATION_STATUS = 0x1FED8;
constexpr uint32_t DC_SOURCE_STATUS_1 = 0x1FFFD;
constexpr uint32_t TANK_STATUS = 0x1FFB7;
constexpr uint32_t GENERATOR_STATUS_1 = 0x1FFDC;

constexpr uint32_t FURNACE_STATUS = 0x1FFE4;
constexpr uint32_t FURNACE_COMMAND = 0x1FFE3;

constexpr uint32_t INVERTER_TEMPERATURE_STATUS = 0x1FEBD;	
constexpr uint32_t DC_LOAD_STATUS_2 = 0x1FEDC;
constexpr uint32_t ATS_AC_STATUS_4 = 0x1FF85;
constexpr uint32_t ATS_STATUS = 0x1FFAA;	
constexpr uint32_t ATS_AC_STATUS_3 = 0x1FFAB;
constexpr uint32_t ATS_AC_STATUS_1 = 0x1FFAD;
constexpr uint32_t DC_LOAD_COMMAND = 0x1FFBC;
constexpr uint32_t DC_LOAD_STATUS = 0x1FFBD;
constexpr uint32_t AC_LOAD_STATUS = 0x1FFBF;
constexpr uint32_t CHARGER_AC_STATUS_1 = 0x1FFCA;
constexpr uint32_t INVERTER_STATUS = 0x1FFD4;
constexpr uint32_t INVERTER_AC_STATUS_1 = 0x1FFD7;

constexpr uint32_t AWNING_STATUS = 0x1FEF3;
constexpr uint32_t AWNING_COMMAND = 0x1FEF2;
constexpr uint32_t AWNING_STATUS_2 = 0x1FDCD;

constexpr uint32_t BATTERY_STATUS = 0x1AAFD;		// not a standard RV-C DGN (kept from upstream); DC_SOURCE_STATUS_1 is the standard one

constexpr uint32_t THERMOSTAT_STATUS_2 = 0x1FEFA;
constexpr uint32_t THERMOSTAT_COMMAND_2 = 0x1FEF8;
constexpr uint32_t WATERHEATER_STATUS = 0x1FFF7;
constexpr uint32_t WATERHEATER_COMMAND = 0x1FFF6;
constexpr uint32_t WATER_PUMP_STATUS = 0x1FFB3;
constexpr uint32_t WATER_PUMP_COMMAND = 0x1FFB2;
constexpr uint32_t SLIDE_STATUS = 0x1FFE8;
constexpr uint32_t SLIDE_COMMAND = 0x1FFE7;
constexpr uint32_t LEVELING_CONTROL_COMMAND = 0x1FFEE;
constexpr uint32_t LEVELING_CONTROL_STATUS = 0x1FFED;
constexpr uint32_t GENERATOR_COMMAND = 0x1FFDA;
constexpr uint32_t DM_RV = 0x1FECA;

// PDU1-format messages: the low byte of the DGN is the destination address
constexpr uint32_t ACKNOWLEDGMENT = 0x0E800;
constexpr uint32_t REQUEST_FOR_DGN = 0x0EA00;
constexpr uint32_t TP_DATA_PACKET = 0x0EB00;
constexpr uint32_t TP_INITIAL_PACKET = 0x0EC00;
constexpr uint32_t ADDRESS_CLAIMED = 0x0EE00;
constexpr uint32_t PRODUCT_ID = 0x0FEEB;

//////////////////////////////////////////////

typedef enum {
	DCDimmerCmdSetBrightness = 0,
	DCDimmerCmdOnDuration,
	DCDimmerCmdOnDelay,
	DCDimmerCmdOff,
	DCDimmerCmdStop,
	DCDimmerCmdToggle,
	DCDimmerCmdMemoryOff,
	DCDimmerCmdRampBrightness = 17,
	DCDimmerCmdRampToggle,
	DCDimmerCmdRampUp,
	DCDimmerCmdRampDown,
	DCDimmerCmdRampUpDown,
	DCDimmerCmdLock = 33,
	DCDimmerCmdUnlock,
	DCDimmerCmdFlash = 49,
	DCDimmerCmdFlashMomentarily,

	DCDimmerCmdNA = 255
} DCDimmerCmd;

typedef enum {
	ACModeAuto = 0,
	ACModeManual,

	ACModeNA = 3
} ACMode;

// Fixed underlying types: these are cast straight from bit fields in received packets, and a
// value outside the named range would otherwise be undefined behaviour.
typedef enum : uint8_t {
	ThermostatModeOff = 0,
	ThermostatModeCool,
	ThermostatModeHeat,
	ThermostatModeAuto,
	ThermostatModeFanOnly,
	ThermostatModeAuxHeat,
	ThermostatModeDehumidify,

	ThermostatModeNA = 7
} ThermostatMode;

typedef enum : uint8_t {
	FanModeAuto = 0,
	FanModeOn,

	FanModeNA = 3
} FanMode;

// RV-C percentages are sent in 0.5% steps, so 200 = 100%. 251-255 are "error"/"not available" codes.
constexpr uint8_t RVCPercentMax = 250;
constexpr uint8_t RVCBrightMax = 200;
constexpr uint8_t RVCFanMax = 200;
constexpr uint8_t RVCInstanceMax = 250;
constexpr uint8_t HomeKitPercentMax = 100;

//////////////////////////////////////////////

constexpr float homeKitShadeOpenValue = 100.0;
constexpr float homeKitShadeClosedValue = 0.0;

constexpr uint8_t homeKitPositionStateClosing = 0;
constexpr uint8_t homeKitPositionStateOpening = 1;
constexpr uint8_t homeKitPositionStateStopped = 2;

constexpr uint8_t homeKitTemperatureDisplayCelsius = 0;
constexpr uint8_t homeKitTemperatureDisplayFahrenheit = 1;

// constexpr float closePerMS = HomeKitPercentMax / closeTimeMS;
// constexpr float openPerMS = HomeKitPercentMax / openTimeMS;

constexpr uint32_t updateTimeMS = 500;
constexpr uint32_t quickUpdateTimeMS = 100;

typedef uint16_t ShadeState;
constexpr uint16_t shadeStateOpen = 0;
constexpr uint16_t shadeStateStopped = 1;
constexpr uint16_t shadeStateClosed = 2;

constexpr uint16_t shadeStateClosing = 1 << 3;
constexpr uint16_t shadeStateOpening = 1 << 4;

constexpr uint16_t shadeStateUserAction = 1 << 5;
constexpr uint16_t shadeStateLocalAction = 1 << 6;
constexpr uint16_t shadeStateHomeKitAction = 1 << 7;

//////////////////////////////////////////////

uint64_t millis64() {
	volatile static uint32_t low32 = 0, high32 = 0;
	uint32_t new_low32 = millis();

	if (new_low32 < low32)
		high32++;

	low32 = new_low32;

	return (uint64_t) high32 << 32 | low32;
}

//////////////////////////////////////////////

uint32_t getMsgBits(uint32_t msg, int8_t startBit, int8_t numBits) {
	int8_t shift = startBit - numBits + 1;
	uint32_t mask = 0xFFFFFFFF >> (32 - numBits);

	return (msg >> shift) & mask;
}

uint32_t makeMsg(uint32_t dgn, uint8_t sourceID=0, uint8_t priority=6) {
	if (sourceID == 0) {
		sourceID = sourceAddress;
	}

	return ((uint32_t)priority<<26) | (dgn << 8) | sourceID;	
}

// For PDU1-format messages (DGN-high below 0xF0) the low byte is a destination address,
// not part of the DGN. Strip it so those messages can be compared against a constant.
uint32_t dgnBase(uint32_t dgn) {
	return (((dgn >> 8) & 0xFF) < 0xF0) ? (dgn & 0x1FF00) : dgn;
}

//////////////////////////////////////////////

constexpr double tempCOffset = -273.0;
constexpr double tempCScale = 0.03125;
constexpr double tempCScaleInv = 1.0 / tempCScale;
// constexpr double tempCRoundingOffset = 0.25;
constexpr double tempCRoundingOffset = -0.25;

// Returns NAN for the RV-C "not available"/"error"/"reserved" codes (0xFFFD-0xFFFF)
double convToTempC(uint16_t value) {
	if (value >= 0xFFFD) {
		return NAN;
	}
	return tempCOffset + value * tempCScale + tempCRoundingOffset;
}

uint16_t convFromTempC(double tempC) {
	return (tempC - tempCOffset + 0.5) / tempCScale;
}

//////////////////////////////////////////////

CAN_frame_t packetQueue[sendQueueSize];
bool packetShortGap[sendQueueSize];
uint16_t packetQueueHead = 0;
uint16_t packetQueueTail = 0;

// A definition file whose instance numbers have not been confirmed on a real coach defines
// RV_DEFINITION_UNVERIFIED. Writes then start out disabled ("@w 1" turns them on) so a guessed
// instance number can never operate the wrong load.
#ifdef RV_DEFINITION_UNVERIFIED
bool doCANWrite = false;
#else
bool doCANWrite = true;
#endif

void processPacketQueue() {
	if (packetQueueHead != packetQueueTail) {
		uint16_t nextIndex = (packetQueueTail + 1) % sendQueueSize;
		uint32_t interval = (packetShortGap[nextIndex]) ? minSendPacketIntervalmS : sendPacketIntervalmS;

		if (lastPacketSendTime >= interval) {
			if (doCANWrite) {
#ifndef DISABLE_CAN_BUS_WRITES
				printf("%u: CAN-Bus Send Packet\n", (uint32_t)millis());
				ESP32Can.CANWriteFrame(&packetQueue[nextIndex]);
#else
				printf("%u: ***DISABLED*** CAN-Bus Send Packet\n", (uint32_t)millis());
#endif
			}
			else {
				printf("%u: ***SIMULATE*** CAN-Bus Send Packet\n", (uint32_t)millis());
			}
			packetQueueTail = nextIndex;
			lastPacketSendTime = 0;
		}
	}
}

void queuePacket(CAN_frame_t *packet, bool shortGap=false) {
	uint16_t nextIndex = (packetQueueHead + 1) % sendQueueSize;

	while (nextIndex == packetQueueTail) {	// wait if queue is full
		processPacketQueue();
	}

	packetQueueHead = nextIndex;
	packetQueue[packetQueueHead] = *packet;
	packetShortGap[packetQueueHead] = shortGap;
}

void initPacket(CAN_frame_t* packet, uint8_t index, uint32_t msgID) {
	packet->FIR.B.DLC = 8;
	packet->FIR.B.RTR = CAN_no_RTR;
	packet->FIR.B.FF = CAN_frame_ext;
	packet->MsgID = makeMsg(msgID);
	packet->data.u8[0] = index;
	for (auto i=1; i<8; i++) {
		packet->data.u8[i] = 0xFF;
	}
}

//////////////////////////////////////////////

void sendDCDimmerCmd(uint8_t index, uint8_t brightness, uint8_t cmd, uint8_t duration=0xFF) {
	CAN_frame_t packet;
	initPacket(&packet, index, DC_DIMMER_COMMAND_2);
	uint8_t* d = packet.data.u8;

	d[2] = min(brightness, RVCBrightMax);
	d[3] = cmd;
	d[4] = duration;
	d[5] = 0;				// no interlock

	queuePacket(&packet);

	printf("%u: Queueing Dimmer packet: #%d, bright=%d, cmd=%d, dur=%d\n", (uint32_t)millis(), index, brightness, cmd, duration);
	displayPacket(&packet, packetPrintYes);
	printf("%u: ** Packet queued.\n", (uint32_t)millis());
}

void sendOnOff(uint8_t index, bool on, uint8_t brightness=RVCBrightMax) {
	printf("%u: sendOnOff: #%d to %d\n", (uint32_t)millis(), index, on);
	sendDCDimmerCmd(index, brightness, (on) ? DCDimmerCmdOnDuration : DCDimmerCmdOff);
	// sendDCDimmerCmd(index, brightness, DCDimmerCmdToggle);
}

void sendLampLevel(uint8_t index, uint8_t brightness) {
	if (brightness == 0) {
		sendOnOff(index, false);
	}
	else {
		printf("%u: sendLampLevel: #%d to %d\n", (uint32_t)millis(), index, brightness);
		sendDCDimmerCmd(index, brightness, DCDimmerCmdSetBrightness, 0);
	}
}

void sendThermostatCommand(uint8_t index, ThermostatMode mode, FanMode fanMode, uint8_t fanSpeed, double tempC) {
	CAN_frame_t packet;
	initPacket(&packet, index, THERMOSTAT_COMMAND_1);
	uint8_t* d = packet.data.u8;

	// bits 6-7 are the schedule mode: 11b = "no change" (00b would switch the schedule off)
	d[1] = (mode & 0x0F) | ((fanMode & 0x03) << 4) | 0xC0;
	d[2] = fanSpeed;

	uint16_t tempVal = convFromTempC(tempC);
	d[4] = d[6] = tempVal >> 8;
	d[3] = d[5] = tempVal & 0xFF;

	queuePacket(&packet);

	// printf("%u: Queueing Thermostat packet: #%d, mode=%d, fanMode=%d, temp=%0.1fºC\n", (uint32_t)millis(), index, mode, fanMode, tempC);
	// displayPacket(&packet, packetPrintYes);
	// printf("%u: ** Packet queued.\n", (uint32_t)millis());
}

//////////////////////////////////////////////

const char* switchTypes[3] = { "43", "43", "49" };
const char* switchHapNames[3] = { "LightBulb", "LightBulb", "Switch" };

struct RVSwitch : SpanService {
	SpanCharacteristic *_on;
	SpanCharacteristic *_brightness = NULL;
	int16_t _index;

	RVSwitch(SwitchDeviceRec *device) : SpanService( switchTypes[device->type], switchHapNames[device->type] ) {
		_index = device->index;

		REQ(On);
		OPT(Name);

		if (device->type == Lamp || device->type == DimmableLamp) {
			OPT(Brightness);
			OPT(Hue);
			OPT(Saturation);
			OPT(ColorTemperature);
		}

		_on = new Characteristic::On();

		if (device->type == DimmableLamp) {
			_brightness = new Characteristic::Brightness(HomeKitPercentMax);
			_brightness->setRange(0, HomeKitPercentMax, 5);
		}
	}
	
	boolean update() {
		if (_on->updated() || (_brightness && _brightness->updated())) {
			if (_brightness) {
				uint16_t newLevel = _on->getNewVal() * _brightness->getNewVal() * RVCBrightMax / HomeKitPercentMax;
				sendLampLevel(_index, newLevel);
			}
			else {
				sendOnOff(_index, _on->getNewVal());
			}
		}
		return(true);  
	}

	void setLevel(uint8_t index, uint8_t dcDimmerLevel) {
		bool on = dcDimmerLevel > 0;
		uint16_t level = dcDimmerLevel * HomeKitPercentMax / RVCBrightMax;

		if (index == _index && on != _on->getVal()) {
			printf("%u: Switch #%d: on = %d\n", (uint32_t)millis(), _index, on);
			_on->setVal(on);
			lastPacketRecvTime = 0;
		}
		if (_brightness && index == _index && on && level != _brightness->getVal()) {
			printf("%u: Switch #%d: level = %d\n", (uint32_t)millis(), _index, level);
			_brightness->setVal(level);
			lastPacketRecvTime = 0;
		}
	}
};

struct RVRoofFan : Service::Fan {
	SpanCharacteristic *_active;
	int16_t _index;
	int16_t _upIndex;
	int16_t _downIndex;

	bool _fanPower = false;
	bool _lidUp = false;

	RVRoofFan(FanDeviceRec *device) : Service::Fan() {
		_index = device->index;
		_upIndex = device->upIndex;
		_downIndex = device->downIndex;
		_active = new Characteristic::Active();
	}
	
	boolean update() {
		if (_active->updated()) {
			bool newValue = _active->getNewVal();
			sendOnOff(_index, newValue);
			if (_upIndex != -1 && _downIndex!=-1) {
				if (newValue) {
					sendOnOff(_downIndex, false);
					sendOnOff(_upIndex, true);
				}
				else {
					sendOnOff(_upIndex, false);
					sendOnOff(_downIndex, true);
				}
				_lidUp = newValue;
			}
			_fanPower = newValue;
		}

		return(true);  
	}
	
	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;
		bool changed = false;

		if (index == _index) {
			_fanPower = on;
			changed = true;
			lastPacketRecvTime = 0;
		}
		else if (index == _upIndex && on) {
			_lidUp = true;
			changed = true;
			lastPacketRecvTime = 0;
		}
		else if (index == _downIndex && on) {
			_lidUp = false;
			changed = true;
			lastPacketRecvTime = 0;
		}

		if (changed) {
			bool newState = _fanPower && (_upIndex==-1 || _lidUp);

			if (newState != _active->getVal()) {
				printf("%u: Fan #%d: active = %d\n", (uint32_t)millis(), _index, newState);
				_active->setVal(newState);
			}
		}
	}
};

// HomeKit thermostat values
enum {
	currentFanStateInactive = 0,
	currentFanStateIdle,
	currentFanStateBlowing,

	targetFanStateManual = 0,
	targetFanStateAuto,

	heatingCoolingStateOff = 0,
	heatingCoolingStateHeat,
	heatingCoolingStateCool
};

struct RVHVACFan : Service::Fan {
	SpanCharacteristic *_active;
	SpanCharacteristic *_currentState;
	SpanCharacteristic *_targetState;
	SpanCharacteristic *_speed;
	
	std::function<bool()> _updateFunction;

	int16_t _index;
	int16_t _fanHIndex;
	int16_t _fanLIndex;
	bool _fanHRunning = false;
	bool _fanLRunning = false;

	RVHVACFan(ThermostatDeviceRec *device, std::function<bool()> updateFunction) : Service::Fan() {
		_active = new Characteristic::Active(false);
		_speed = new Characteristic::RotationSpeed(0);
		_speed->setRange(0, HomeKitPercentMax, HomeKitPercentMax/2);
		_currentState = new Characteristic::CurrentFanState(currentFanStateIdle);
		_targetState = new Characteristic::TargetFanState(targetFanStateAuto);
	
		_index = device->coolingInstance;
		_fanHIndex = device->fanHIndex;
		_fanLIndex = device->fanLIndex;
		_updateFunction = updateFunction;
	}
	
	boolean update() {
		bool changed = false;

		if (_active->updated()) {
			printf("%u: RVHVACFan #%d - Active: %d\n", (uint32_t)millis(), _index, _active->getNewVal());
			changed = true;
		}
		if (_speed->updated()) {
			printf("%u: RVHVACFan #%d - Speed: %d\n", (uint32_t)millis(), _index, _speed->getNewVal());
			changed = true;
		}
		if (_targetState->updated()) {
			printf("%u: RVHVACFan #%d - Target State: %d\n", (uint32_t)millis(), _index, _targetState->getNewVal());
		}
		if (changed) {
			_updateFunction();
		}
		return(true);  
	}

	void setModeSpeed(FanMode fanMode, uint8_t rvcSpeed) {
		bool newActive = fanMode == FanModeOn;

		if (newActive != _active->getVal()) {
			printf("%u: RVHVACFan #%d - setActive: %d\n", (uint32_t)millis(), _index, newActive);
			_active->setVal(newActive);
		}
		if (rvcSpeed > RVCPercentMax) {		// "not available" - keep the last known speed
			return;
		}

		// Same scale as getModeSpeed() (200 = 100%), snapped to the 50% steps HomeKit was told about
		constexpr uint8_t speedStep = HomeKitPercentMax / 2;
		uint8_t percent = min(rvcSpeed, RVCFanMax) * HomeKitPercentMax / RVCFanMax;
		uint8_t speed = ((percent + speedStep / 2) / speedStep) * speedStep;

		if (speed != _speed->getVal()) {
			printf("%u: RVHVACFan #%d - setSpeed: %d\n", (uint32_t)millis(), _index, speed);
			_speed->setVal(speed);
		}
	}

	void getModeSpeed(FanMode* fanMode, uint8_t *speed) {
		if (_active->getNewVal()) {
			*fanMode = FanModeOn;
			*speed = _speed->getNewVal() * RVCFanMax / HomeKitPercentMax;
		}
		else {
			*fanMode = FanModeAuto;
			*speed = 0;
		}
	}

	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;
		bool changed = false;

		if (index == _fanHIndex && on != _fanHRunning) {
			_fanHRunning = on;
			changed = true;
			lastPacketRecvTime = 0;
		}
		else if (index == _fanLIndex && on != _fanLRunning) {
			_fanLRunning = on;
			changed = true;
			lastPacketRecvTime = 0;
		}

		if (changed) {
			uint8_t newState = (_fanHRunning || _fanLRunning) ? currentFanStateBlowing : currentFanStateIdle;

			if (newState != _currentState->getVal()) {
				printf("%u: HVACFan #%d: currentState = %d\n", (uint32_t)millis(), _index, newState);
				_currentState->setVal(newState);
			}
		}
	}
};

struct RVThermostat : Service::Thermostat {
	SpanCharacteristic *_ambientTemp;
	SpanCharacteristic *_targetTemp;
	SpanCharacteristic *_targetState;
	SpanCharacteristic *_currentState;

	RVHVACFan *_fan;

	int16_t _coolingInstance;
	int16_t _compressorIndex;
	ThermostatMode _coolingMode = ThermostatModeOff;

	int16_t _furnaceInstance;
	int16_t _combustionIndex;
	ThermostatMode _furnaceMode = ThermostatModeOff;
	bool _separateFurnace;				// the furnace is a different thermostat instance than cooling

	double _coolSetpoint = NAN;			// cool setpoint reported by the cooling instance
	double _heatSetpoint = NAN;			// heat setpoint reported by the cooling instance
	double _furnaceSetpoint = NAN;		// heat setpoint reported by a separate furnace instance

	static constexpr double targetMinC = 10.0;
	static constexpr double targetMaxC = 32.0;
	static constexpr double ambientMinC = -40.0;
	static constexpr double ambientMaxC = 100.0;

	bool _compressorRunning = false;
	bool _furnaceRunning = false;

	RVThermostat(ThermostatDeviceRec *device) : Service::Thermostat() {
		_ambientTemp = new Characteristic::CurrentTemperature(20);
		_ambientTemp->setRange(ambientMinC, ambientMaxC, 0.1);		// the HomeKit default bottoms out at 0ºC
		_targetTemp = new Characteristic::TargetTemperature();
		_targetTemp->setRange(targetMinC, targetMaxC, (1.0 / 1.8))->setVal(20);
		_currentState = new Characteristic::CurrentHeatingCoolingState(heatingCoolingStateOff);
		_targetState = new Characteristic::TargetHeatingCoolingState(heatingCoolingStateOff);
		new Characteristic::TemperatureDisplayUnits(homeKitTemperatureDisplayFahrenheit);

		if (device->furnaceInstance != -1) {
			_targetState->setValidValues(3, heatingCoolingStateOff, heatingCoolingStateHeat, heatingCoolingStateCool);
		}
		else {
			_targetState->setValidValues(2, heatingCoolingStateOff, heatingCoolingStateCool);
		}

		_coolingInstance = device->coolingInstance;
		_compressorIndex = device->compressorIndex;

		_furnaceInstance = device->furnaceInstance;
		_combustionIndex = device->combustionIndex;
		_separateFurnace = _furnaceInstance != -1 && _furnaceInstance != _coolingInstance;

		_fan = new RVHVACFan(device, [this]()->bool { updateThermostat(); return true; });
	}

	void updateThermostat() {
		printf("%u: Thermostat #%d: Send thermostat info\n", (uint32_t)millis(), _coolingInstance);
		const ThermostatMode modeLookup[] { ThermostatModeOff, ThermostatModeHeat, ThermostatModeCool };
		uint8_t targetState = _targetState->getNewVal();
		ThermostatMode opMode = (targetState < 3) ? modeLookup[targetState] : ThermostatModeOff;
		double tempC = _targetTemp->getNewVal<double>();
		FanMode fanMode;
		uint8_t speed;

		_fan->getModeSpeed(&fanMode, &speed);

		if (opMode == ThermostatModeOff && fanMode == FanModeOn) {
			opMode = ThermostatModeFanOnly;
		}

		if (_separateFurnace && opMode==ThermostatModeHeat) {
			if (_coolingMode != ThermostatModeOff) {
				sendThermostatCommand(_coolingInstance, ThermostatModeOff, FanModeAuto, 0xFF, tempC);
			}
			sendThermostatCommand(_furnaceInstance, opMode, FanModeAuto, 0xFF, tempC);
		}
		else {
			// Also the path for a coach where one thermostat instance does both heating and cooling
			if (_separateFurnace && _furnaceMode != ThermostatModeOff) {
				sendThermostatCommand(_furnaceInstance, ThermostatModeOff, FanModeNA, 0xFF, tempC);
			}
			sendThermostatCommand(_coolingInstance, opMode, fanMode, speed, tempC);
		}
	}

	static bool isHeatMode(ThermostatMode mode) {
		return mode == ThermostatModeHeat || mode == ThermostatModeAuxHeat;
	}

	boolean update() {
		bool sendInfo = false;

		if (_targetState->updated()) {
			printf("%u: RVThermostat #%d, Target State: %d\n", (uint32_t)millis(), _coolingInstance, _targetState->getNewVal());
			sendInfo = true;
		}
		if (_targetTemp->updated()) {
			printf("%u: RVThermostat #%d - Target Temp: %f\n", (uint32_t)millis(), _coolingInstance, _targetTemp->getNewVal<double>());
			sendInfo = true;
		}
		if (sendInfo) {
			updateThermostat();
		}

		return(true);  
	}

	void setLevel(uint8_t index, uint8_t level) {
		bool on = level > 0;
		bool changed = false;

		if (index == _compressorIndex && on != _compressorRunning) {
			printf("%u: Thermostat #%d: compressor = %d\n", (uint32_t)millis(), index, on);
			_compressorRunning = on;
			changed = true;
		}
		if (index == _combustionIndex && on != _furnaceRunning) {
			printf("%u: Thermostat #%d: furnace = %d\n", (uint32_t)millis(), index, on);
			_furnaceRunning = on;
			changed = true;
		}
		if (changed) {
			uint8_t newState;

			if (_furnaceRunning) {
				newState = heatingCoolingStateHeat;
			}
			else if (_compressorRunning) {
				newState = heatingCoolingStateCool;
			}
			else {
				newState = heatingCoolingStateOff;
			}
			if (newState != _currentState->getVal()) {
				_currentState->setVal(newState);
				lastPacketRecvTime = 0;
			}
		}
		_fan->setLevel(index, level);
	}

	void setAmbientTemp(uint8_t index, double tempC) {
		if (index != _coolingInstance || isnan(tempC)) {
			return;
		}
		tempC = constrain(tempC, ambientMinC, ambientMaxC);

		if (fabs(tempC - _ambientTemp->getVal<double>()) > 0.2) {
			printf("%u: Set ambient temp #%d: %0.1fºC\n", (uint32_t)millis(), _coolingInstance, tempC);
			_ambientTemp->setVal(tempC);
			lastPacketRecvTime = 0;
		}
	}

	void setInfo(uint8_t index, ThermostatMode opMode, FanMode fanMode, uint8_t fanSpeed, double heatTemp, double coolTemp) {
		if (index == _coolingInstance) {
			_coolingMode = opMode;
			if (!isnan(coolTemp)) {
				_coolSetpoint = coolTemp;
			}
			if (!isnan(heatTemp)) {
				_heatSetpoint = heatTemp;
			}
			_fan->setModeSpeed(fanMode, fanSpeed);
		}
		else if (_separateFurnace && index == _furnaceInstance) {
			_furnaceMode = opMode;
			if (!isnan(heatTemp)) {
				_furnaceSetpoint = heatTemp;
			}
		}
		else {
			return;
		}

		// Work out the HomeKit target state from what both instances report
		bool furnaceHeating = _separateFurnace && isHeatMode(_furnaceMode);
		uint8_t mode = heatingCoolingStateOff;

		if (furnaceHeating) {
			mode = heatingCoolingStateHeat;
		}
		else if (_coolingMode == ThermostatModeCool) {
			mode = heatingCoolingStateCool;
		}
		else if (isHeatMode(_coolingMode) && _furnaceInstance != -1) {	// "Heat" is only a valid HomeKit value when a furnace is defined
			mode = heatingCoolingStateHeat;
		}

		if (mode != _targetState->getVal()) {
			printf("%u: Thermostat #%d: targetState = %d\n", (uint32_t)millis(), index, mode);
			_targetState->setVal(mode);
			lastPacketRecvTime = 0;
		}

		// The target temperature follows the setpoint of whichever side is in use
		double target = _coolSetpoint;

		if (mode == heatingCoolingStateHeat) {
			double heatTarget = (furnaceHeating) ? _furnaceSetpoint : _heatSetpoint;

			if (!isnan(heatTarget)) {
				target = heatTarget;
			}
		}

		if (!isnan(target)) {
			target = constrain(target, targetMinC, targetMaxC);

			if (fabs(target - _targetTemp->getVal<double>()) > 0.2) {
				printf("%u: Thermostat #%d: targetTemp = %f\n", (uint32_t)millis(), index, target);
				_targetTemp->setVal(target);
				lastPacketRecvTime = 0;
			}
		}
	}
};

//////////////////////////////////////////////

inline float degCfromDegF(float degF) {
	return degF / 1.8;
}

inline float tempCfromTempF(float tempF) {
	return degCfromDegF(tempF - 32.0);
}

struct RVBattery : Service::Thermostat {
	SpanCharacteristic *_ambientTemp;
	SpanCharacteristic *_targetTemp;
	SpanCharacteristic *_targetState;
	SpanCharacteristic *_currentState;

	int16_t _instance;
	uint8_t _sourcePriority = 0;
	uint32_t _sourcePriorityTime = 0;

	RVBattery(int16_t instance) : Service::Thermostat() {
		_instance = instance;
		_ambientTemp = new Characteristic::CurrentTemperature(tempCfromTempF(100));
		_ambientTemp->setRange(tempCfromTempF(100), tempCfromTempF(160), degCfromDegF(1.0));
		_targetTemp = new Characteristic::TargetTemperature(tempCfromTempF(100));
		_targetTemp->setRange(tempCfromTempF(100), tempCfromTempF(160), degCfromDegF(1.0));

		_currentState = new Characteristic::CurrentHeatingCoolingState(heatingCoolingStateOff);
		_targetState = new Characteristic::TargetHeatingCoolingState(heatingCoolingStateOff);

		new Characteristic::TemperatureDisplayUnits(homeKitTemperatureDisplayFahrenheit);
		_targetState->setValidValues(2, heatingCoolingStateOff, heatingCoolingStateCool);

	}

	void loop() {
		if (_targetState->getVal() != heatingCoolingStateOff) {
			_targetState->setVal((int)heatingCoolingStateOff);
		}
	}

	// Several devices can report the same DC source; RV-C says to believe the one with the highest
	// priority. A lower priority reading is only accepted once the better source has gone quiet.
	void setVoltage(uint8_t index, float voltage, uint8_t priority=0xFF) {
		constexpr uint32_t sourceTimeoutMS = 10000;
		uint32_t now = millis();

		if (index != _instance) {
			return;
		}
		if (priority < _sourcePriority && (now - _sourcePriorityTime) < sourceTimeoutMS) {
			return;
		}
		_sourcePriority = priority;
		_sourcePriorityTime = now;

		// Displayed as a "temperature" of volts*10 ºF: keep it inside the range HomeKit was given
		float voltageAsTemp = tempCfromTempF(constrain(voltage * 10.0f, 100.0f, 160.0f));

		if (fabs(voltageAsTemp - _ambientTemp->getVal<float>()) > degCfromDegF(0.4)) {
			printf("%u: Set voltage #%d: %0.1fV\n", (uint32_t)millis(), _instance, voltage);
			_ambientTemp->setVal(voltageAsTemp);
			lastPacketRecvTime = 0;
		}
	}
};

constexpr auto maxBatteries = 2;
uint16_t batteryCount = 0;

RVBattery* batteries[maxBatteries];

void setBatteryVoltage(uint8_t index, float voltage, uint8_t priority=0xFF) {
	for (auto i=0; i<batteryCount; i++) {
		batteries[i]->setVoltage(index, voltage, priority);
	}
}

//////////////////////////////////////////////

constexpr uint32_t endStopExtraTimeMS = 500;

struct RVAwning : Service::WindowCovering {
	SpanCharacteristic* _currentPosition = NULL;
	SpanCharacteristic* _targetPosition = NULL;

	ShadeState _shadeState = 0;
	float _currentShadePosition = homeKitShadeOpenValue;
	uint64_t _nextPositionUpdateTime = 0;

	uint64_t _operationEndTime = 0;
	uint64_t _lastLoopTime = 0;			// per awning - a function static would be shared by all of them

	uint8_t _retractIndex;
	uint8_t _extendIndex;
	uint32_t _openTimeMS;
	uint32_t _closeTimeMS;

	RVAwning(AwningDeviceRec* device) : Service::WindowCovering() {
		_currentPosition = new Characteristic::CurrentPosition(homeKitShadeOpenValue);
		_targetPosition = new Characteristic::TargetPosition(homeKitShadeOpenValue);
		_retractIndex = device->retractIndex;
		_extendIndex = device->extendIndex;
		_openTimeMS = max(device->retractTime, (uint32_t)1);		// used as divisors
		_closeTimeMS = max(device->extendTime, (uint32_t)1);
	}

	boolean update() {
		if (_targetPosition->updated()) {
			uint64_t curTime = millis64();

			float targetValue = _targetPosition->getNewVal<float>();
			float moveAmount = targetValue - _currentShadePosition;
			uint32_t moveTimeMS = fabs(moveAmount / HomeKitPercentMax) * ((moveAmount > 0) ? _openTimeMS : _closeTimeMS);

			_operationEndTime = curTime + moveTimeMS;

			if (targetValue == homeKitShadeClosedValue || targetValue == homeKitShadeOpenValue) {
				_operationEndTime += endStopExtraTimeMS;
			}

			_shadeState = shadeStateHomeKitAction | ((moveAmount > 0) ? shadeStateOpening : shadeStateClosing);

			printf("%u: Changing position from %0.1f%% to %0.1f%% over %dmS.\n", (uint32_t)millis(), _currentShadePosition, targetValue, (int)(_operationEndTime-curTime));
			printf("%u: State changed to 0x%02X\n", (uint32_t)millis(), _shadeState);
		}
		return true;
	}

	void loop() {
		uint64_t &lastTime = _lastLoopTime;
		uint64_t curTime = millis64();

		if (lastTime == 0) {
			lastTime = curTime;
		}

		if (curTime > lastTime) {
			bool needPositionUpdate = false;
			bool needTargetUpdate = false;

			// if (_userOpenButton->pressed()) {
			// 	_shadeState = shadeStateUserAction | shadeStateOpening;
			// }
			// else if (_userCloseButton->pressed()) {
			// 	_shadeState = shadeStateUserAction | shadeStateClosing;
			// }
			// else if (_shadeState & shadeStateUserAction) {
			// 	_shadeState = 0;
			// 	needTargetUpdate = true;
			// }

			bool moving = (_shadeState & (shadeStateOpening | shadeStateClosing)) != 0;
			bool movingOpen = (_shadeState & shadeStateOpening) != 0;

			if (moving) {
				// percent per millisecond: positive while opening, negative while closing
				float rate = (movingOpen) ? ((float)HomeKitPercentMax / _openTimeMS) : -((float)HomeKitPercentMax / _closeTimeMS);

				_currentShadePosition += (curTime - lastTime) * rate;
				_currentShadePosition = max(homeKitShadeClosedValue, min(homeKitShadeOpenValue, _currentShadePosition));

				if (_shadeState & shadeStateHomeKitAction && curTime > _operationEndTime) {
					_currentShadePosition = _targetPosition->getVal<float>();
					_currentPosition->setVal(_currentShadePosition);	// otherwise HomeKit shows "opening..." forever
					_shadeState = 0;
					printf("%u: Homekit operation complete.\n", (uint32_t)millis());
				}
			}

			// OutputState newOutputState;

			// if (_shadeState & shadeStateHomeKitAction) {
			// 	newOutputState = (_shadeState & shadeStateOpening) ? outputStateOpen : outputStateClose;
			// }
			// else {
			// 	newOutputState = outputStateIdle;
			// }

			// if (newOutputState != _outputState) {
			// 	SerPrintf("Output state changed to %d\n", newOutputState);

			// 	_controlPin->clearDelayedState();
			// 	_directionPin->clearDelayedState();

			// 	bool newDirectionOutputState;
			// 	bool newControlOutputState;

			// 	if (newOutputState == outputStateOpen) {
			// 		newDirectionOutputState = LOW;
			// 		newControlOutputState = HIGH;
			// 	}
			// 	else if (newOutputState == outputStateClose) {
			// 		newDirectionOutputState = HIGH;
			// 		newControlOutputState = HIGH;
			// 	}
			// 	else {
			// 		newDirectionOutputState = LOW;
			// 		newControlOutputState = LOW;
			// 	}

			// 	if (newDirectionOutputState != _directionPin->state() && newControlOutputState != _controlPin->state()) {
			// 		if (newControlOutputState == HIGH) {		// direction  needs time to settle before setting control
			// 			_controlPin->setState(newControlOutputState, relaySettlingTimeMS);
			// 			_directionPin->setState(newDirectionOutputState, 0);
			// 		}
			// 		else {									// direction needs to hold while control settles
			// 			_controlPin->setState(newControlOutputState, 0);
			// 			_directionPin->setState(newDirectionOutputState, relaySettlingTimeMS);
			// 		}
			// 	}
			// 	else {
			// 		_controlPin->setState(newControlOutputState, 0);
			// 		_directionPin->setState(newDirectionOutputState, 0);
			// 	}

			// 	_outputState = newOutputState;
			// 	needPositionUpdate = true;
			// }

			// if (needPositionUpdate || needTargetUpdate || (_nextPositionUpdateTime>0 && curTime > _nextPositionUpdateTime)) {
			// 	if (needTargetUpdate || _shadeState & shadeStateUserAction) {
			// 		float nextPosition = _currentShadePosition;
					
			// 		if (_shadeState & shadeStateOpening) {
			// 			 nextPosition = min(homeKitShadeOpenValue, _currentShadePosition + openPerMS * updateTimeMS);
			// 		}
			// 		else if (_shadeState & shadeStateClosing) {
			// 			 nextPosition = max(homeKitShadeClosedValue, _currentShadePosition - closePerMS * updateTimeMS);
			// 		}

			// 		if (_targetPosition->getVal<float>() != nextPosition) {
			// 			_targetPosition->setVal(nextPosition);
			// 			SerPrintf("Target changed to %0.1f%%\n", nextPosition);
			// 		}
			// 	}
			// 	if (_currentPosition->getVal<float>() != _currentShadePosition) {
			// 		_currentPosition->setVal(_currentShadePosition);
			// 		SerPrintf("Position changed to %0.1f%% - output=%d\n", _currentShadePosition, _outputState);
			// 	}
			// 	_nextPositionUpdateTime = 0;
			// }

			// if (moving && _nextPositionUpdateTime == 0) {
			// 	_nextPositionUpdateTime = curTime + updateTimeMS;
			// }

			// _controlPin->update(curTime);
			// _directionPin->update(curTime);

			// updateIndicator(_shadeState, _currentShadePosition);

			lastTime = curTime;
		}
	}

	void setLevel(uint8_t index, uint8_t dcDimmerLevel) {
		bool on = dcDimmerLevel > 0;
		uint16_t level = dcDimmerLevel * HomeKitPercentMax / RVCBrightMax;

		if (index == _retractIndex) {
			printf("%u: Awning #%d: retract = %d\n", (uint32_t)millis(), index, on);
			lastPacketRecvTime = 0;
		}
		else if (index == _extendIndex) {
			printf("%u: Awning #%d: extend = %d\n", (uint32_t)millis(), index, on);
			lastPacketRecvTime = 0;
		}
	}
};

//////////////////////////////////////////////
#ifndef HAVE_AWNINGS
const AwningDeviceRec awningList[] = {
};
#endif

constexpr uint16_t maxSwitches = sizeof(switchList) / sizeof(SwitchDeviceRec);
constexpr uint16_t maxFans = sizeof(fanList) / sizeof(FanDeviceRec);
constexpr uint16_t maxThermostats = sizeof(thermostatList) / sizeof(ThermostatDeviceRec);
constexpr uint16_t maxAwnings = sizeof(awningList) / sizeof(AwningDeviceRec);

uint16_t switchCount = 0;
uint16_t fanCount = 0;
uint16_t thermostatCount = 0;
uint16_t awningCount = 0;

RVSwitch* switches[maxSwitches];
RVRoofFan* fans[maxFans];
RVThermostat* thermostats[maxThermostats];
RVAwning* awnings[maxAwnings];

void setSwitchLevel(uint8_t index, uint8_t level) {
	for (auto i=0; i<switchCount; i++) {
		switches[i]->setLevel(index, level);
	}
	for (auto i=0; i<fanCount; i++) {
		fans[i]->setLevel(index, level);
	}
	for (auto i=0; i<thermostatCount; i++) {
		thermostats[i]->setLevel(index, level);
	}
	for (auto i=0; i<awningCount; i++) {
		awnings[i]->setLevel(index, level);
	}
}

void setAmbientTemp(uint8_t index, double tempC) {
	for (auto i=0; i<thermostatCount; i++) {
		thermostats[i]->setAmbientTemp(index, tempC);
	}
}

void setThermostatInfo(uint8_t index, ThermostatMode opMode, FanMode fanMode, uint8_t fanSpeed, double heatTemp, double coolTemp) {
	for (auto i=0; i<thermostatCount; i++) {
		thermostats[i]->setInfo(index, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
	}
}

//////////////////////////////////////////////

// HomeKit identifies each accessory by its id (AID). HomeSpan normally numbers them in creation
// order, so inserting or removing a line in a device list renumbers everything after it and
// HomeKit loses the room, name and automations of those accessories. With STABLE_ACCESSORY_IDS
// the id comes from the device's own instance number instead, so the lists can be edited freely.
constexpr uint32_t aidSwitchBase = 1000;
constexpr uint32_t aidFanBase = 2000;
constexpr uint32_t aidThermostatBase = 3000;
constexpr uint32_t aidAwningBase = 4000;
constexpr uint32_t aidBatteryBase = 5000;
constexpr uint32_t aidDuplicateBase = 9000;

void beginAccessory(const char* name, uint32_t base, int16_t index) {
	uint32_t aid = 0;		// 0 = let HomeSpan number it

#ifdef STABLE_ACCESSORY_IDS
	static std::vector<uint32_t> usedAIDs;
	static uint32_t duplicateCount = 0;

	aid = base + (uint16_t)index;
	if (index < 0 || std::find(usedAIDs.begin(), usedAIDs.end(), aid) != usedAIDs.end()) {
		printf("%u: *** WARNING: \"%s\" has an invalid or already used instance number (%d) - check the device lists ***\n", (uint32_t)millis(), name, index);
		aid = aidDuplicateBase + duplicateCount++;
	}
	usedAIDs.push_back(aid);
#endif

	new SpanAccessory(aid);
		new Service::AccessoryInformation();
			new Characteristic::Identify();
			new Characteristic::Name(name);
}

void createDevices() {
	SPAN_ACCESSORY();   // create Bridge

		const char* typeNames[] = { "Lamp", "Dimmable", "Switch" };
		for (SwitchDeviceRec device : switchList) {
			printf("%u: Creating %s #%d: \"%s\"\n", (uint32_t)millis(), typeNames[device.type], device.index, device.name);
			beginAccessory(device.name, aidSwitchBase, device.index);
				switches[switchCount++] = new RVSwitch(&device);
		}

		for (FanDeviceRec fan : fanList) {
			printf("%u: Creating Fan #%d: \"%s\"\n", (uint32_t)millis(), fan.index, fan.name);
			beginAccessory(fan.name, aidFanBase, fan.index);
				fans[fanCount++] = new RVRoofFan(&fan);
		}

		for (ThermostatDeviceRec thermostat : thermostatList) {
			printf("%u: Creating Thermostat #%d: \"%s\"\n", (uint32_t)millis(), thermostat.coolingInstance, thermostat.name);
			beginAccessory(thermostat.name, aidThermostatBase, thermostat.coolingInstance);
				thermostats[thermostatCount++] = new RVThermostat(&thermostat);
		}

		for (AwningDeviceRec awning : awningList) {
			printf("%u: Creating Awning \"%s\"\n", (uint32_t)millis(), awning.name);
			beginAccessory(awning.name, aidAwningBase, awning.extendIndex);
				awnings[awningCount++] = new RVAwning(&awning);
		}

		#ifdef CREATE_BATTERIES
		beginAccessory("House - V*10", aidBatteryBase, 1);
			batteries[batteryCount++] = new RVBattery(1);
		beginAccessory("Chassis - V*10", aidBatteryBase, 2);
			batteries[batteryCount++] = new RVBattery(2);
		#endif
}

//////////////////////////////////////////////
// Bus census: a table of every DGN / source address / instance seen on the bus, plus a
// monitor that reports changes as they happen. This is how the instance numbers of a new
// coach are discovered - see docs/DISCOVERY.md.

typedef enum {
	dgnKindStatus = 0,		// periodic data (voltages, tanks, time, ...)
	dgnKindControl,			// status of something a person switches: reported when it changes
	dgnKindCommand			// a command from a panel/app: every one is reported
} DGNKind;

typedef struct {
	uint32_t dgn;
	const char* name;
	DGNKind kind;
} DGNInfo;

const DGNInfo dgnInfoList[] = {
	{ DC_DIMMER_STATUS_3, "DC_DIMMER_STATUS_3", dgnKindControl },
	{ DC_DIMMER_COMMAND_2, "DC_DIMMER_COMMAND_2", dgnKindCommand },
	{ DC_LOAD_STATUS, "DC_LOAD_STATUS", dgnKindControl },
	{ DC_LOAD_STATUS_2, "DC_LOAD_STATUS_2", dgnKindControl },
	{ DC_LOAD_COMMAND, "DC_LOAD_COMMAND", dgnKindCommand },
	{ AC_LOAD_STATUS, "AC_LOAD_STATUS", dgnKindControl },
	{ GENERIC_INDICATOR_STATUS, "GENERIC_INDICATOR_STATUS", dgnKindControl },
	{ GENERIC_INDICATOR_COMMAND, "GENERIC_INDICATOR_COMMAND", dgnKindCommand },
	{ THERMOSTAT_STATUS_1, "THERMOSTAT_STATUS_1", dgnKindControl },
	{ THERMOSTAT_STATUS_2, "THERMOSTAT_STATUS_2", dgnKindControl },
	{ THERMOSTAT_COMMAND_1, "THERMOSTAT_COMMAND_1", dgnKindCommand },
	{ THERMOSTAT_COMMAND_2, "THERMOSTAT_COMMAND_2", dgnKindCommand },
	{ THERMOSTAT_AMBIENT_STATUS, "THERMOSTAT_AMBIENT_STATUS", dgnKindStatus },
	{ AIR_CONDITIONER_STATUS, "AIR_CONDITIONER_STATUS", dgnKindControl },
	{ AIR_CONDITIONER_COMMAND, "AIR_CONDITIONER_COMMAND", dgnKindCommand },
	{ FURNACE_STATUS, "FURNACE_STATUS", dgnKindControl },
	{ FURNACE_COMMAND, "FURNACE_COMMAND", dgnKindCommand },
	{ WATERHEATER_STATUS, "WATERHEATER_STATUS", dgnKindControl },
	{ WATERHEATER_COMMAND, "WATERHEATER_COMMAND", dgnKindCommand },
	{ WATER_PUMP_STATUS, "WATER_PUMP_STATUS", dgnKindControl },
	{ WATER_PUMP_COMMAND, "WATER_PUMP_COMMAND", dgnKindCommand },
	{ AWNING_STATUS, "AWNING_STATUS", dgnKindControl },
	{ AWNING_STATUS_2, "AWNING_STATUS_2", dgnKindControl },
	{ AWNING_COMMAND, "AWNING_COMMAND", dgnKindCommand },
	{ SLIDE_STATUS, "SLIDE_STATUS", dgnKindControl },
	{ SLIDE_COMMAND, "SLIDE_COMMAND", dgnKindCommand },
	{ LEVELING_CONTROL_STATUS, "LEVELING_CONTROL_STATUS", dgnKindControl },
	{ LEVELING_CONTROL_COMMAND, "LEVELING_CONTROL_COMMAND", dgnKindCommand },
	{ GENERATOR_STATUS_1, "GENERATOR_STATUS_1", dgnKindStatus },
	{ GENERATOR_COMMAND, "GENERATOR_COMMAND", dgnKindCommand },
	{ GENERIC_CONFIGURATION_STATUS, "GENERIC_CONFIGURATION_STATUS", dgnKindStatus },
	{ DC_SOURCE_STATUS_1, "DC_SOURCE_STATUS_1", dgnKindStatus },
	{ BATTERY_STATUS, "BATTERY_STATUS (non-standard)", dgnKindStatus },
	{ TANK_STATUS, "TANK_STATUS", dgnKindStatus },
	{ DATE_TIME_STATUS, "DATE_TIME_STATUS", dgnKindStatus },
	{ INVERTER_STATUS, "INVERTER_STATUS", dgnKindStatus },
	{ INVERTER_AC_STATUS_1, "INVERTER_AC_STATUS_1", dgnKindStatus },
	{ INVERTER_TEMPERATURE_STATUS, "INVERTER_TEMPERATURE_STATUS", dgnKindStatus },
	{ CHARGER_AC_STATUS_1, "CHARGER_AC_STATUS_1", dgnKindStatus },
	{ ATS_AC_STATUS_1, "ATS_AC_STATUS_1", dgnKindStatus },
	{ ATS_AC_STATUS_3, "ATS_AC_STATUS_3", dgnKindStatus },
	{ ATS_AC_STATUS_4, "ATS_AC_STATUS_4", dgnKindStatus },
	{ ATS_STATUS, "ATS_STATUS", dgnKindStatus },
	{ DM_RV, "DM_RV", dgnKindStatus },
	{ PRODUCT_ID, "PRODUCT_ID", dgnKindStatus },
	{ ACKNOWLEDGMENT, "ACKNOWLEDGMENT", dgnKindStatus },
	{ REQUEST_FOR_DGN, "REQUEST_FOR_DGN", dgnKindStatus },
	{ ADDRESS_CLAIMED, "ADDRESS_CLAIMED", dgnKindStatus },
};

const DGNInfo* findDGNInfo(uint32_t dgn) {
	for (const DGNInfo &info : dgnInfoList) {
		if (info.dgn == dgn) {
			return &info;
		}
	}
	return NULL;
}

typedef struct {
	uint32_t dgn;				// PDU1 destination address stripped
	uint32_t count;
	uint32_t changes;
	uint32_t lastReportMS;
	uint8_t source;
	uint8_t instance;			// data byte 0 - the instance for nearly every RV-C status/command
	uint8_t dlc;
	uint8_t data[8];
} CensusEntry;

enum {
	monitorOff = 0,
	monitorControls,			// control status changes and commands
	monitorUnknown,				// ...plus changes in DGNs this code has no name for
	monitorAll					// ...plus every other change
};

constexpr uint16_t maxCensusEntries = 320;
constexpr uint32_t monitorMinReportIntervalMS = 1000;	// per entry, for the chattier monitor levels

CensusEntry census[maxCensusEntries];
uint16_t censusCount = 0;
uint32_t censusOverflow = 0;
uint32_t sourceSeenBits[8] = { 0 };
uint32_t rxQueueFullCount = 0;
bool addressConflict = false;
uint8_t monitorMode = monitorOff;

void printPacketData(const uint8_t* data, uint8_t dlc) {
	for (auto i=0; i<dlc; i++) {
		printf("%02X ", data[i]);
	}
}

void printMonitorLine(const char* tag, const CensusEntry* entry, const DGNInfo* info, const uint8_t* oldData) {
	printf("%u: [%s] %05X %s src=%d inst=%d: ", (uint32_t)millis(), tag, entry->dgn, (info) ? info->name : "(unnamed)", entry->source, entry->instance);
	printPacketData(entry->data, entry->dlc);
	if (oldData) {
		printf(" was: ");
		printPacketData(oldData, entry->dlc);
	}
	if (entry->dgn == DC_DIMMER_STATUS_3 && entry->dlc >= 3 && entry->data[2] <= RVCPercentMax) {
		printf(" level=%d%%", min(entry->data[2], RVCBrightMax) / 2);
	}
	printf("\n");
}

// PRODUCT_ID replies are longer than one frame and arrive as an INITIAL_PACKET followed by
// DATA_PACKETs. Only as much reassembly as it takes to print them.
typedef struct {
	bool active;
	uint8_t source;
	uint16_t length;
	char text[80];
} ProductIDSession;

constexpr uint8_t maxProductIDSessions = 4;
ProductIDSession productIDSessions[maxProductIDSessions];

void printProductID(uint8_t source, const char* text, uint16_t length) {
	printf("%u: Product ID src=%d (0x%02X): \"", (uint32_t)millis(), source, source);
	for (auto i=0; i<length; i++) {
		char c = text[i];
		if (c == 0 || (uint8_t)c == 0xFF) {
			break;
		}
		printf("%c", (c >= 32 && c < 127) ? c : '.');
	}
	printf("\"\n");
}

void handleProductIDPackets(uint32_t dgn, uint8_t source, const uint8_t* d, uint8_t dlc) {
	if (dgn == PRODUCT_ID) {
		printProductID(source, (const char*)d, dlc);
	}
	else if (dgn == TP_INITIAL_PACKET && dlc == 8) {
		uint32_t carriedDGN = d[5] | (d[6] << 8) | ((uint32_t)(d[7] & 0x01) << 16);

		if (carriedDGN == PRODUCT_ID) {
			ProductIDSession* session = &productIDSessions[0];

			for (ProductIDSession &candidate : productIDSessions) {
				if (!candidate.active || candidate.source == source) {
					session = &candidate;
					break;
				}
			}
			session->active = true;
			session->source = source;
			session->length = min((uint16_t)(d[1] | (d[2] << 8)), (uint16_t)sizeof(session->text));
			memset(session->text, 0, sizeof(session->text));
		}
	}
	else if (dgn == TP_DATA_PACKET && dlc >= 2 && d[0] >= 1) {
		for (ProductIDSession &session : productIDSessions) {
			if (session.active && session.source == source) {
				uint16_t offset = (d[0] - 1) * 7;

				for (auto i=1; i<dlc && offset < session.length; i++) {
					session.text[offset++] = d[i];
				}
				if (offset >= session.length) {
					printProductID(source, session.text, session.length);
					session.active = false;
				}
				break;
			}
		}
	}
}

void censusRecord(CAN_frame_t *packet) {
	if (packet->FIR.B.RTR == CAN_RTR || packet->FIR.B.FF != CAN_frame_ext) {
		return;
	}

	uint32_t dgn = dgnBase(getMsgBits(packet->MsgID, 24, 17));
	uint8_t source = getMsgBits(packet->MsgID, 7, 8);
	uint8_t dlc = min((uint8_t)packet->FIR.B.DLC, (uint8_t)8);
	const uint8_t* d = packet->data.u8;

	sourceSeenBits[source >> 5] |= 1UL << (source & 31);

	if (source == sourceAddress && !addressConflict) {
		addressConflict = true;
		printf("%u: *** WARNING: another device on the bus is using source address %d. Change sourceAddress in config.h! ***\n", (uint32_t)millis(), sourceAddress);
	}

	if (dgn == PRODUCT_ID || dgn == TP_INITIAL_PACKET || dgn == TP_DATA_PACKET) {
		handleProductIDPackets(dgn, source, d, dlc);
		if (dgn != PRODUCT_ID) {
			return;				// byte 0 of a transport packet is not an instance
		}
	}

	const DGNInfo* info = findDGNInfo(dgn);
	DGNKind kind = (info) ? info->kind : dgnKindStatus;
	uint8_t instance = (dlc > 0) ? d[0] : 0xFF;
	uint32_t now = millis();

	bool wanted = (monitorMode == monitorAll) ||
				  (monitorMode >= monitorControls && kind != dgnKindStatus) ||
				  (monitorMode >= monitorUnknown && info == NULL);

	for (auto i=0; i<censusCount; i++) {
		CensusEntry* entry = &census[i];

		if (entry->dgn == dgn && entry->source == source && entry->instance == instance) {
			bool changed = entry->dlc != dlc || memcmp(entry->data, d, dlc) != 0;
			uint8_t oldData[8];

			entry->count++;
			if (changed) {
				memcpy(oldData, entry->data, sizeof(oldData));
				memcpy(entry->data, d, dlc);
				entry->dlc = dlc;
				entry->changes++;
			}

			// Commands are events: two presses of a "toggle" button are identical frames.
			if (wanted && (changed || kind == dgnKindCommand)) {
				if (kind != dgnKindStatus || (now - entry->lastReportMS) >= monitorMinReportIntervalMS) {
					printMonitorLine((kind == dgnKindCommand) ? "CMD" : "CHG", entry, info, (changed) ? oldData : NULL);
					entry->lastReportMS = now;
				}
			}
			return;
		}
	}

	if (censusCount >= maxCensusEntries) {
		censusOverflow++;
		return;
	}

	CensusEntry* entry = &census[censusCount++];

	memset(entry, 0, sizeof(CensusEntry));
	entry->dgn = dgn;
	entry->source = source;
	entry->instance = instance;
	entry->dlc = dlc;
	entry->count = 1;
	entry->lastReportMS = now;
	memcpy(entry->data, d, dlc);

	if (wanted) {
		printMonitorLine((kind == dgnKindCommand) ? "CMD" : "NEW", entry, info, NULL);
	}
}

void cmdCensus(const char *buff) {
	static uint16_t order[maxCensusEntries];

	for (auto i=0; i<censusCount; i++) {		// insertion sort by dgn, instance, source
		CensusEntry* entry = &census[i];
		int16_t j = i - 1;

		while (j >= 0) {
			CensusEntry* other = &census[order[j]];
			bool after = (other->dgn != entry->dgn) ? (other->dgn > entry->dgn) :
						 (other->instance != entry->instance) ? (other->instance > entry->instance) : (other->source > entry->source);
			if (!after) {
				break;
			}
			order[j + 1] = order[j];
			j--;
		}
		order[j + 1] = i;
	}

	printf("\nDGN    Name                          Inst  Src   Count Changes  Last data\n");
	for (auto i=0; i<censusCount; i++) {
		CensusEntry* entry = &census[order[i]];
		const DGNInfo* info = findDGNInfo(entry->dgn);

		printf("%05X  %-28s  %4d  %3d  %6u  %6u  ", entry->dgn, (info) ? info->name : "(unnamed)", entry->instance, entry->source, entry->count, entry->changes);
		printPacketData(entry->data, entry->dlc);
		printf("\n");
	}

	printf("\n%d entries", censusCount);
	if (censusOverflow) {
		printf(" (table full: %u packets not recorded)", censusOverflow);
	}
	printf(". Source addresses seen:");
	for (auto source=0; source<256; source++) {
		if (sourceSeenBits[source >> 5] & (1UL << (source & 31))) {
			printf(" %d", source);
		}
	}
	printf("\nReceive queue found full %u times (packets were probably lost each time).\n", rxQueueFullCount);
	if (addressConflict) {
		printf("*** Another device is using our source address %d - change sourceAddress in config.h ***\n", sourceAddress);
	}
	printf("\n");
}

void cmdCensusClear(const char *buff) {
	censusCount = 0;
	censusOverflow = 0;
	rxQueueFullCount = 0;
	memset(sourceSeenBits, 0, sizeof(sourceSeenBits));
	printf("%u: Census cleared.\n", (uint32_t)millis());
}

void cmdSetMonitor(const char *buff) {
	const char* labels[] = { "Off", "Controls and commands", "Controls, commands and unnamed DGNs", "Everything" };

	buff++;
	while (buff[0]==' ') buff++;

	if (buff[0] >= '0' && buff[0] <= '3') {
		monitorMode = buff[0] - '0';
		printf("%u: Change monitor: %s.\n", (uint32_t)millis(), labels[monitorMode]);
	}
	else {
		printf("%u: cmdSetMonitor: parameter error!\n", (uint32_t)millis());
	}
}

// Ask every device that has been heard to identify itself (make*model*serial*unit).
// PRODUCT_ID may not be requested globally, so each address is asked in turn.
void cmdIdentify(const char *buff) {
	uint16_t requests = 0;

	for (auto dest=0; dest<254; dest++) {
		if (dest != sourceAddress && (sourceSeenBits[dest >> 5] & (1UL << (dest & 31)))) {
			CAN_frame_t packet;
			initPacket(&packet, PRODUCT_ID & 0xFF, REQUEST_FOR_DGN | dest);
			packet.data.u8[1] = (PRODUCT_ID >> 8) & 0xFF;
			packet.data.u8[2] = (PRODUCT_ID >> 16) & 0xFF;
			queuePacket(&packet);
			requests++;
		}
	}
	printf("%u: Requested Product ID from %d devices%s.\n", (uint32_t)millis(), requests, (doCANWrite) ? "" : " - but CAN-Bus writes are disabled, see '@w'");
}

//////////////////////////////////////////////

const char* getValuePair(const char* buff, int16_t* val1, int16_t* val2) {
	*val1 = -1;
	*val2 = -2;

	const char* equals = strchr(buff, '=');

	if (equals) {
		equals += 1;
		*val1 = atoi(buff);
		*val2 = atoi(equals);

		buff = strchr(equals, ',');
		if (buff) {
			buff += 1;
		}
	}
	else {
		buff = NULL;
	}

	return buff;
}

const char* getNextValue(const char* buff, int16_t *val) {
	*val = -1;
	if (buff && buff[0]) {
		*val = atoi(buff);
		buff = strchr(buff, ',');
		if (buff) {
			buff += 1;
		}
	}
	else {
		buff = NULL;
	}
	return buff;
}

void cmdSet(const char *buff, uint8_t multiplier, const char* label) {
	int16_t index;
	int16_t val;

	buff += 1;
	
	while (buff) {
		buff = getValuePair(buff, &index, &val);

		if (index>=0 && index<=RVCInstanceMax && val>=0 && val*multiplier<=RVCPercentMax) {
			printf("%u: %s: index=%d, val=%d\n", (uint32_t)millis(), label, index, val);
			setSwitchLevel(index, val*multiplier);
		}
		else {
			printf("%u: %s: parameter error!\n", (uint32_t)millis(), label);
		}
	}
}

void cmdSetLevel(const char *buff) {
	cmdSet(buff, 1, "cmdSetLevel");
}

void cmdSetState(const char *buff) {
	cmdSet(buff, RVCBrightMax, "cmdSetState");
}

void cmdSetAmbient(const char *buff) {
	int16_t index;
	int16_t tempF;

	buff += 1;
	
	while (buff) {
		buff = getValuePair(buff, &index, &tempF);

		if (index!=-1 && tempF!=-1) {
			double tempC = (tempF - 32.0) / 1.8;
			printf("%u: cmdSetAmbient: index=%d, tempF=%dºF (%0.1fºC)\n", (uint32_t)millis(), index, tempF, tempC);
			setAmbientTemp(index, tempC);
		}
		else {
			printf("%u: cmdSetAmbient: parameter error!\n", (uint32_t)millis());
		}
	}
}

void cmsSetThermostat(const char *buff) {
	int16_t index;
	int16_t opMode;
	int16_t tempF;
	int16_t fanMode;
	int16_t fanSpeed;

	buff += 1;
	
	buff = getValuePair(buff, &index, &opMode);
	buff = getNextValue(buff, &tempF);
	buff = getNextValue(buff, &fanMode);
	buff = getNextValue(buff, &fanSpeed);

	if (fanMode == -1) {
		fanMode = FanModeAuto;
	}
	if (fanSpeed == -1) {
		fanSpeed = 0;
	}

	if (index>=0 && index<=RVCInstanceMax && opMode>=0 && opMode<=ThermostatModeNA && fanMode>=0 && fanMode<=FanModeNA && fanSpeed>=0 && fanSpeed<=255 && tempF!=-1) {
		double tempC = (tempF - 32.0) / 1.8;
		printf("%u: cmsSetThermostat: index=%d, mode=%d, temp=%dºF (%0.1fºC), fanMode=%d, fanSpeed=%d\n", (uint32_t)millis(), index, opMode, tempF, tempC, fanMode, fanSpeed);
		setThermostatInfo(index, (ThermostatMode)opMode, (FanMode)fanMode, fanSpeed, tempC, tempC);
	}
	else {
		printf("%u: cmsSetThermostat: parameter error!\n", (uint32_t)millis());
	}
}

void cmdSendOnOff(const char *buff) {
	int16_t index;
	int16_t val;

	buff += 1;
	
	while (buff) {
		buff = getValuePair(buff, &index, &val);

		if (index>=0 && index<=RVCInstanceMax && (val==0 || val==1)) {
			printf("%u: cmdSendOnOff: index=%d, val=%d\n", (uint32_t)millis(), index, val);
			sendOnOff(index, val);
		}
		else {
			printf("%u: cmdSendOnOff: parameter error!\n", (uint32_t)millis());
		}
	}
}

void cmdSetCANWrite(const char *buff) {
	buff++;
	while (buff[0]==' ') buff++;

	switch (buff[0]) {
		case '0':
			doCANWrite = false;
			printf("%u: CAN-Bus packet writes DISABLED.\n", (uint32_t)millis());
			break;
		case '1':
			doCANWrite = true;
			printf("%u: CAN-Bus packet writes ENABLED.\n", (uint32_t)millis());
			break;
		default:
			printf("%u: cmdSetCANWrite: parameter error!\n", (uint32_t)millis());
	}
}

bool awningDebug = false;

void cmdSetAwningDebug(const char *buff) {
	buff++;
	while (buff[0]==' ') buff++;

	awningDebug = (buff[0] == '1');
	printf("%u: Awning packet debug: %s.\n", (uint32_t)millis(), (awningDebug) ? "On" : "Off");
}

void cmdSetPacketLog(const char *buff) {
	buff++;
	while (buff[0]==' ') buff++;

	switch (buff[0]) {
		case '0':
			packetPrintMode = packetPrintNo;
			printf("%u: Packet logging: Off.\n", (uint32_t)millis());
			break;
		case '1':
			packetPrintMode = packetPrintYes;
			printf("%u: Packet logging: On.\n", (uint32_t)millis());
			break;
		case '2':
			packetPrintMode = packetPrintIfUnknown;
			printf("%u: Packet logging: If Unknown.\n", (uint32_t)millis());
			break;
		case '3':
			packetPrintMode = packetPrintIfKnown;
			printf("%u: Packet logging: If Known.\n", (uint32_t)millis());
			break;
		default:
			printf("%u: cmdSetPacketLog: parameter error!\n", (uint32_t)millis());
	}
}

void addCommands() {
	new SpanUserCommand('l',"<index>=<level:0-200>,... - set level of <index>", cmdSetLevel);
	new SpanUserCommand('s',"<index>=<state:0-1>,... - set state of <index>", cmdSetState);
	new SpanUserCommand('o',"<index>=<state:0-1>,... - send onOff to <index>", cmdSendOnOff);
	new SpanUserCommand('a',"<index>=<tempºF>,... - set ambient temp of <index>", cmdSetAmbient);
	new SpanUserCommand('w',"<0-1> - set CAN-Bus write enable", cmdSetCANWrite);
	new SpanUserCommand('p',"<0-3> - set packet logging: 0=No, 1=Yes, 2=If unknown, 3=If known", cmdSetPacketLog);
	new SpanUserCommand('t',"<index>=<mode:0-2>,<tempºF>,optional(<fanmode:0-1>,<fanspeed:0-200>) - set info for thermostat <index>", cmsSetThermostat);
	new SpanUserCommand('c',"- print the census of every DGN/instance/source address seen on the bus", cmdCensus);
	new SpanUserCommand('x',"- clear the census", cmdCensusClear);
	new SpanUserCommand('m',"<0-3> - monitor bus changes: 0=Off, 1=Controls+commands, 2=...+unnamed DGNs, 3=Everything", cmdSetMonitor);
	new SpanUserCommand('i',"- ask every device heard on the bus for its Product ID", cmdIdentify);
	new SpanUserCommand('d',"<0-1> - awning packet debug output", cmdSetAwningDebug);
}

//////////////////////////////////////////////
typedef struct {
	uint8_t pinNumber;
	uint8_t mode;
	uint8_t state;
} PinSetup;

PinSetup pinList[] = {
	{ indicatorPinR, OUTPUT, HIGH },
	{ indicatorPinG, OUTPUT, HIGH },
	{ indicatorPinB, OUTPUT, HIGH },
};

void flashPin(uint8_t pin, uint16_t count, uint16_t period) {
	for (auto i=0; i<count; i++) {
		digitalWrite(pin, LOW);
		delay(period/2);
		digitalWrite(pin, HIGH);
		delay(period/2);
	}
}

void initPins() {
	for (PinSetup pin : pinList) {
		pinMode(pin.pinNumber, pin.mode);
		digitalWrite(pin.pinNumber, pin.state);
	}
}

//////////////////////////////////////////////

bool wifiConnected = false;
bool hadWifiConnection = false;

void wifiStatusChanged(HS_STATUS status) {
	if (status == HS_WIFI_CONNECTING) {
		wifiConnected = false;
		if (hadWifiConnection) {
			printf("%u: WIFI: Lost Connection..\n", (uint32_t)millis());
		}
	}
}

void wifiReady() {
	wifiConnected = true;
	hadWifiConnection = true;
	printf("%u: WIFI: Ready..\n", (uint32_t)millis());
}

//////////////////////////////////////////////

void setup() {
	initPins();
	flashPin(indicatorPinB, 20, 200);

	Serial.begin(115200);
	printf("%u: RV Bridge - Startup\n", (uint32_t)millis());

	#ifdef OVERRIDE_MAC_ADDRESS
 		uint8_t newMACAddress[] = OVERRIDE_MAC_ADDRESS;
 		esp_base_mac_addr_set(&newMACAddress[0]);
 		printf("%u: MAC address updated to: %s\n", (uint32_t)millis(), WiFi.macAddress().c_str());
 	#endif

	printf("%u: Init CAN module\n", (uint32_t)millis());
	CAN_cfg.speed = CAN_SPEED_250KBPS;
	CAN_cfg.tx_pin_id = canTxPin;
	CAN_cfg.rx_pin_id = canRxPin;
	CAN_cfg.rx_queue = xQueueCreate(receiveQueueSize, sizeof(CAN_frame_t));
	ESP32Can.CANInit();

	printf("%u: Init HomeSpan\n", (uint32_t)millis());
	#ifndef SKIP_WIFI_CREDENTIALS
		homeSpan.setWifiCredentials(ssid, sspwd);
	#else
		homeSpan.enableAutoStartAP();
	#endif
	homeSpan.setSketchVersion(versionString);
	homeSpan.setWifiCallback(wifiReady);
	homeSpan.setStatusCallback(wifiStatusChanged);
	homeSpan.begin(Category::Bridges, "RV-Bridge", DEFAULT_HOST_NAME, "RV-Bridge-ESP32");

	createDevices();
	addCommands();

	// Reduce processor frequency to lower current consumption
	setCpuFrequencyMhz(160);

	#ifdef RV_DEFINITION_UNVERIFIED
		printf("%u: *** This RV definition is unverified: CAN-Bus writes are OFF. Use '@m 1' and '@c' to find the\n", (uint32_t)millis());
		printf("%u: *** real instance numbers (docs/DISCOVERY.md), then '@w 1' to enable writes. ***\n", (uint32_t)millis());
	#endif

	printf("%u: Init complete.\n", (uint32_t)millis());
}

//////////////////////////////////////////////

void processPacket(CAN_frame_t *packet) {
	// every RV-C message handled here is a full 8 byte extended frame
	if (packet->FIR.B.RTR != CAN_RTR && packet->FIR.B.FF == CAN_frame_ext && packet->FIR.B.DLC >= 8) {
		uint32_t dgn = getMsgBits(packet->MsgID, 24, 17);
		// uint8_t sourceAddr = getMsgBits(packet->MsgID, 7, 8);
		// uint8_t priority = getMsgBits(packet->MsgID, 28, 3);

		uint8_t* d = packet->data.u8;

		if (dgn == DC_DIMMER_STATUS_3) {
			uint8_t instance = d[0];
			// uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			// uint8_t enable = (d[3] >> 6) & 3;
			// uint8_t delayDuration = d[4];
			// DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			// uint8_t status = (d[6] >> 2) & 3;

			// if (instance == 4) {
			// 	displayPacket(packet, packetPrintYes);
			// }
			setSwitchLevel(instance, brightness);
		}
		else if (dgn == THERMOSTAT_AMBIENT_STATUS) {
			uint8_t instance = d[0];
			double tempC = convToTempC(d[2]<<8 | d[1]);
			
			setAmbientTemp(instance, tempC);
		}
		else if (dgn == THERMOSTAT_STATUS_1) {
			uint8_t instance = d[0];
			ThermostatMode opMode = (ThermostatMode)(d[1] & 0x0F);
			FanMode fanMode = (FanMode)((d[1]>>4) & 0x03);
			// uint8_t scheduleEnabled = ((d[1]>>6) & 0x03);
			uint8_t fanSpeed = d[2];
			double heatTemp = convToTempC(d[4]<<8 | d[3]);
			double coolTemp = convToTempC(d[6]<<8 | d[5]);
			
			setThermostatInfo(instance, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
		}
		else if (dgn == BATTERY_STATUS || dgn == DC_SOURCE_STATUS_1) {
			uint8_t instance = d[0];
			uint16_t v_int = d[3]<<8 | d[2];

			if (v_int < 0xFFFD) {
				float voltage = v_int * 0.050;
				// The proprietary message outranks any DC_SOURCE_STATUS_1 (device priority in byte 1)
				uint8_t priority = (dgn == BATTERY_STATUS) ? 0xFF : min(d[1], (uint8_t)0xFE);

				setBatteryVoltage(instance, voltage, priority);
			}
		}
	}
}

void displayPacket(CAN_frame_t *packet, uint8_t printPacket) {
	bool knownPacket = true;

	if (packet->FIR.B.RTR == CAN_RTR) {
		printf("%u: RTR from 0x%08X, DLC %d\r\n", (uint32_t)millis(), packet->MsgID,  packet->FIR.B.DLC);
	}
	else {
		uint32_t dgn = getMsgBits(packet->MsgID, 24, 17);
		uint8_t sourceAddr = getMsgBits(packet->MsgID, 7, 8);
		uint8_t priority = getMsgBits(packet->MsgID, 28, 3);

		uint8_t* d = packet->data.u8;

		if (dgn == DC_DIMMER_STATUS_3) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t enable = (d[3] >> 6) & 3;
			uint8_t delayDuration = d[4];
			DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			uint8_t status = (d[6] >> 2) & 3;

			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: DC_DIMMER_STATUS_3: inst=%d, grp=0X%02X, bright=%d, enable=%d, dur=%d, last cmd=%d, status=0X%02X\n", (uint32_t)millis(), instance, group, brightness, enable, delayDuration, lastCmd, status);				
			}
		}
		else if (dgn == DC_DIMMER_COMMAND_2) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = d[2];
			DCDimmerCmd cmd = (DCDimmerCmd)d[3];
			uint8_t delayDuration = d[4];

			// if (instance!=32 && instance!=43 && instance!=44 && instance!=53 && instance!=54 && instance!=55 && instance!=56) {
			// 	printPacket = packetPrintYes;
			// }

			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: DC_DIMMER_COMMAND_2: inst=%d, grp=0X%02X, bright=%d, cmd=0X%02X, dur=%d\n", (uint32_t)millis(), instance, group, brightness, cmd, delayDuration);	
				printPacket = packetPrintYes;			
			}
		}
		else if (dgn == THERMOSTAT_AMBIENT_STATUS) {
			uint8_t instance = d[0];
			double tempC = convToTempC(d[2]<<8 | d[1]);
			
			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: THERMOSTAT_AMBIENT_STATUS: #%d, temp=%0.1fºC\n", (uint32_t)millis(), instance, tempC);
			}
		}
		else if (dgn == THERMOSTAT_STATUS_1 || dgn == THERMOSTAT_COMMAND_1) {
			uint8_t instance = d[0];
			ThermostatMode opMode = (ThermostatMode)(d[1] & 0x0F);
			FanMode fanMode = (FanMode)((d[1]>>4) & 0x03);
			// uint8_t scheduleEnabled = ((d[1]>>6) & 0x03);
			uint8_t fanSpeed = d[2];
			double heatTemp = convToTempC(d[4]<<8 | d[3]);
			double coolTemp = convToTempC(d[6]<<8 | d[5]);

			const char* name = (dgn == THERMOSTAT_COMMAND_1) ? "THERMOSTAT_COMMAND_1" : "THERMOSTAT_STATUS_1";
			
			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: %s: inst=%d, opMode=%d, fanMode=%d, fanSpeed=%d, heatTemp=%0.1fºC, coolTemp=%0.1fºC\n", (uint32_t)millis(), name, instance, opMode, fanMode, fanSpeed, heatTemp, coolTemp);
			}
		}
		else if (dgn == FURNACE_STATUS || dgn == FURNACE_COMMAND) {
			uint8_t instance = d[0];
			ACMode opMode = (ACMode)(d[1] & 0x03);
			uint8_t fanSpeed = d[2];
			uint8_t heatOutput = d[3];

			const char* name = (dgn == FURNACE_COMMAND) ? "FURNACE_COMMAND" : "FURNACE_STATUS";

			if (printPacket==packetPrintYes || printPacket==packetPrintIfKnown) {
				printf("%u: %s: inst=%d, opMode=%d, fanSpeed=%d, heatOutput=%d\n", (uint32_t)millis(), name, instance, opMode, fanSpeed, heatOutput);
			}
		}
		else if (dgn == DC_LOAD_COMMAND) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t direction = (d[3] >> 4) & 0xF;
			uint8_t command = d[4];

			printf("%u: DC_LOAD_COMMAND: inst=%d, grp=0X%02X, bright=%d, dir=%d, cmd=%d\n", (uint32_t)millis(), instance, group, brightness, direction, command);				
		}
		else if (dgn == AWNING_COMMAND) {
			uint8_t instance = d[0];
			uint8_t direction = d[2];
			uint8_t position = d[3];
			uint8_t retract = d[5];

			printf("%u: AWNING_COMMAND: inst=%d, dir=%d, pos=%d, retract=%d\n", (uint32_t)millis(), instance, direction, position, retract);			
		}
		else if (dgn == AIR_CONDITIONER_STATUS) { }
		else if (dgn == AIR_CONDITIONER_COMMAND) { }
		else if (dgn == AWNING_STATUS) { }
		else if (dgn == AWNING_STATUS_2) { }
		else if (dgn == DATE_TIME_STATUS) { }
		else if (dgn == GENERIC_INDICATOR_COMMAND) { }
		else if (dgn == GENERIC_CONFIGURATION_STATUS) { }
		else if (dgn == DC_SOURCE_STATUS_1) {
			uint8_t instance = d[0];
			uint8_t priority = d[1];
			uint16_t v_int = d[3]<<8 | d[2];
			uint32_t i_int = d[7]<<24 | d[6]<<16 | d[5]<<8 | d[4];
			float voltage = v_int * 0.050;
			float current = -2000000.0 + i_int * 0.001;

			printf("%u: DC_SOURCE_STATUS_1: inst=%d, pri=%d, voltage=%0.1f, current=%0.1f\n", (uint32_t)millis(), instance, priority, voltage, current);		
		}
		else if (dgn == BATTERY_STATUS) { }
		else if (dgn == TANK_STATUS) { }
		else if (dgn == GENERATOR_STATUS_1) { }
		else if (dgn == 0x0FECA) { /* DM_1 - 1FECA - DM_RV */ }
		else if (dgn == 0x0E8FF) { }
		else if (dgn == 0x0EAFF) { }
		else if (dgn == 0x15FCE) { }
		else if (dgn == 0x1BBFD) { }
		else if (dgn == 0x1FACE) { }
		else if (dgn == 0x1FACF) { }
		else if (dgn == 0x1FBDA) { }
		else {
			knownPacket = false;
		}

		if (printPacket==packetPrintYes || (!knownPacket && printPacket==packetPrintIfUnknown) || (knownPacket && printPacket==packetPrintIfKnown)) {
			if (packet->FIR.B.FF == CAN_frame_std) {
				printf("%u: Std - ", (uint32_t)millis());
			}
			else {
				printf("%u: Ext - ", (uint32_t)millis());
			}

			printf("dgn=%05X, src=%02X, pri=%d, Data: ", dgn, sourceAddr, priority);

			for (auto i = 0; i < packet->FIR.B.DLC; i++) {
				printf("%02X ", packet->data.u8[i]);
			}
			printf("\n");
		}
	}
}

void debugPacket(CAN_frame_t *packet) {
	if (packet->FIR.B.RTR != CAN_RTR) {
		uint32_t dgn = getMsgBits(packet->MsgID, 24, 17);
		uint8_t sourceAddr = getMsgBits(packet->MsgID, 7, 8);
		uint8_t priority = getMsgBits(packet->MsgID, 28, 3);

		uint8_t* d = packet->data.u8;

		if (dgn == DC_LOAD_COMMAND) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t direction = (d[3] >> 4) & 0xF;
			uint8_t command = d[4];

			printf("%u: DC_LOAD_COMMAND: inst=%d, grp=0X%02X, bright=%d, dir=%d, cmd=%d\n", (uint32_t)millis(), instance, group, brightness, direction, command);				
		}
		else if (dgn == AWNING_COMMAND) {
			uint8_t instance = d[0];
			uint8_t direction = d[2];
			uint8_t position = d[3];
			uint8_t retract = d[5];

			printf("%u: AWNING_COMMAND: inst=%d, dir=%d, pos=%d, retract=%d\n", (uint32_t)millis(), instance, direction, position, retract);			
		}
		else if (dgn == AWNING_STATUS) {
			printf("%u: AWNING_STATUS\n", (uint32_t)millis());
		}
		else if (dgn == AWNING_STATUS_2) {
			printf("%u: AWNING_STATUS_2\n", (uint32_t)millis());			
		}
		else if (dgn == DC_DIMMER_STATUS_3) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = min(d[2], RVCBrightMax);
			uint8_t enable = (d[3] >> 6) & 3;
			uint8_t delayDuration = d[4];
			DCDimmerCmd lastCmd = (DCDimmerCmd)d[5];
			uint8_t status = (d[6] >> 2) & 3;

			if (instance == 5 || instance == 6) {
				printf("%u: DC_DIMMER_STATUS - AWNING: inst=%d, grp=0X%02X, bright=%d, enable=%d, dur=%d, last cmd=%d, status=0X%02X\n", (uint32_t)millis(), instance, group, brightness, enable, delayDuration, lastCmd, status);				
			}
		}
		else if (dgn == DC_DIMMER_COMMAND_2) {
			uint8_t instance = d[0];
			uint8_t group = d[1];
			uint8_t brightness = d[2];
			DCDimmerCmd cmd = (DCDimmerCmd)d[3];
			uint8_t delayDuration = d[4];

			if (instance == 5 || instance == 6) {
				printf("%u: DC_DIMMER_COMMAND - AWNING: inst=%d, grp=0X%02X, bright=%d, cmd=0X%02X, dur=%d\n", (uint32_t)millis(), instance, group, brightness, cmd, delayDuration);	
			}
		}
	}
}

//////////////////////////////////////////////

void loop() {
	static elapsedMillis heartbeatTime;

	bool sendIndicator = lastPacketSendTime < packetBlinkTime;
	bool recvIndicator = lastPacketRecvTime < packetBlinkTime;
    
	if (sendIndicator || recvIndicator) {
		heartbeatTime = heartbeatBlinkTime;		// adjust next heartbeat time to avoid overlap with send/recv indicators
	}

	bool hearbeatIndicator = (heartbeatTime % heatbeatRate) < heartbeatBlinkTime;

	if (!wifiConnected) {
		hearbeatIndicator = (heartbeatTime % (heatbeatRate/4)) > heartbeatBlinkTime*5;
	}

	digitalWrite(indicatorPinG, !hearbeatIndicator);
	digitalWrite(indicatorPinR, !sendIndicator);
	digitalWrite(indicatorPinB, !recvIndicator);

	homeSpan.poll();

	// HomeSpan does not call the wifi callback after the first connection
	// We do that manually here
	if (hadWifiConnection && !wifiConnected) {
		static uint64_t nextWifiCheck = 0;
		uint64_t time = millis64();

		if (time >= nextWifiCheck) {
			if (WiFi.status()==WL_CONNECTED) {
				wifiReady();
			}

			nextWifiCheck = time + 500;
		}
	}

	CAN_frame_t packet;

	if (uxQueueMessagesWaiting(CAN_cfg.rx_queue) >= receiveQueueSize) {
		rxQueueFullCount++;
	}

	// Drain the queue: one packet per pass falls behind whenever homeSpan.poll() is busy
	for (auto i=0; i<maxPacketsPerLoop && xQueueReceive(CAN_cfg.rx_queue, &packet, 0) == pdTRUE; i++) {
		if (packetPrintMode != packetPrintNo) {
			displayPacket(&packet, packetPrintMode);
		}
		if (awningDebug) {
			debugPacket(&packet);
		}
		censusRecord(&packet);
		processPacket(&packet);
	}

	processPacketQueue();
}
