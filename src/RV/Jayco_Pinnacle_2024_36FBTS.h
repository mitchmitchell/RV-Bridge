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
 
// Device definitions for a 2024 Jayco Pinnacle 36FBTS

#include "devices.h"

// index, type, name
const SwitchDeviceRec switchList[] = {
    9401, DimmableLamp, "Living Room",
    9402, DimmableLamp, "Kitchen Pendant",
    9404, DimmableLamp, "Kitchen Accent",
    9403, DimmableLamp, "Dinette",
    9404, DimmableLamp, "Bedroom",
    9104, DimmableLamp, "Bed Reading",
    9105, DimmableLamp, "Bed/Bath Accent",
    9106, DimmableLamp, "Security",
    9107, DimmableLamp, "Awning 1 & Step",
    9108, DimmableLamp, "Awning 2",
    9109, Lamp, "Rear Accent",

    9023, Switch, "Water Pump",
    9000, Switch, "Gas Water Heater",
    9001, Switch, "Electric Water Heater",
    9002, Switch, "Tank Heaters"
};

// index, upIndex[or -1], downIndex[or -1], name
const FanDeviceRec fanList[] = {
    // 9421, -1, -1, "Bathroom" // not used with optional third AC
};

// coolingInstance, compressorIndex, fanHIndex, fanLIndex, furnaceInstance[or -1], combustionIndex[or -1], name
const ThermostatDeviceRec thermostatList[] = {
    9000, 9025, 9026, 9027, 9552, 9533, "Main",
    9101, 9229, 9230, 9231, -1, -1, "Second AC",
    9103, 9029, 9030, 9031, -1, -1, "Third AC"
};

// extendIndex, retractIndex, extendTime, retractTime, name
#define HAVE_AWNINGS
const AwningDeviceRec awningList[] = {
    9005, 9006, 1000, 1000, "Awning 1",
    9003, 9004, 1000, 1000, "Awning 2"
};
