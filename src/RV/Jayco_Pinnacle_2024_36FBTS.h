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
    252, DimmableLamp, "Living Room",
    251, DimmableLamp, "Kitchen Pendant",
    250, DimmableLamp, "Kitchen Accent",
    249, DimmableLamp, "Dinette",
    248, DimmableLamp, "Bedroom",
    247, DimmableLamp, "Bed Reading",
    246, DimmableLamp, "Bed/Bath Accent",
    245, DimmableLamp, "Security",
    244, DimmableLamp, "Awning 1 & Step",
    243, DimmableLamp, "Awning 2",
    242, Lamp, "Rear Accent",

    241, Switch, "Water Pump",
    240, Switch, "Gas Water Heater",
    239, Switch, "Electric Water Heater",
    238, Switch, "Tank Heaters"
};

// index, upIndex[or -1], downIndex[or -1], name
const FanDeviceRec fanList[] = {
    // 237, -1, -1, "Bathroom" // not used with optional third AC
};

// coolingInstance, compressorIndex, fanHIndex, fanLIndex, furnaceInstance[or -1], combustionIndex[or -1], name
const ThermostatDeviceRec thermostatList[] = {
    236, 235, 234, 233, 232, 231, "Main",
    230, 229, 228, 227, -1, -1, "Second AC",
    226, 225, 224, 223, -1, -1, "Third AC"
};

// extendIndex, retractIndex, extendTime, retractTime, name
#define HAVE_AWNINGS
const AwningDeviceRec awningList[] = {
    222, 221, 1000, 1000, "Awning 1",
    220, 219, 1000, 1000, "Awning 2"
};
