# Finding the instance numbers for a new coach

Every light, pump, fan and thermostat on an RV-C network is addressed by an *instance* number.
RV-Bridge needs those numbers in a definition file (`src/RV/*.h`). On Firefly coaches they are
printed on the panel and shown on a diagnostics screen. On other systems nothing shows them, and
they have to be read off the bus. This page describes how.

The golden rule: **listen first, transmit last, and never guess.** The same controller that
switches the lights also drives slides, jacks, awnings and the generator. Do not sweep through
instance numbers to "see what turns on".

## The tools

Open the serial monitor (115200 baud) and type these HomeSpan commands:

| Command | What it does |
| --- | --- |
| `@c` | Print the census: every message type (DGN), instance and source address seen so far, with how often it was seen, how often it changed, and its last data |
| `@x` | Clear the census |
| `@m 1` | Monitor: print a line whenever a switch-like status changes, and for every command another device sends |
| `@m 2` | As above, plus changes in DGNs RV-Bridge has no name for |
| `@m 3` | Every change of every message (noisy) |
| `@m 0` | Monitor off |
| `@i` | Ask every device heard on the bus for its Product ID (transmits, so needs writes enabled) |
| `@w 0` / `@w 1` | Disable / enable transmitting |
| `@o 12=1` | Send ON (`=0` for OFF) to instance 12 |
| `@p 1` | Raw packet log |

Monitor lines look like this:

```
35845: [CMD] 1FEDB DC_DIMMER_COMMAND_2 src=156 inst=12: 0C FF C8 05 FF 00 FF FF
35845: [CHG] 1FEDA DC_DIMMER_STATUS_3 src=142 inst=12: 0C FF C8 FC FF 05 04 FF  was: 0C FF 00 FC FF 03 00 FF  level=100%
```

The first line is the wall panel (source address 156) telling instance 12 to toggle. The second is
the controller (source 142) reporting that instance 12 went from 0 to 100%. `inst=12` is the
number that goes in the definition file.

## Procedure

### 1. Connect without transmitting

Build and flash the **Debug** environment. It is compiled with `DISABLE_CAN_BUS_WRITES` and cannot
send a message no matter what is typed. (It still acknowledges frames, as every CAN node does.)

RV-C runs at 250 kbit/s on a twisted pair with a 120 ohm terminator at each end. With the coach's
12V off, a meter across CAN-H and CAN-L should read about 60 ohms. Connect RV-Bridge as a short
drop off the trunk. If your CAN transceiver board has its own 120 ohm terminator, disable it:
a third terminator can upset the network.

### 2. Take a baseline

Leave everything alone for a minute, then `@c`. This shows which message types the coach uses at
all. Look for:

* `DC_DIMMER_STATUS_3` - this is what RV-Bridge uses for lights and switched loads. One line per
  instance is the best case.
* `DC_LOAD_STATUS`, `GENERIC_INDICATOR_STATUS`, `WATER_PUMP_STATUS`, `WATERHEATER_STATUS` - the coach
  reports loads a different way. RV-Bridge does not decode these yet.
* `THERMOSTAT_STATUS_1` and `THERMOSTAT_AMBIENT_STATUS` - one instance per climate zone.
* `(unnamed)` rows - proprietary or not-yet-named messages.
* A warning that another device uses RV-Bridge's source address. If so, change `sourceAddress` in
  `config.h` to one that is not in the "Source addresses seen" list.

Save this output. It answers the question "can RV-Bridge talk to this coach as it stands?"

### 3. Map each control from the wall panel

`@m 1`, then operate **one thing at a time** from the coach's own touch screen or switches and
write down the instance that reports a change:

1. Each light, on then off. For a dimmable light also set it to about half: the `level=` value
   should follow, which confirms it is a dimmer (`DimmableLamp`) and not on/off (`Lamp`).
2. Water pump, each water heater, tank heaters and any other switched load (`Switch`).
3. Each climate zone: change the set temperature by one degree. The `THERMOSTAT_STATUS_1` instance
   that changes is that zone's `coolingInstance`. Then change the mode (cool, heat, fan).
   If heating shows up on a *different* instance, that is the `furnaceInstance`; if it shows on the
   same one, use the same number for both.
4. While an air conditioner runs, watch for load instances that switch on a little after the mode
   change: those are the compressor and fan (`compressorIndex`, `fanHIndex`, `fanLIndex`). If the
   coach does not report them, use `-1` (they only drive the "currently heating/cooling"
   indicator).

If a control produces no line at `@m 1`, repeat it at `@m 2`. A change in an `(unnamed)` DGN means
the coach uses a message RV-Bridge does not understand yet. Keep that line: with the DGN and a
before/after pair, support can be added.

Do **not** operate slides, jacks, awnings or the generator for this. If you want to know which
numbers they use so that you can avoid them, note the `[CMD]` lines only.

### 4. Note how the panel commands things

The `[CMD]` lines show exactly what the coach's own panel sends: which DGN, and for
`DC_DIMMER_COMMAND_2` the command code in the 4th data byte (00 set level, 01 on, 03 off,
05 toggle). RV-Bridge sends `01`/`03` for on/off and `00` for a brightness. If the panel uses a
different DGN to command a load, RV-Bridge's commands may be ignored for that load.

### 5. Write the definition file and confirm

Put the numbers into the definition file. Then, with a build that can transmit:

1. `@w 1`
2. For each **light** you mapped: `@o <instance>=1`, check that the right light comes on, `@o <instance>=0`.
3. Only then try the pump and other loads, and finally the thermostats from the Home app.

When everything checks out, delete `#define RV_DEFINITION_UNVERIFIED` from the definition file so the
bridge starts with writes enabled.

## Notes for the 2024 Jayco Pinnacle (BMPRO JAYCOMMAND)

* The Pinnacle's JAYCOMMAND system is made by BMPRO, not Firefly. BMPRO's manuals state that the
  Node "communicates via RV-C" with the wall display, and owners have connected RV-C inverters and
  solar controllers to the same bus, so there is a real RV-C network to listen to.
* What is **not** published is which RV-C messages BMPRO uses for lights and climate, or any
  instance numbers. The numbers in `Jayco_Pinnacle_2024_36FBTS.h` are placeholders. Step 2 above is
  the first real test of whether this coach can be driven by RV-Bridge without new decoding code.
* The BMPRO Node also runs the slides (S1-S6), awnings (A1-A2), jacks and generator start.
  That is the reason for the "never guess" rule, and why the definition file keeps writes off
  until it has been verified.
* The wall switches talk to the Node by Bluetooth, not CAN, so pressing them will show the Node's
  status change but no `[CMD]` line. Use the touch screen to see commands.
* A third air conditioner replaces the bathroom fan on some units; map whichever your coach has.
