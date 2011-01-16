# OTOH Firmware

OTOH Firmware sketch is derived from the StandardFirmata sketch.
The firmware sketch augments it with an interface to control the **timeline** and the **waveform**.

## The Firmata protocol and StandardFirmata

Firmata is a generic protocol for communicating with microcontrollers from software on a host computer.
It is intended to work with any host computer software package.
Right now there is a matching object in a number of languages.
It is easy to add objects for other software to use this protocol.
Basically, this firmware establishes a protocol for talking to the Arduino from the host software.
The aim is to allow people to completely control the Arduino from software on the host computer.

More informations about the Firmata protocol and the StandardFirmata skatch could be found at [http://firmata.org](http://firmata.org).

## OTOH specific interface

The firmware uses **MIDI SysEx messages** to send and receive custom messages.

These messages are composed by

 * `START_SYSEX` (`0xF0`)
 * one byte representing the specific SysEx command (`0x00`-`0x7F`)
 * between 0 and `MAX_DATA_BYTES` 7-bit bytes of arbitrary data used as arguments for the command
 * `END_SYSEX` (`0xF7`)

## Blink led

Blink led commands are used for debugging purpose.

### Blink led

Makes the 13th led blink.

SysEx command: `0x02`

Arguments:

 * `delay` (2 bytes)<br/>
   delay time in msecs between on and off

### Turn on

Turns the 13th led on.

SysEx command: `0x03`

Arguments: none

### Turn off

Turns the 13th led off.

SysEx command: `0x04`

Arguments: none

## Timeline

Timeline commands rules the timeline behavior.
The timeline is composed by 54 leds of one Maxim matrix distributed between 7 registers and 8 columns.

Leds in the timeline matrix are automatically turned on sequentially with a delay between one and the next set by the `speed` argument.
At any moment there could be a maximum of 2 leds turned on, this means that there are two timelines.
Each timeline follows a direction (clockwise or anti-clockwise) and could start and and at any point of the timeline.

**Note:** timeline speed argument sets an upper limit to loadable samples length.

### Timeline start

Starts a timeline.

SysEx command: `0x05`

Arguments:

 * `id` (1 byte)<br/>
   the timeline id
 * `speed` (2 bytes)<br/>
   timeline speed in msecs
 * `orientation` (1 byte)<br/>
   1 for clockwise, 2 for anti-clockwise orientation
 * `offset` (1 byte)<br/>
   timeline starting led, values between 0 and 55
 * `limit` (1 byte)<br/>
   timeline ending led, values between 1 and 56

### Timeline stop

Stops a timeline.

SysEx command: `0x06`

Arguments:

 * `id` (1 byte)<br/>
   the timeline id

## Waveform

Waveform commands rules the waveform led matrix behavior.
The waveform matrix is composed by 128 leds distributed between 2 Maxim matrixes.

Waveform leds are divided into 32 registers each composed by 4 columns.
By calling the `waveform` command you can change the value of every register.

Columns leds are turned on by a value corresponding to their power of 2, for example:

 * 1 turns first led on
 * 2 turns second led on
 * 3 turns first and second led on
 * etc.

following this principle each register could assume a value between 0 and 15 respectively meaning all register leds off and on.

### Waveform

Changes a waveform register value.

SysEx command: `0x07`

Arguments:

 * `register` (1 byte)<br/>
   select register, a value between 1 and 32
 * `column` (1 byte)<br/>
   register value, a value between 1 and 16 (TODO: fix to match the definition - values between 0 and 15)

### Waveform clear

Turns all the waveform leds off.

SysEx command: `0x08`

Arguments: none