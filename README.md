# Random Position Machine

## Circuit diagram

The circuit diagram is stored privately in OneDrive. Access must be granted directly by the owner when needed.

## Teensy 4.1 pin connections

The dual-motor firmware is in [code/teensy.c](code/teensy.c).

| Signal | Motor 1 | Motor 2 |
| --- | ---: | ---: |
| STEP | 2 | 5 |
| DIR | 3 | 6 |
| ENA | 4 | 7 |
| Encoder A | 14 | 17 |
| Encoder B | 15 | 18 |

The firmware is configured for TB6600 drivers and 1/32 microstepping. Encoder inputs use the Teensy's internal pull-up resistors.