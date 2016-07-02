# Coffee_machine

## Dependencies
- Makefile chain [Arduino-Makefile](https://github.com/sudar/Arduino-Makefile)
- RF24 and I2Cmaster custom libraries. They are located in *lib* subfloder

## Board selection
See boards available with `make show_boards`
Update BOARD_TAG in Makefile file.

## Use
- Update ARDUINO_SKETCHBOOK in Makefile file with Arduino-Makefile download path.
- Update ARDUINO_LIBS in Makefile file with libraries path
- Build and upload with `make` and `make upload`

## Note
Tested with arduino 1.6.0
