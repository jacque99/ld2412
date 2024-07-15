# HLK-LD2412 mmWave Radar Presense Detector UART driver

This repository contains an ESP-IDF driver for a LD242 Presense detector connected over UART.

## Using the component

Run the following command in your ESP-IDF project to install this component:
```bash
idf.py add-dependency "jacque99/ld2412"
```

## Example

To run the provided example, create it as follows:
The LD2412 sends target distance periodically, to make it human readable display_time_task() shows one of 6 datas. 

```bash
idf.py create-project-from-example "jacque99/ld2412:ld2412-example"
```

Then build as usual:
```bash
cd ld2412-example
idf.py build
```

And flash it to the board:
```bash
idf.py -p PORT flash monitor
```

The example uses GPIOs 4 and 5 for UART RX and TX, respectively.

## Wiring

ESP32 | LD2412 <br />
3.3V <-> VCC <br />
GND  <-> GND <br />
TX   <-> RX <br />
RX   <-> TX 

## License

This component is provided under Apache 2.0 license, see [LICENSE](LICENSE.md) file for details.

## Contributing

Please check [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.