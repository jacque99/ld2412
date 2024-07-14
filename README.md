# HLK-LD2412 24GHz Radar Presense Detector UART driver

This repository contains an ESP-IDF driver for a LD242 Presense detector connected over UART.

## Using the component

Run the following command in your ESP-IDF project to install this component:
```bash
idf.py add-dependency "lhagva/ld2412"
```

## Example

To run the provided example, create it as follows:

```bash
idf.py create-project-from-example "lhagva/ld2412:ld2412-example"
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

The example uses GPIOs 4 and 5 for TX and RX, respectively.

## License

This component is provided under Apache 2.0 license, see [LICENSE](LICENSE.md) file for details.

## Contributing

Please check [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.