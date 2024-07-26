# HLK-LD2412 mmWave Radar Presense Detector UART driver

This repository contains an ESP-IDF driver for a LD242 Presense detector connected over UART.
The LD2412 radar sends target measurement result data peridocally to the UART interface.
It also receives commands from controller and responds by ACK frame. 

uart_config.c file
The function uart_config() initializes UART port of the ESP32.
uart_event_task. Here UART RX callback takes place. 
uart_reception_task handles received frame data, once the related queue is filled. 
uart_transmission_task sends data to the UART buffer, once the related queue is filled.

ld2412.c file
ld2412_parse_target_data_frame and ld2412_parse_command_ack_frame functions process target frame data and ack frame data accordingly, depending on the frame header/end.
send_command() function. This function constructs data frame including the header and end, and sends to the TX queue.
A few examples commands are implemented here in a separate functions, such as control_config_mode(), control_engineering_mode(), read_firmware_version() and set_baud_rate().

## Using the component

Run the following command in your ESP-IDF project to install this component:
```bash
idf.py add-dependency "jacque99/ld2412"
```

## Example

To run the provided example, create it as follows:
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
The example uses BOOT button, button click call backs are used to send commands to the LD2412 radar. 
The radar responds with ACK to those commands. 
display_time_task is created for demonstration purposes, to show measured results on the display when necessary. 

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