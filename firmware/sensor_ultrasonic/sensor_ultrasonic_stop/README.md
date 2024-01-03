# PARTS Common Robot Platform - SBUS and Ultrasonic Sensor Integration

## Introduction
This repository contains the source code for integrating SBUS communication with an ultrasonic sensor on the Portland Area Robotics Society's Common Robot Platform. It's designed to provide an example of how to use SBUS for remote control and an ultrasonic sensor for obstacle detection, specifically tailored for robotics enthusiasts at all skill levels.

### Features
- SBUS communication for remote control input.
- Ultrasonic sensor integration for obstacle detection.
- Motor control logic to halt forward movement when an obstacle is within a specified distance.

## Installation

### Prerequisites
- Arduino IDE
- Hardware: Any compatible Arduino board with SBUS support and an ultrasonic sensor.

### Libraries
- `sbus.h`: For SBUS communication.
- `NewPing.h`: For ultrasonic sensor functionality.

To install these libraries, use the Library Manager in the Arduino IDE and search for "SBUS" and "NewPing", respectively.

### Setup
1. Clone this repository or download the source code.
2. Open the `.ino` file in the Arduino IDE.
3. Connect your Arduino board to your computer.
4. Select your board and port in the Arduino IDE.
5. Upload the sketch to your board.

## Usage
After uploading the code to your Arduino board:
1. Connect the SBUS receiver to the specified Serial port on the Arduino.
2. Connect the ultrasonic sensor to the designated trigger and echo pins.
3. Power on the SBUS transmitter for remote control.
4. The robot will move according to the SBUS inputs and will stop moving forward if an obstacle is detected within 10 cm.

## Contributing
Contributions to this project are welcome. Here are some ways you can contribute:
- Reporting bugs.
- Suggesting enhancements.
- Sending pull requests with improvements to the code or documentation.

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## License
This project is licensed under the [MIT License](LICENSE) - see the LICENSE file for details.

## Acknowledgments
- Portland Area Robotics Society (PARTS) for the inspiration and support.
- Contributors and community members who have provided feedback and suggestions.

---

For more information and additional resources, please visit [Portland Area Robotics Society](https://portlandrobotics.org/).
