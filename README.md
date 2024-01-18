# STM32 Elevators Monitor System

## Introduction
The STM32 Elevators Monitor System is a state-of-the-art project showcasing the powerful capabilities of the STM32 microcontroller for real-time elevator control. Leveraging the robust features of C, this project demonstrates intricate USB UART communication, precise timer management, and advanced data structures for efficient data handling.

## Technical Specifications
- **Microcontroller Clock**: Configured with a pre-scaler to operate at 32 MHz, allowing for 16-bit timers with a count of 35999 to achieve 1 ms time steps, further scaled internally using `TIME_STEP`.
- **Communication**: Implements USB UART communication for reliable data transfer, ensuring seamless interaction with the Qt application.
- **Timers**: Utilizes advanced timer configurations to manage elevator states and transitions with millisecond accuracy.
- **Data Structures**: Employs carefully designed data structures and leverages `__weak` methods for data deserialization to facilitate maintainability and scalability.
- **CRC Validation**: Incorporates CRC-8 checksum for enhanced data integrity, ensuring the reliability of the communication protocol.
- **Hardware Configuration**: Pins are activated according to the STM32 datasheet to support a 48 MHz UART operation. The system is based on CubeIDE version 1.8.0 and is configured for crystal oscillator with a rising edge.

## Expandability
The design is inherently scalable, allowing for the addition of more elevators. It supports a variety of features such as ADCs and DMAs, providing a flexible base for further development.

## Recommendations
For developers looking to extend the capabilities of this project, the STM32 microcontroller's onboard ADCs and DMAs offer a wealth of opportunities. Interested parties can delve into my repositories to find projects utilizing these features, including implementations with FreeRTOS or other real-time operating systems, to draw inspiration and build upon a robust framework.

## Getting Started
Clone this repository and open the project in CubeIDE version 1.8.0 or later. Ensure that the crystal oscillator settings and UART configurations match your hardware setup for optimal performance.

## Contributing
Contributions are welcome, particularly those that enhance the system's real-time capabilities or integrate additional features such as ADCs and DMAs. Please submit pull requests with detailed descriptions of your enhancements.

## License
This project is released under a permissive license, allowing for broad use with an emphasis on shared improvement and innovation. For specific terms, please refer to the LICENSE file.

## Acknowledgements
Special thanks to the STM32 community for their invaluable resources and support that made this project possible.

We invite you to join us in pushing the boundaries of what's possible with microcontrollers and real-time systems.
