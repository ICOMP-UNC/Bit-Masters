# Installation and Setup Guide

## Objective

This guide provides clear and detailed instructions on how to build, flash, and run the firmware on your STM32F103C8 target hardware.

## Prerequisites

Before starting, make sure you have the following tools and dependencies installed:

- **PlatformIO**: The development environment used for building and uploading the firmware. Install the PlatformIO extension for Visual Studio.
- **ST-Link (or similar)**: A tool for flashing the firmware to your STM32F103C8 board.
- **OpenOCD (optional)**: OpenOCD can also be used for flashing, but is not required if using ST-Link.

## Steps for Installation

### 1. Install PlatformIO

- Open Visual Studio and navigate to the Extensions menu.
- Search for **PlatformIO** and install the PlatformIO extension.
- Restart Visual Studio to complete the installation.

### 2. Clone the Repository

Clone the project repository into a directory of your choice.

```bash
git clone https://github.com/ICOMP-UNC/Bit-Masters.git
cd <project_directory>
```

### 3. Install Dependencies

PlatformIO will automatically handle dependencies when building the project. However, you can manually install them by running the following command:

```bash
pio lib install
```

### 4. Build the Firmware

To build the firmware for the STM32F103C8, follow these steps:

- Open the project in Visual Studio.
- Press **Ctrl+Shift+B** to initiate the build process or use the PlatformIO interface.

PlatformIO will automatically compile the code using the configuration in the `platformio.ini` file.

### 5. Flash the Firmware

To upload the firmware to the STM32F103C8 board:

- Connect your STM32F103C8 to your PC via ST-Link or any compatible programmer.
- Click the **Upload** button in PlatformIO or run the following command in the terminal:

```bash
pio run --target upload
```

This will flash the firmware to your board. PlatformIO will automatically detect the connected hardware and upload the compiled firmware.

### 6. Supported Platforms

This project is configured for the following:

- **Microcontroller**: STM32F103C8
- **Operating Systems**: Windows, macOS, Linux
- **Tools**: PlatformIO, ST-Link, OpenOCD (optional)

Make sure your system has the necessary drivers for the ST-Link and the STM32F103C8.
