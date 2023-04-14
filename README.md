# Edge Impulse Example: standalone inferencing (Infineon/Cypress PSoC62)

This code example runs an exported impulse on the Infineon/Cypress PSoC62 and demonstrates:
* Running the classifier from a trained model in the EdgeImpulse Online Studio
* Using a features data set from the EdgeImpulse Studio

See the documentation at [Running your impulse locally](https://docs.edgeimpulse.com/docs/running-your-impulse-locally-1).

## Requirements

### Hardware

* Development board: [PSoC 62S2 Wi-Fi Bluetooth pioneer kit](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-062s2-43012/) (`CY8CKIT-062S2-43012`)

While the project is mainly tested with the CY8CKIT-062S2-43012 pioneer kit, other PSoC6-based development boards will work too.

### Software

* Install ModusToolbox SDK and IDE

* Toolchain in SDK is GNU Arm® embedded compiler v9.3.1


## Building the application

### Get the Edge Impulse SDK

Unzip the deployed `C/C++ library` from your Edge Impulse project and copy only the folders to the root directory of this repository:

   ```
   example-standalone-inferencing-psoc62/PSoC62_App
   ├─ LICENSE
   ├─ Makefile
   ├─ deps
   ├─ edge-impulse-sdk
   ├─ libs
   ├─ main.c
   ├─ makefile.init
   ├─ model-parameters
   └─ tflite-model  
   ```

### Compile

Initialize PSoC62 SDK. This step is optional, because all required libraries are included in this repo.

   ```bash
   make getlibs
   ```

Compile in terminal as shown below (or use ModusToolbox IDE).

   ```bash
   make build
   ```

### Flash

Connect the PSoC62 pioneer kit to your computer and execute the command below.

   ```bash
   make program
   ```

### Run

Use screen, minicom or terminal in ModusToolbox IDE to set up a serial connection over USB. The following UART settings are used: 115200 baud, 8N1.

   ```bash
   minicom -D /dev/cu.usbmodem11203
   ```

Where `/dev/cu.usbmodem11203` needs to be changed to the actual port where the PSoC62 pioneer kit is connected on your system.
