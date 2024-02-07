# tautuk_esp32s3_core

### Install the ESP-IDF on the VS Code

- Download the VScode and then install it on your PC.

https://code.visualstudio.com/

- Install the ESP-IDF extension on the VSCode. You can refer to the video below. When you install the ESP-IDF, you need to select the v5.1.2 because the current code is using libraries of the esp-idf v5.1.2.

https://www.youtube.com/watch?v=eaQip9ZLUdc

- After installing the ESP-IDF extension and ESP-IDF on the VScode, you can open the resource folder on the vscode.

- You can see the port number, target device, menuconfig, clean, build, flash, and monitor buttons on the status bar.

You can see images on the screenshot folder and refer to the screenshots to build the project.

### Resistor Configuration

- The ESP32S3 is using the resistor-divider to measure a gas sensor on the ADC channel. You can find the #defines to set the resistors(RL, R1, and R2). Here RL is the load resistor of the gas sensor and R1 and R2 are the divider resistors. You need to set the resistor values(Kohm). In the test board, I used 10K ohm resistors.

    #define R1                      10 <br>
    #define R2                      10 <br>
    #define RL                      10 <br>

### Building the project

NOTE: You can refer to the screenshot in the screenshots folder.

After adding the project into the VScode, you can set the COM port and Target Model. In the "Command bar", you can find the buttons for COM port, Target Device, Clean, Build, Flash, and Monitor buttons. After setting the port and target device(ESP32S3), you can build, Flash and Monitor the firmware



