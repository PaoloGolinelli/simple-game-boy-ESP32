# simple-game-boy-ESP32
In this repository you can find all the material required to build a DIY console using the micro controller ESP32

## Content of the repository
- code for ESP32:
    In the beginning of the code you can select 3 preprocessor's variables which are used to differentiate between the boards you're using. 
    - MULTIPLAY: set to true to add the multiplayer games; require Bluethooth (BT) communication. Use this only if you plan on building two consoles to play with friends.
    - BT_CLASSIC: set to true if you want to use the classic BT protocol; set to false to use the BT Low Energy (BLE) protocol. If possible I recommend using the classic protocol, because it is faster and more resilient, but consumes more energy. ESP32 S3 mini has only BLE integrated, while WROOM has both.
    - ESP32_MOD: set to true if you're using WROOM module, false if youre using S3 mini. Used only to define the pins.

- schematic diagram of the circuit on breadboard
    Before building the whole console try it on breadboard to verify your board and components are compatible with the code.
    - S3 mini zero: https://app.cirkitdesigner.com/project/c8f486d2-5efd-4bba-b75a-e2c31b4107fe
    - WROOM D1 mini: https://app.cirkitdesigner.com/project/0857a7c8-4681-452b-a24f-5374fa44a8b1

- schematic diagram of the circuit to weld on matrix perforated board ("PCB")
    The 3D files to print the casing are specific to the S3 mini zero version. Even tho I provide the sketches to implement on perf-board with both micro-controllers, only the first is complete of all components.
    - S3 mini zero: https://app.cirkitdesigner.com/project/2bdec6b9-4f45-4acb-8dcb-3ecd578236df
        For the positioning of the components on the perfboard please refer to the 3D model of the assembly. Mind that some components are welded on the front, while others are on the back.
    - WROOM D1 mini: https://app.cirkitdesigner.com/project/fdfa9b3a-22b1-4d85-bbf5-8aa331098814

- stl file for 3D printing the case and positioning of the components
    - Assembly
    - top cover
    - lower cover
    - joystick cover

## Components required to make your own
- ESP32 S3 MINI Zero (or WROOM D1 mini)
- 2 axis joystick: KY-023
- LCD display: SSD1306 OLED 0.96'' I2C IIC 128x64
- breadboard
- jumper M/M
- jumper M/F
- matrix board 7x3
- slide switch: SS-12D00G 4mm
- 9V battery clip connector
- 9V battery
- step down voltage regulator: DD4012SA 5V

### Instructions 

## Required tools
- filament 3D printer
- welder