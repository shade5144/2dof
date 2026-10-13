## Description
Simple ESP32 code for a 2DOF arm made using ice-cream sticks and 2 servo motors

## Motivation
This is essentially my first sojourn into the world of robotics. I was waiting for a 5DOF 3D-printed arm that I ordered online, but figured that there might be a lot to learn even from
a simple 2 DOF arm construction. 

Initially this started as an attempt to make a robot with a pen on its end effector to draw things, but the mechanics for the "holding the pen and applying appropriate pressure" part
turned out to be a bit of a hassle, so I decided against it.

## Scope
The only things I wanted to get done here were:

    1. **Bluetooth** connectivity between my Laptop and the ESP32.
    2. Real time **tracking of the arm's position** using a wire soldered to the internal potentiometers of the servos.
    3. Using the data from point 2, create a basic **Digital Twin** of the arm.
    3. **Moving the servos** using remote commands via BT
    4. **Inverse Kinematics** for the arm. Not done in this release, but will be included in the next one.
    5. (**Maybe**) Getting the arm to follow a certain **trajectory**. This I still need to look into.

## Software Requirement
- This has been tested only on Linux, so not sure about how it should be built and flashed in Windows
- ESP-IDF development environment

## Build instructions
For the client:
```sh
cd bt_client
make
```

For the server(after making sure that the ESP32 is connected):
```sh
cd bt_server
idf.py -p [YOUR_ESP_PORT] flash monitor
```

## Progress
- You can connect to the ESP32 via a Linux system and send commands to set servo angles

- Have to display the arm's position on the client
- Have to implement inverse kinematics

## Setup
The electronics shall be explained later, in more detail. This is what the setup LOOKS like though
![The Arm](./assets/arm1.jpg)

![ESP32 and electronics](./assets/arm2.jpg)

![PSU and Buck Convertor](./assets/arm3.jpg)

![Overall Setup](./assets/arm4.jpg)

## Working
1. Setting Servo Angles:

https://github.com/user-attachments/assets/f5839d24-be0e-44d2-81b0-e9b985a42f51

## Circuit Schematic
TODO

## Development Notes
I ran into a couple of issues in the first release, which I have touched upon here.
- Measuring the servo's position via a wire to the internal potentiometer appeared to be fine in isolation, but when I had to apply voltage to also MOVE the servos while it was being
  measured, this caused the servos to start malfunctioning. Likely this is because the ESP32's ADC does not tolerate the impedance of the internal pot, leading to current draw. My 
  solution to this was to add a simple op-amp buffer, which fixed it. Ideally though, I am aware, I ought to use a specialized ADC.
- The wathcdog timer for CPU0, which runs the bluetooth stack, kept getting tripped. I initially thought it was because of some other task than BT running in CPU0, but pinning my 2 
  tasks to CPU1 didn't seem to work. Then, I tried lowering the priorities of my tasks, `BtAdcSendTask` and `ServoMoveTask`, which seemed to fix the issue.
- The `adc_oneshot_read` also started tripping the watchdog timer(and to some extent this issue is still there) so I just added a delay of 10 ms that runs for every 16 reads
  (see `transmit_task` in `main.c`)
- I initially had a more complicated design for the arm with trusses and stuff, but I discarded this design as my scope changed(as mentioned in the `Motivation` section).
- I looked at a few algorithms for filtering the noisy ADC(exponential average, a median filter) for the servo positions, as it was really quite variable but settled on a simple mean
  due to the complexity of the other approaches.

## References
- The esp-idf documentation and the various examples included in the development framework were of immense help.
- The tutorials on the bluez project by MIT
