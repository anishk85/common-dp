Thor Arm Hardware Wiring Diagram
This guide provides a detailed circuit diagram and explanation for wiring the physical hardware of the Thor robot arm. This setup uses a Raspberry Pi for high-level control (ROS 2) and an Arduino Mega for real-time motor and encoder processing.

IMPORTANT: Always disconnect all power sources (both USB and the main DC power supply) before making or changing any connections.

System Overview Diagram
This diagram shows the high-level connections between all the major components.

Detailed Wiring Instructions
Follow these steps carefully. It is crucial to connect the grounds correctly to avoid damaging your components.

1. Power Distribution
You have two separate power circuits: one for the logic (Pi & Arduino) and one for the high-power motors. They must share a common ground.

Motor Power (5V 10A DC Supply):

Connect the Positive (+) terminal of the 5V 10A power supply to the VM+ (or VCC) pin on all three TB6612FNG motor driver boards.

Connect the Negative (-) terminal (Ground) of the 5V 10A power supply to the GND pin on all three TB6612FNG motor driver boards.

Logic Power (Raspberry Pi):

The Raspberry Pi is powered by its own 5V 3A USB-C adapter.

The Arduino Mega will be powered by the Raspberry Pi via the USB cable.

CRITICAL - Common Ground:

Run a single jumper wire from any GND pin on the Arduino Mega to the Negative (-) terminal bus of your motor power supply (or any GND pin on one of the motor drivers). This is the most important connection. Without it, the control signals from the Arduino will not work correctly.

2. Control Connection (Raspberry Pi to Arduino)
Connect the Raspberry Pi to the Arduino Mega using a standard USB-A to USB-B cable. This single cable provides both power to the Arduino and the serial data connection.

3. Motor and Driver Connections (Arduino to Motors)
You will need three TB6612FNG driver boards. Each board controls two motors (Motor A and Motor B).

For each of the 6 motors:

Connect the two motor leads to the A01/A02 (for Motor A) or B01/B02 (for Motor B) terminals on a driver board.

Connect the PWM pin from the Arduino (e.g., Pin 2 for Joint 1) to the PWMA (or PWMB) pin on the driver board.

Connect two digital pins from the Arduino (e.g., Pins 22 and 23 for Joint 1) to the AIN1/AIN2 (or BIN1/BIN2) pins on the driver board. These control the motor's direction.

Connect the motor driver's STBY (Standby) pin to an Arduino digital pin (e.g., Pin 8). You can connect all three STBY pins together and control them with a single Arduino pin. This pin must be set to HIGH to enable the drivers.

4. Encoder Feedback Connections (Motors to Arduino)
Each goBILDA motor has a 6-pin encoder cable.

For each of the 6 encoders:

+ (VCC): Connect this to the 5V pin on the Arduino.

- (GND): Connect this to any GND pin on the Arduino.

A (Channel A): Connect this to an interrupt-capable digital pin on the Arduino Mega (e.g., Pin 18).

B (Channel B): Connect this to any other digital pin on the Arduino Mega (e.g., Pin 19).

Why Interrupt Pins?
The Arduino Mega has specific pins that can trigger interrupts (Pins 2, 3, 18, 19, 20, 21). Using these for Channel A of each encoder guarantees that the Arduino will never miss a pulse, ensuring your position tracking is perfectly accurate. Refer to an Arduino Mega pinout diagram to identify all interrupt pins.

After completing this wiring, your next step will be to upload the thor_firmware.ino sketch to the Arduino, making sure to update the pin definitions in the code to match the physical connections you just made.