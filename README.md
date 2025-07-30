# dynamixel_motor




## scan dynamixel
/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 

#define MAX_BAUD  5
const int32_t buad[MAX_BAUD] = {57600, 115200, 1000000, 2000000, 3000000};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  int8_t index = 0;
  int8_t found_dynamixel = 0;

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);   //set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         //Wait until the serial port is opened
    
  for(int8_t protocol = 1; protocol < 3; protocol++) {
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion((float)protocol);
    DEBUG_SERIAL.print("SCAN PROTOCOL ");
    DEBUG_SERIAL.println(protocol);
    
    for(index = 0; index < MAX_BAUD; index++) {
      // Set Port baudrate.
      DEBUG_SERIAL.print("SCAN BAUDRATE ");
      DEBUG_SERIAL.println(buad[index]);
      dxl.begin(buad[index]);
      for(int id = 0; id < DXL_BROADCAST_ID; id++) {
        //iterate until all ID in each buadrate is scanned.
        if(dxl.ping(id)) {
          DEBUG_SERIAL.print("ID : ");
          DEBUG_SERIAL.print(id);
          DEBUG_SERIAL.print(", Model Number: ");
          DEBUG_SERIAL.println(dxl.getModelNumber(id));
          found_dynamixel++;
        }
      }
    }
  }
  
  DEBUG_SERIAL.print("Total ");
  DEBUG_SERIAL.print(found_dynamixel);
  DEBUG_SERIAL.println(" DYNAMIXEL(s) found!");
}

void loop() {
  // put your main code here, to run repeatedly:
}


## position control 

/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
* Licensed under the Apache License, Version 2.0
*******************************************************************************/

#include <Dynamixel2Arduino.h>

// Hardware configuration
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2;
#elif defined(ARDUINO_SAM_DUE)
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2;
#elif defined(ARDUINO_SAM_ZERO)
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2;
#elif defined(ARDUINO_OpenCM904)
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22;
#elif defined(ARDUINO_OpenCR)
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84;
#elif defined(ARDUINO_OpenRB)
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2;
#endif

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint32_t DXL_BAUDRATE = 1000000;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

void setupMotor(uint8_t id) {
  dxl.ping(id);
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, 30);
}

void moveMotorRaw(uint8_t id, int goal_position) {
  dxl.setGoalPosition(id, goal_position);
  int present_position = 0;
  while (abs(goal_position - present_position) > 10) {
    present_position = dxl.getPresentPosition(id);
    DEBUG_SERIAL.print("Motor ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" Position (raw): ");
    DEBUG_SERIAL.println(present_position);
  }
}

void moveMotorDegree(uint8_t id, float goal_position_deg) {
  dxl.setGoalPosition(id, goal_position_deg, UNIT_DEGREE);
  float present_position_deg = 0.0;
  while (abs(goal_position_deg - present_position_deg) > 2.0) {
    present_position_deg = dxl.getPresentPosition(id, UNIT_DEGREE);
    DEBUG_SERIAL.print("Motor ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" Position (deg): ");
    DEBUG_SERIAL.println(present_position_deg);
  }
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  setupMotor(DXL_ID_1);
  setupMotor(DXL_ID_2);
}

void loop() {
  // Move both motors to 1000 raw position
  moveMotorRaw(DXL_ID_1, 1000);
  moveMotorRaw(DXL_ID_2, 1000);
  delay(1000);

  // Move both motors to 5.7 degrees
  moveMotorDegree(DXL_ID_1, 5.7);
  moveMotorDegree(DXL_ID_2, 5.7);
  delay(1000);
}


## ✅ Step-by-step Python code for Raspberry Pi:

#!/usr/bin/env python3
import os
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table address for X-Series
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_ID = 1
BAUDRATE = 57600
DEVICENAME = '/dev/ttyACM0'  # OpenCR usually mounts here (check with `ls /dev/tty*`)

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0
DXL_MIN_POSITION_VALUE = 100    # Define your own limits
DXL_MAX_POSITION_VALUE = 1000
DXL_MOVING_STATUS_THRESHOLD = 20  # Threshold for position error

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def open_port():
    if portHandler.openPort():
        print("Port opened successfully")
    else:
        print("Failed to open the port")
        quit()

    if portHandler.setBaudRate(BAUDRATE):
        print(f"Baudrate set to {BAUDRATE}")
    else:
        print("Failed to set baudrate")
        quit()

def enable_torque():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Torque Enable Communication Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Torque Enable Error: {packetHandler.getRxPacketError(dxl_error)}")
    else:
        print("Torque enabled")

def disable_torque():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Torque Disable Communication Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Torque Disable Error: {packetHandler.getRxPacketError(dxl_error)}")
    else:
        print("Torque disabled")

def move_motor(position):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, position)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Move Motor Communication Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Move Motor Error: {packetHandler.getRxPacketError(dxl_error)}")
    else:
        print(f"Motor moved to position {position}")

def read_position():
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Read Position Communication Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Read Position Error: {packetHandler.getRxPacketError(dxl_error)}")
    else:
        print(f"Current Position: {dxl_present_position}")
    return dxl_present_position

# Main control loop
if __name__ == "__main__":
    open_port()
    enable_torque()

    try:
        for i in range(3):
            print(f"\nMoving to max position {DXL_MAX_POSITION_VALUE}")
            move_motor(DXL_MAX_POSITION_VALUE)
            time.sleep(2)

            print("Reading position...")
            read_position()
            time.sleep(1)

            print(f"\nMoving to min position {DXL_MIN_POSITION_VALUE}")
            move_motor(DXL_MIN_POSITION_VALUE)
            time.sleep(2)

            print("Reading position...")
            read_position()
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        disable_torque()
        portHandler.closePort()
        print("Port closed, program terminated")

✅ How to Run:

    Install the SDK:

pip install dynamixel-sdk

Save the script, e.g., dynamixel_control.py, then run:

    python3 dynamixel_control.py

✅ Check Device Name:

On the Raspberry Pi, verify the OpenCR connection:

ls /dev/ttyACM*

Typically, OpenCR shows as /dev/ttyACM0. If not, adjust DEVICENAME accordingly.
✅ Optional: Use dynamixel_workbench on OpenCR

If you flashed OpenCR with Dynamixel Workbench firmware, this Python code works directly. Otherwise, ensure firmware supports Protocol 2.0 communication via USB.
