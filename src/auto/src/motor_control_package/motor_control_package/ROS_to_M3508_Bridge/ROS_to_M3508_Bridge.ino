  #include <SPI.h>
#include <mcp2515.h>

// Define CAN Bus Related Stuff
struct can_frame canMsgIn;   // CAN frame for incoming messages
struct can_frame canMsgOut;  // CAN frame for outgoing messages
MCP2515 mcp2515(53);         // MCP2515 CAN bus controller instance

// Arrays to store motor data
int motors_speed[] = {0, 0, 0, 0, 0, 0, 0, 0};   // Array to store motor speeds (range: -9158 to 9158 RPM)
int motors_angle[] = {0, 0, 0, 0, 0, 0, 0, 0};   // Array to store motor angles (range: 0 to 8192)
int motors_torque[] = {0, 0, 0, 0, 0, 0, 0, 0};  // Array to store motor torques (range: unknown now, find it out if you are free)

// Global variable to store the output message
String output_message;

// MCP2515 Setup Function
void can_mcp2515_setup() {
  mcp2515.reset();                             // Reset MCP2515 controller
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);  // Set CAN bus bitrate to 1 Mbps and clock frequency to 8 MHz
  mcp2515.setNormalMode();                     // Set MCP2515 to normal mode
}

/*
  Connection Diagram:

  MCP2515 Pin  |  Arduino Mega 2560 Pin
  -------------|-----------------------
  VCC          |  5V
  GND          |  GND
  CS           |  Pin 53 (SS)
  SO           |  Pin 50 (MISO)
  SI           |  Pin 51 (MOSI)
  SCK          |  Pin 52 (SCK)
  INT          |  Pin 2
*/

// CAN Write Function
void can_write(int motor_ID, int power) {
  int motor_write_address = 0;

  // Determine motor write address based on the motor ID
  if (motor_ID >= 1 && motor_ID <= 4) {
    

    motor_write_address = (motor_ID - 1) * 2;
    canMsgOut.can_id = 0x200;  // Set CAN message ID for first group of motors
  } else if (motor_ID >= 5 && motor_ID <= 8) {
    motor_write_address = (motor_ID - 5) * 2;
    canMsgOut.can_id = 0x1FF;  // Set CAN message ID for second group of motors
  }

  canMsgOut.can_dlc = 8;  // Set CAN message length to 8 bytes

  // Set motor power values in the CAN message data
  canMsgOut.data[motor_write_address] = (power >> 8) & 0xFF;
  canMsgOut.data[motor_write_address + 1] = power & 0xFF;

  mcp2515.sendMessage(&canMsgOut);  // Send CAN message
}

// CAN Read Data Function
void can_read_data() {
  int single_motor_speed = 0;
  int single_motor_angle = 0;
  int single_motor_torque = 0;

  while (mcp2515.readMessage(&canMsgIn) == MCP2515::ERROR_OK) {
    // Read speed, angle, and torque from CAN message and map it to the motor data
    if (canMsgIn.can_id >= 0x201 && canMsgIn.can_id <= 0x208) {
      int motor_index = canMsgIn.can_id - 0x201;

      // Read speed (signed 16-bit)
      single_motor_speed = (canMsgIn.data[2] << 8) | canMsgIn.data[3];
      if (single_motor_speed > 32767) single_motor_speed -= 65536;  // Convert to signed 16-bit
      motors_speed[motor_index] = single_motor_speed;

      // Read angle (unsigned 16-bit)
      single_motor_angle = (canMsgIn.data[0] << 8) | canMsgIn.data[1];
      motors_angle[motor_index] = single_motor_angle;

      // Read torque (signed 16-bit)
      single_motor_torque = (canMsgIn.data[4] << 8) | canMsgIn.data[5];
      if (single_motor_torque > 32767) single_motor_torque -= 65536;  // Convert to signed 16-bit
      motors_torque[motor_index] = single_motor_torque;
    }
  }

  // Update the global output message with the latest motor data
  update_output_message();
}

// Function to update the global output message with data for all 8 motors
void update_output_message() {
  output_message = "";
  for (int i = 0; i < 4; i++) {  // Only update for motor IDs 1-4
    output_message += String(i + 1) + ",";
    output_message += String(motors_speed[i]);
    if (i < 3) {
      output_message += ";";
    }
  }
}

// Function to process and send each motor command
void process_commands(const String &commands) {
  int last_index = 0;
  int next_index = 0;

  while (next_index != -1) {
    next_index = commands.indexOf(';', last_index);
    String command = commands.substring(last_index, next_index);
    command.trim();

    // Ensure the command has a valid format (motor_ID,power)
    int separatorIndex = command.indexOf(',');
    if (separatorIndex > 0) {
      int motor_ID = command.substring(0, separatorIndex).toInt();
      int power = command.substring(separatorIndex + 1).toInt();

      // Validate motor_ID and power range
      if (motor_ID >= 1 && motor_ID <= 8 && power >= -9158 && power <= 9158) {
        can_write(motor_ID, power);
      } else {
        Serial.println("Invalid motor ID or power range");
      }
    } else {
      Serial.println("Invalid command format");
    }

    last_index = next_index + 1;
  }
}

// Setup Function
void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud rate
  while (!Serial) {
    ;  // Wait for serial connection to be established
  }
  can_mcp2515_setup();  // Setup MCP2515 CAN bus controller
}

// Main Loop Function
void loop() {
  can_read_data();  // Read data from all motors

  // Output the message containing data for motors 1-4
  Serial.println(output_message);

  // Check for incoming serial commands to control motor power
  if (Serial.available()) {
    String commands = Serial.readStringUntil('\n');
    commands.trim();  // Trim any leading/trailing whitespace
    process_commands(commands);  // Process and send each motor command
  }

  delay(20);  // Adjust delay as necessary
}
