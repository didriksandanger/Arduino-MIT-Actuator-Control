// The follow code allows control of a MIT based actuator through a MCP2515 CAN BUS module.
// For CAN BUS communication it uses this library: https://github.com/Seeed-Studio/Seeed_Arduino_CAN
//
//
// To use the actuator in motor mode/ position control enter these commands in serial one by one
// M1          (Motor enable)
// P<pos>      (desired position, e.g. P1.45)
// M0          (Motor disable, whenever you are done)


#include <SPI.h>

#define CAN_2515
//change these pin numbers if they dont match your configuration
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif


// Define actuator value limits (Values based on a AK80-9 actuator from T-Motor
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// Set actuator values
#define ACTUATOR_ID 0x01
float p_in = 0.0f; // position target
float v_in = 0.0f; //velocity target 
float kp_in = 20.0f; // stiffness target (proportional gain)
float kd_in = 1.0f; // dampening target (derivative gain)
float t_in = 0.0f; // torque target

// Measured values
float p_out = 0.0f; // actual position
float v_out = 0.0f; // actual velocity
float t_out = 0.0f; // actual torque

// Timing variable
unsigned long previousMillis = 0; // used for serial timestamps
unsigned long printInterval = 50; // serial prints every 50 ms

void setup() {
    Serial.begin(115200);
    pinMode(CAN_INT_PIN, INPUT);
    delay(1000);

    while (CAN_OK != CAN.begin(CAN_1000KBPS)) {             // init can bus : baudrate = 1000k
        Serial.println("CAN init fail, retry...");
        delay(100);
    }
    Serial.println("CAN init ok!");
}

void loop() {
  // Serial control of actuator
  if(Serial.available() > 0){
      char cmd;
      float receivedFloat;
      bool receivedBool;
      cmd = Serial.read();
      switch(cmd){
          case 'P':
            //Posistion
            receivedFloat = Serial.parseFloat();
            Serial.print("Position set to: ");
            Serial.println(receivedFloat);
            p_in = constrain(receivedFloat, P_MIN, P_MAX);
            // Send data
            packCmd();
            Serial.flush();
            break;
          case 'M':
            //State
            receivedBool = Serial.parseInt();
            if(receivedBool){
              Serial.println("Enabled");
              EnterMotorMode();
            }
            else{
                Serial.println("Disabled");
                ExitMotorMode();
              }
            Serial.flush();
            break;
          default:
            Serial.println("Unknown command.");
            Serial.flush();
        }
    }

  if(!digitalRead(CAN_INT_PIN)){
      // Receive data
      unpackReply();
  }

  // Print data
  if(millis() - previousMillis <= printInterval){
      Serial.println(millis() - previousMillis);
      previousMillis = millis();
      Serial.print("\t");
      Serial.print(p_in);
      Serial.print("\t");
      Serial.print(p_out);
      Serial.print("\t");
      Serial.print(v_out);
      Serial.print("\t");
      Serial.println(t_out);
    }

}



void EnterMotorMode() {
  // Enter Motor Mode (enable)
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFC;
  if(CAN.sendMsgBuf(ACTUATOR_ID, 0, 8, buf) == CAN_OK){
      Serial.println("Packet sent");
  }
  else{
      Serial.println("Error while sending packet");
  }
}

void ExitMotorMode() {
  // Exit Motor Mode (disable)
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFD;
  if(CAN.sendMsgBuf(ACTUATOR_ID, 0, 8, buf) == CAN_OK){
      Serial.println("Packet sent");
  }
  else{
      Serial.println("Error while sending packet");
  }
}

void Zero() {
  // Zero motor
  byte buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFE;
  if(CAN.sendMsgBuf(ACTUATOR_ID, 0, 8, buf) == CAN_OK){
      Serial.println("Packet sent");
  }
  else{
      Serial.println("Error while sending packet");
  }
}

void packCmd() {
  // CAN Command Packet Structure
  // 16-bit position, between -4*pi, 4*pi
  // 12-bit velocity, between -30 and 30 rad/s
  // 12-bit kp, between 0 and 500 Nm/rad
  // 12-bit kd, between 0 and 100 Nm*s/rad
  // 12-bit feed forward torque, between -18 and 18 Nm
  // CAN Packet is 8 x 8-bit words
  // Formatted as follows. For each quantity, bit 0 is LSB
  //     0: [position[15-8]]
  //     1: [position[7-0]]
  //     2: [velocity[11-4]]
  //     3: [velocity[3-0], kp[11-8]]
  //     4: [kp[7-0]]
  //     5: [kd[11-4]]
  //     6: [kd[3-0], torque[11-8]]
  //     7: [torque[7-0]]

  byte buf[8];

  /// limit data to be within bounds ///
  float p_des = constrain(p_in, P_MIN, P_MAX); // desired position
  float v_des = constrain(v_in, V_MIN, V_MAX); // desired velocity
  float kp = constrain(kp_in, KP_MIN, KP_MAX); // desired stiffness
  float kd = constrain(kd_in, KD_MIN, KD_MAX); // desired dampening
  float t_ff = constrain(t_in, T_MIN, T_MAX); // feed forward torque

  /// convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  /// pack ints into the CAN buffer ///

  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;
  if(CAN.sendMsgBuf(ACTUATOR_ID, 0, 8, buf) == CAN_OK){
      Serial.println("Packet sent");
  }
  else{
      Serial.println("Error while sending packet");
  }
}




void unpackReply(){

    // CAN Reply Packet Structure
    // 16-bit position, between -4*pi, 4*pi
    // 12-bit velocity, between -30 and 30 rad/s
    // 12-bit current. between -40 and 40
    // CAN Packet is 5 x 8-bit words
    // Formatted as follows. For each quantity, bit 0 is LSB
    //     0: [position[15-8]]
    //     1: [position[7-0]]
    //     2: [velocity[11-4]]
    //     3: [velocity[3-0], current[11-8]]
    //     4: [current[7-0]]

    byte len = 0;
    byte buf[8];
    if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data coming
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

        // unpack data from can buffer
        unsigned int id = buf[0];
        unsigned int p_int = (buf[1] << 8) | buf[2]; // position data
        unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4); // velocity data
        unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5]; // torque data

        if(id == ACTUATOR_ID){
          // then convert uint to floats
          p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
          v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
          t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);
        }
    }
  }

  unsigned int float_to_uint(float x, float x_min, float x_max, int bits){
      // Converts a float to an unsigned integer with a given range and length
      float span = x_max - x_min;
      if(x < x_min){
        x = x_min;
      }
      else if(x > x_max){
        x = x_max;
      }
      return (int) ((x- x_min)*((float)((1<<bits)/span)));
    }

  float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
      // Converts an unsigned int to a float with a given range and length
      float span = x_max - x_min;
      float offset = x_min;
      return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }
