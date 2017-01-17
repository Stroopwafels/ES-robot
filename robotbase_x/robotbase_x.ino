#define SENSOR_DATA_LENGTH 8

// Define robot pins
  // H-bridge: LCHB-100
  #define HB_1REV         7
  #define HB_1EN          24
  #define HB_1FWD         6
  #define HB_2REV         3
  #define HB_2EN          25
  #define HB_2FWD         2
  // Bluetooth dongle: HC-05
  #define BT_TX           18
  #define BT_RX           19
  // Sonic sensor: HC-SR04
  #define SENSOR_TRIGGER  23
  #define SENSOR_ECHO     22
  // Yellow LED
  #define YLED            13
  //NOTE: For Aduino MEGA documentation, see: https://www.arduino.cc/en/Main/ArduinoBoardMegaADK

// Define robot geometry
#define ROBOT_WIDTH 0.2 //[m] Distance between tracks //TODO: measure Distance

// Typedefs
typedef struct Twist {
  // Linear speeds
  float lx
  float ly
  float lz
  // Angular speeds
  float ax
  float ay
  float az
} Twist;

// Variables
char sensor_data[SENSOR_DATA_LENGTH];  //String for the sonic sensor data

// Time variables
unsigned long time = millis();

void setup() {
  // Set baud rate for serial transmission
  Serial.begin(9600);  
  
  // Initialise pins


  // Empty data strings
  for(int i=0; i < SENSOR_DATA_LENGTH; i++)
    sensor_data[i] = 0;

}

void loop() {
  // If there's a command, do stuff
  if(Serial.available() > 0) {
    // Get twistdata
  }
  
  // Print the sonic sensor data
  if(sensor_data[0]) {
    for(int i=0; i < SENSOR_DATA_LENGTH; i++) {
      Serial.print(sensor_data[i]);
      sensor_data[i] = 0;
    }
    
    Serial.print("\n"); //TODO Replace with EOT?
  }
}
