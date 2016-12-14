#define SENSOR_DATA_LENGTH 8

char sensor_data[SENSOR_DATA_LENGTH];  //String for the sonic sensor data

void setup() {
  Serial.begin(9600);  //Set baud rate for serial transmission
  
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
