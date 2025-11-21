// Pin Definitions
const int hallSensorPin0 = A0; // Analog pin connected to A1320 sensor
const int hallSensorPin1 = A1;
const int hallSensorPin2 = A2;
const int hallSensorPin3 = A3;
const int hallSensorPin4 = A4;

const int ledPin = 13;  // LED pin for visual feedback (optional)
const int MODE_DET_PIN = 2;
const int MODE_PROB_PIN = 3;
const int MODE_TEST_PIN = 4;
const int MODE_ADMIN_PIN = 5;


// Constants
const float voltageRef = 5.0;     // Reference voltage of Arduino (5V)
const int adcResolution = 1023;  // ADC resolution for Arduino (10-bit)
const float magnet_present_threshold_low = 0.5; // experimentally determined voltage threshold 
                                            // when north pole is infront of sensor
const float magnet_present_threshold_high = 1.75; // experimentally determined voltage threshold
                                                  // when south pole is in front of sensor

const int MODE_UNKNOWN = -1;
const int MODE_ADMIN = 0;
const int MODE_PROB = 1;
const int MODE_DET = 2;
const int MODE_TEST = 3;
const int MODE_EXP = 4;

const int EMPTY_BLOCK = 0;
const int BLOCK_1 = 1;
const int BLOCK_2 = 2;
const int BLOCK_3 = 3;
const int BLOCK_4 = 4;
const int BLOCK_5 = 5;
const int BLOCK_6 = 6;

const float EMPTY_MIN = 0.0f;
const float EMPTY_MAX = 0.0f;
const float BLOCK1_MIN = 100.0f;
const float BLOCK1_MAX = 1000000.0f;
const float BLOCK2_MIN = 2.0f;
const float BLOCK2_MAX = 2.0f;
const float BLOCK3_MIN = 3.0f;
const float BLOCK3_MAX = 3.0f;
const float BLOCK4_MIN = 4.0f;
const float BLOCK4_MAX = 4.0f;
const float BLOCK5_MIN = 5.0f;
const float BLOCK5_MAX = 5.0f;
const float BLOCK6_MIN = 6.0f;
const float BLOCK6_MAX = 6.0f;

int previous_block = EMPTY_BLOCK;
int current_block = EMPTY_BLOCK;
int placement_count = 0;
const int MAX_COUNT_PROB = 11;

int rand1, rand2, rand3;

int previous_mode = MODE_UNKNOWN;

void setup() {
  Serial.println("Setup");
  reset();
  pinMode(hallSensorPin0, INPUT); // Set Hall sensor pin as input
  pinMode(hallSensorPin1, INPUT);
  pinMode(hallSensorPin2, INPUT);
  pinMode(hallSensorPin3, INPUT);
  pinMode(MODE_PROB_PIN, INPUT_PULLUP);
  pinMode(MODE_DET_PIN, INPUT_PULLUP);
  pinMode(MODE_TEST_PIN, INPUT_PULLUP);
  pinMode(MODE_ADMIN_PIN, INPUT_PULLUP);
  
  pinMode(ledPin, OUTPUT);       // Set LED pin as output (optional)
  digitalWrite(ledPin, LOW);     // Turn off LED initially
  Serial.begin(9600);            // Initialize serial communication
}


void loop(){
  Serial.println("I'm about to call get current mode");
  int current_mode = get_current_mode();

  if(current_mode != previous_mode){
    Serial.println("Mode Changed");
    reset();
  }
  if(current_mode == MODE_ADMIN){
      run_admin_mode();
  }
  else if(current_mode == MODE_PROB){
    run_prob_mode();
  }
  else if(current_mode == MODE_DET){
    run_det_mode();
  }
  else if(current_mode == MODE_TEST){
    run_test_mode();
  }
  else if(current_mode == MODE_EXP){
    run_exp_mode();
  }
  else{
    Serial.print("knob read error");
  }
  previous_mode = current_mode;
}

void run_prob_mode(){
  // For probabilistic condition:
// - 1st block makes machine light up 3/3 times
// - 2nd block makes machine light up 0/3 times
// - 3rd block makes machine light up 2⁄3 times (on 2nd and 3rd tries)
// - 4th block makes machine light up 1⁄3 times (on 3rd try)
  current_block = get_block_id();
  if(current_block == EMPTY_BLOCK){
    set_light(false);
  }
  if(placement_count > MAX_COUNT_PROB || previous_block == current_block){
    return;
  }
  Serial.print("Previous block = "); Serial.println(previous_block);
  Serial.print("Current block = "); Serial.println(current_block);
  Serial.print("Placement count = "); Serial.println(placement_count);
  if(previous_block == EMPTY_BLOCK && current_block != EMPTY_BLOCK){
    switch(placement_count){
      case 0: // 1st block placement
        set_light(true);
        break;        
      case 1: // 2nd block placement
        set_light(true);
        break;
      case 2: // 3rd block placement
        set_light(true);
        break;
      case 7: // 8th block placement (3rd block, second time)
        set_light(true);
        break;
      case 8: // 9th block placement (3rd block, thrid time)
        set_light(true);
        break;
      case 11: // last block placement (4th block, third time)
        set_light(true);
        break;
      default:
        set_light(false);
        break;
    }
    placement_count++;
  }
  else if(previous_block != EMPTY_BLOCK && current_block == EMPTY_BLOCK){
    set_light(false);
  }
  previous_block = current_block;
}

void run_det_mode(){
//   For deterministic condition:
// - 1st block makes machine light up 3/3 times
// - 2nd block makes machine light up 0/3 times
// - 3rd block makes machine light up 3/3 times
// - 4th block makes machine light up 3/3 times
  current_block = get_block_id();
  if(current_block == EMPTY_BLOCK){
    set_light(false);
  }
  if(placement_count > MAX_COUNT_PROB || previous_block == current_block){
    return;
  }
  Serial.print("Previous block = "); Serial.println(previous_block);
  Serial.print("Current block = "); Serial.println(current_block);
  Serial.print("Placement count = "); Serial.println(placement_count);
  if(previous_block == EMPTY_BLOCK && current_block != EMPTY_BLOCK){
    switch(placement_count){
      case 3: // 4th block placement (2nd block 1st time)
        set_light(false);
        break;
      case 4: // 5th block placement (2nd block, 2nd time)
        set_light(false);
        break;
      case 5: // 6th block placement (2nd block, 3rd time)
        set_light(false);
        break;  
      default:
        set_light(true);
        break;
    }
    placement_count++;
  }
  else if(previous_block != EMPTY_BLOCK && current_block == EMPTY_BLOCK){
    set_light(false);
  }
  previous_block = current_block;
}

  
void run_test_mode(){
//   Now kids get to play with the machine by themselves. Machine would be le
// “on.”

// - 6 blocks total
// - 3 make the machine go every time they’re placed on top (we gave all of these
// blocks magnets in each hole at the bottom of the block and the code said that if
// there is 1 or more magnet(s) on the machine, the light would turn on))
// - 3 never make the machine go (none of these blocks had magnets on them)
  current_block = get_block_id();
  if(current_block == EMPTY_BLOCK){
    set_light(false);
  }
  if(previous_block == EMPTY_BLOCK && current_block != EMPTY_BLOCK){
  set_light(true);
  }
  else {
  set_light(false);
  }
  //get_block_id();
  //delay(1000);
}

void run_admin_mode(){
  Serial.println("admin");
}

//void run_exp_mode(){
  //Serial.println(get_sensor_value());
//}

void reset(){
  placement_count = 0;
  previous_block = EMPTY_BLOCK;
  current_block = EMPTY_BLOCK;
  // rand1 = random(1,7);
  // rand2 = random(1,7);
  // while(rand2 == rand1){
  //   rand2 = random(1,7);
  // }
  // rand3 = random(1,7);
  // while(rand3 == rand2 || rand3 == rand1){
  //   rand3 = random(1,7);
  // }
  set_light(false);
  Serial.println("reset");
}

int get_block_id(){
  //delay(1000); 
  int block = EMPTY_BLOCK;
  bool active0 = is_sensor_active(hallSensorPin0);
  bool active1 = is_sensor_active(hallSensorPin1);
  bool active2 = is_sensor_active(hallSensorPin2);
  bool active3 = is_sensor_active(hallSensorPin3);
  bool active4 = is_sensor_active(hallSensorPin4);

  if(!active0 && !active1 && !active2 && !active3 && !active4 ){
    block = EMPTY_BLOCK;
  }
  else if(!active0 && active1 && !active2 && !active3 && !active4 || !active0 && !active1 && active2 && !active3 && !active4 || !active0 && !active1 && !active2 && active3 && !active4 || !active0 && !active1 && !active2 && !active3 && active4){
    block = BLOCK_1;
  }
  else if(!active0 && active1 && active2 && !active3 && !active4 || !active0 && !active1 && active2 && !active3 && active4 || !active0 && !active1 && !active2 && active3 && active4 || !active0 && active1 && !active2 && active3 && !active4){
    block = BLOCK_2;
  }
  else if(!active0 && active1 && !active2 && !active3 && active4 || !active0 && !active1 && active2 && active3 && !active4){
    block = BLOCK_3;
  }
  else if(!active0 && active1 && active2 && !active3 && active4 || !active0 && !active1 && active2 && active3 && active4 || !active0 && active1 && !active2 && active3 && active4 || !active0 && active1 && active2 && active3 && !active4){
    block = BLOCK_4;
  }
  else if(!active0 && active1 && active2 && active3 && active4){
    block = BLOCK_5;
  }
  else if(active0 && !active1 && !active2 && !active3 && !active4){
    block = BLOCK_6;
  }
  else{
   Serial.println("block id error");
  }
  Serial.print("Block ID: "); Serial.println(block);
  return block;
}

float get_sensor_value(int hallSensor){
  int sensorValue = analogRead(hallSensor);
  float voltage = sensorValue * (voltageRef / adcResolution);
  return voltage;
}

bool is_sensor_active(int hallSensor){
  float sensor_voltage = get_sensor_value(hallSensor);
  return ((sensor_voltage  < magnet_present_threshold_low) 
          || (sensor_voltage > magnet_present_threshold_high) );
}

void set_light(bool on){
  digitalWrite(ledPin, on ? HIGH : LOW);
  Serial.print("I'm setting light to: "); Serial.println(on);
}

int get_current_mode(){
  //delay(1000);
  Serial.print("Pin 1 is "); Serial.println(digitalRead(MODE_DET_PIN));
  Serial.print("Pin 2 is "); Serial.println(digitalRead(MODE_PROB_PIN));
  Serial.print("Pin 3 is "); Serial.println(digitalRead(MODE_TEST_PIN));
  Serial.print("Pin 4 is "); Serial.println(digitalRead(MODE_ADMIN_PIN));
  if(digitalRead(MODE_PROB_PIN) == HIGH){
    return MODE_PROB;
  }
  else if(digitalRead(MODE_DET_PIN) == HIGH){
    return MODE_DET;
  }
  else if(digitalRead(MODE_TEST_PIN) == HIGH){
    return MODE_TEST;
  }
  else if(digitalRead(MODE_ADMIN_PIN) == HIGH){
    return MODE_ADMIN;
  }
  else{
    return MODE_UNKNOWN;
  }
}

void run_exp_mode(){
  delay(1000);
  Serial.println(get_block_id());
  Serial.print("Sensor 0 is "); Serial.println(get_sensor_value(hallSensorPin0));
  Serial.print("Sensor 1 is "); Serial.println(get_sensor_value(hallSensorPin1));
  Serial.print("Sensor 2 is "); Serial.println(get_sensor_value(hallSensorPin2));
  Serial.print("Sensor 3 is "); Serial.println(get_sensor_value(hallSensorPin3));
  Serial.print("Sensor 4 is "); Serial.println(get_sensor_value(hallSensorPin4));
}



// void loopold() {
//   // Read analog value from sensor
//   //int sensorValue = analogRead(hallSensorPin);
  
//   // Convert analog value to voltage
//   //float voltage = sensorValue * (voltageRef / adcResolution);
  
//   // Calculate magnetic field strength (in Gauss)
//   // Center voltage (no magnetic field) is approximately 2.5V
//   float magneticFieldStrength = (voltage - 2.5) / (sensitivity / 1000.0); // Gauss
  
//   // Print results to Serial Monitor
//   Serial.print("Voltage: ");
//   Serial.print(voltage, 3); // Print voltage with 3 decimal places
//   Serial.print(" V, Magnetic Field Strength: ");
//   Serial.print(magneticFieldStrength, 3); // Print field strength with 3 decimal places
//   Serial.println(" G");
  
//   // Optional: Light up LED if strong field detected
//   if (abs(magneticFieldStrength) > 100) { // Adjust threshold as needed
//     digitalWrite(ledPin, HIGH);
//   } else {
//     digitalWrite(ledPin, LOW);
//   }
  
//   delay(500); // Delay for readability
// }
