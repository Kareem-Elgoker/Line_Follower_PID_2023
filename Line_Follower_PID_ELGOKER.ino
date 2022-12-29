
//@Author: Kareem Eljoker & Kareem Abdelaziz

// LCD I2C Libraries
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Initialize lcd module
LiquidCrystal_I2C lcd(0x27,16,2); 

// Make constants for speed and direction pins
enum MotorsSpeed{EN_A = 5, EN_B};   
enum MotorsDirictions{IN_1 = 3, IN_2, IN_3 = 7, IN_4};

// Initialize sensors pins
enum sensors{SEN1 = 9, SEN2, SEN3, SEN4, SEN5}; 

// Make constant variable indecate for a line : (1) for black line , (0) for white line
const int LINE = 1;


// Store motor pins in an array to loop over them
int MotorPins[6] = {EN_A, IN_1, IN_2, EN_B, IN_3, IN_4};

// Store sensors pins in an array to loop over them
int Sensors[5] = {SEN1, SEN2, SEN3, SEN4, SEN5};

// Variable that store the sensors readings in a binary representation
int readings;

// Initialize base speed for straight forward and the maximum speed for motors
// // These two constants can be modified over testing the project
const int base_speed = 190, max_speed = 250;

// This variable is used to make a delay in the first run of the robot
int is_first_turn_on = 1;

//////////////////////////////////////////////////////////////////////

// LCD functions that controle its operations
void lcd_display_team_name();
void lcd_change_state();

// Motors functions that controle their operations
void set_speed_for_both_wheels(int speed);
void forward_both_wheels();
void stop_both_wheels();

void set_speed_for_left_wheel(int speed);
void set_speed_for_right_wheel(int speed);

void forward_left_wheel();
void forward_right_wheel();

void stop_left_wheel();
void stop_right_wheel();

// These functions Control the alignment of the robot on the line based on the PID control
void get_readings();
int get_error();

double PID_Controle();
void adjust_motors_speed();

//////////////////////////////////////////////////////////////////////

void setup()
{

  lcd.init();                     
  lcd.init();
  lcd.backlight();

  // Initialize Motor Pins as OUTPUT
  for(int i = 0; i < 6; i++) {
    pinMode(MotorPins[i], OUTPUT);
  }

  // Initialize Sensors Pins as INPUT
  for(int i = 0; i < 5; i++) {
    pinMode(Sensors[i], INPUT);
  }

  lcd_display_team_name();

  // Initialize the base speed for motors at the begining
  set_speed_for_both_wheels(base_speed);
  delay(1500);
  forward_both_wheels();
  
}

void loop()
{
  
  // These three calling methods :
  // 1 - Get the readings of the 5 sensors
  // 2 - adjust motors speed to make it run on the line
  // 3 - Write the state of robot on the lcd
  get_readings();
  adjust_motors_speed();
  lcd_change_state();

}

//////////////////////////////////////////////////////////////////////

void set_speed_for_both_wheels(int speed) {
  set_speed_for_left_wheel(speed);
  set_speed_for_right_wheel(speed);
}
void forward_both_wheels(){
  forward_left_wheel();
  forward_right_wheel();
}
void stop_both_wheels()
{
  stop_left_wheel();
  stop_right_wheel();
}

void set_speed_for_left_wheel(int speed) {
  analogWrite(EN_A, speed);
}
void set_speed_for_right_wheel(int speed) {
  analogWrite(EN_B, speed);
}

void forward_left_wheel()
{
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);
}
void forward_right_wheel()
{
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
}

void stop_left_wheel()
{
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
}
void stop_right_wheel()
{
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
}

//////////////////////////////////////////////////////////////////////

void get_readings() {
  readings = B00000;
  for(int i = 0; i < 5; i++) {
    if(digitalRead(Sensors[i]) == LINE) readings += 1 << i;
  }
}

int get_error() {
  
  static int Error = 0;  

  switch(readings) {

    case B00111:
    case B01111:
      Error = 5;
      break;
    case B00001:
      Error = 4;
      break;
    case B00011:
      Error = 3;
      break;
    case B00010:
      Error = 2;
      break;
    case B00110:
      Error = 1;
      break;
    case B00100:
    case B11111:
      Error = 0;
      break;
    case B01100:
      Error = -1;
      break;
    case B01000:
      Error = -2;
      break;
    case B11000:
      Error = -3;
      break;
    case B10000:
      Error = -4;
      break;
    case B11100:
    case B11110:
      Error = -5;
      break;
    default:
      Error = Error;
  }
  
  return Error;
}

double PID_Controle() {

  static int last_error = 0, I = 0;

  // These three constants can be modified over testing the project
  double Kp = 69;
  double Kd = 650;
  double Ki = 0;

  int error = get_error();

  int P = error;
  int D = error - last_error;
  I += error;
  
  last_error = error;

  return Kp * P + Kd * D + Ki * I;

}

void adjust_motors_speed() {

  double PID_Const = PID_Controle();
  
  int Left_Motor_Speed = base_speed - PID_Const;
  int Right_Motor_Speed = base_speed + PID_Const;

  if (Left_Motor_Speed > max_speed) {
    forward_left_wheel();
    Left_Motor_Speed = max_speed;
  }
  else if (Left_Motor_Speed < 0) {
    stop_left_wheel();
  }
  else forward_left_wheel();

  if (Right_Motor_Speed > max_speed) {
    forward_right_wheel();
    Right_Motor_Speed = max_speed;
  }
  else if (Right_Motor_Speed < 0) {
    stop_right_wheel();
  }
  else forward_right_wheel();

  set_speed_for_left_wheel(Left_Motor_Speed);
  set_speed_for_right_wheel(Right_Motor_Speed);

}

//////////////////////////////////////////////////////////////////////

void lcd_display_team_name() {

  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.println("Cyberious    ");
  lcd.setCursor(5, 1);
  lcd.println("START");

}

void lcd_change_state() {

  static int state = -1;

  int error = get_error();
  
  if(error == 0 && state != 0) {
    state = 0;
    lcd.setCursor(5, 1);
    lcd.println("forward");
  }
  else if (error < 0 && state != 1) {
    state = 1;
    lcd.setCursor(5, 1);
    lcd.println(" right ");
  }
  else if (error > 0 && state != 2){
      state = 2;
      lcd.setCursor(5, 1);
      lcd.println(" left  ");
  }
}
