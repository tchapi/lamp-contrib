/*
 * Facedetect Robot Control
 * ========================
 * 
 * Author: Marcus Bauer 2012,2013; Tom Igoe 2011 (serial code)
 * Copyright: Bearstech 2013
 * License: Apache Public License 2.0
 *          http://www.apache.org/licenses/LICENSE-2.0
 * 
 * - connect via Bluetooth to Android phone which does face detection
 * - get commands to move 4 servos
 * - get command to switch light
 * - send values from proximity sensor to phone
 * - bluetooth comm uses \n as record separator
 * 
 */


#include <Servo.h> 
#include <SoftwareSerial.h>   //Software Serial Port
#define RxD 2
#define TxD 4

#define DEBUG_ENABLED  1

SoftwareSerial blueToothSerial(RxD,TxD);

Servo myservo[4];
const int head_servo_tilt = 5;
const int head_servo_roll = 6;
const int hinge_servo_arm = 7;
const int hinge_servo_trunk = 8;

// variables to store the servo position and minima and maxima
int servo[4];
int SERVO_START[4];
int SERVO_MIN[4];
int SERVO_MAX[4];
int target[4];

boolean target_equals_actual = false;

// states of motion
//     TODO: this could be more elegantly done with bit banging
boolean play = true;
boolean proximity_lock = false;
boolean go_search  = true;
boolean go_left    = false;
boolean go_right   = false;
boolean go_up      = false;
boolean go_down    = false;
boolean go_back    = false;
boolean go_forward = false;

// output states
boolean relais_on = false;

// loop speed
const int LOOP_DELAY = 40;

// Bluetooth state pin
const int BT_STATE = A1;

// IR proximity pin
const int IR_SENSOR = A4;

// state
const int OKAY    =-1;
const int SEARCH  = 0;
const int LEFT	  = 1;
const int RIGHT	  = 2;
const int UP	  = 3;
const int DOWN	  = 4;
const int BACK	  = 5;
const int FORWARD = 6;
const int PLAY	  = 7;
const int NOPLAY  = 8;
const int LIGHT   = 9;
const int NOLIGHT = 10;
const int DIMLIGHT= 11;
const int RELAIS  = 12;
const int NORELAIS= 13;

// actions
const int PLUS    =  995;
const int MINUS   =  996;
const int RESET   =  998;
const int KEEP    =  999;

// LED strip
const int LED_STRIP = 3;

// LED to use for signalling
const int LED = 13;
int led_state = HIGH;

// variables for serial comm (bluetooth)
String inputString = "";          // a string to hold incoming data
boolean commandReceived = false;  // whether the string is complete
char charBuf[64];

// serial input watchdog
long patience = 5000;
long lastEvent = 0;
long currentTime = 0;
long lastFaceTime = 0;
long lastProximityTime = 0;

// seeedstudio bluetooth shield timer
long lastDisconnected = 0;
boolean btConnected = false;
boolean btConnectable = false;


void setup() {

  analogWrite(LED_STRIP, 250);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  delay(1000); 
  digitalWrite(LED, LOW);

  inputString.reserve(200);

  // asign pins to servos
  myservo[0].attach(head_servo_tilt);
  myservo[1].attach(head_servo_roll);
  myservo[2].attach(hinge_servo_arm);
  myservo[3].attach(hinge_servo_trunk);

  // define servo limits
  // tilt/roll
  SERVO_MIN[0] = 70; // forward -> down
  SERVO_MIN[1] = 20; // back -> down
  
  // arm/trunk
  SERVO_MIN[2] = 35; // right
  SERVO_MIN[3] = 60; // down

  // tilt/roll
  SERVO_MAX[0] = 155; // back -> up
  SERVO_MAX[1] = 140; // forward -> up
  
  // arm/trunk
  SERVO_MAX[2] = 140; // left
  SERVO_MAX[3] = 145; // up


  SERVO_START[0] = 140;
  SERVO_START[1] = 70;
  SERVO_START[2] = 90;
  SERVO_START[3] = 140;

  // set initial positions
  for (int i=0; i<4; i++) {
    servo[i]  = SERVO_START[i];
    target[i] = SERVO_START[i];
    myservo[i].write(SERVO_START[i]);
  }

  Serial.begin(9600);
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  setupBlueToothConnection();
  // Wait 1s and flush the serial buffer
  delay(1000);
  Serial.flush();
  blueToothSerial.flush();

}

void setupBlueToothConnection()
{
  blueToothSerial.begin(38400); //Set BluetoothBee BaudRate to default baud rate 38400
  blueToothSerial.print("\r\n+STWMOD=0\r\n");//set the bluetooth work in master mode
  blueToothSerial.print("\r\n+STNA=SmartLamp\r\n");//set the bluetooth name as "SeeedBTMaster"
  blueToothSerial.print("\r\n+STOAUT=1\r\n");
  blueToothSerial.print("\r\n+STAUTO=0\r\n");// Auto-connection is forbidden here
  delay(2000); // This delay is required.
  blueToothSerial.print("\r\n+INQ=1\r\n");//make the master inquire
  Serial.println("Done setuping Bluetooth.");
  delay(2000); // This delay is required.
  btConnectable = true;
}


void loop() {

  // We cannot use serialEvent with SoftwareSerial
  mySerialEvent();

  //---------------------------------------------------------
  // check if bluetooth is connected - else blink and RETURN
  //---------------------------------------------------------
  int bluetooth_state = analogRead(BT_STATE); 
  delay(1);
  
  if (bluetooth_state < 100) {  // actually 0 on N/C and 667 on connect
    blink(400);
    btConnected = false;
    if (!btConnectable) {
      blueToothSerial.print("\r\n+INQ=1\r\n");
      btConnectable = true;
    }
    return;
  } 
  else {
    led_state = HIGH;
    digitalWrite(LED, led_state);
    if (!btConnected) {
      btConnected = true;
      btConnectable = false;
      delay(1500); // heisenbug
    }
  }

  //---------------------------
  // proximity sensor
  //---------------------------
  /*
  int distance = get_ir_sensor();
  if (distance > 450) {
    Serial.print("PROXIMITY,");
    Serial.println(distance);
    lastProximityTime = millis();
  }
  */
  proximity_lock = false;
  // proximity_lock = ( (millis() - lastProximityTime)  < 5000 ) ? true : false;

  //------------------------------------
  // loop() delay to adjust servo speed
  //------------------------------------
  delay(LOOP_DELAY);

  //-----------------------
  // serial input watchdog
  //-----------------------
  currentTime = millis();
  //if (currentTime - lastEvent > patience)
  //	go_straight_up();

  //------------
  // movements
  //------------
  // check if play is on and no proximity event
  if (play &&  !proximity_lock ) {
    check_state();
    go_to_target();
  }

  //-----------------------------------
  // parse serial command from Android
  //-----------------------------------
  if (commandReceived) {

    // split string on "," to get first argument which is
    // the command 
    inputString.toCharArray(charBuf, 64);
    char *p = charBuf;
    char *str;
    str = strtok_r(p, ",", &p);

    // if we got something, then compare command
    if(str != NULL) {
      if (strcmp(str,"search")==0) {
        set_state(SEARCH);
      }
      else if (strcmp(str,"okay")==0) {
        set_state(OKAY); 
      }			
      else if (strcmp(str,"left")==0) {
        set_state(LEFT); 
      }
      else if (strcmp(str,"right")==0) {
        set_state(RIGHT);
      }
      else if (strcmp(str,"up")==0) {
        set_state(UP);
      }
      else if (strcmp(str,"down")==0) {
        set_state(DOWN);
      }
      else if (strcmp(str,"forward")==0) {
        set_state(FORWARD);
      }
      else if (strcmp(str,"back")==0) {
        set_state(BACK);
      }
      // voice commands
      else if (strcmp(str,"play")==0) {
        set_state(PLAY);
      }
      else if (strcmp(str,"noplay")==0) {
        set_state(NOPLAY);
      }
      else if (strcmp(str,"light")==0) {
        set_state(LIGHT);
      }
      else if (strcmp(str,"nolight")==0) {
        set_state(NOLIGHT);
      }
      else if (strcmp(str,"dimlight")==0) {
        set_state(DIMLIGHT);
      }

      else {
        Serial.print("CMD:");
        Serial.println(inputString);
        delay(3000);
      }
    }

    // check second argument
    //str = strtok_r(p, ",", &p);
    //if(str != NULL) {
    //	int i = atoi(str);
    //	Serial.println(i);
    //}

    // clear the string:
    inputString = "";
    commandReceived = false;
  }
}


//======================================================================
// serial event
//======================================================================
/*
 SerialEvent occurs whenever new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void mySerialEvent() {


  while (blueToothSerial.available()) {

    char inChar = (char)blueToothSerial.read(); 

    // discard old commands
    if (commandReceived == true) {
      commandReceived = false;
      inputString = "";
    }

    // add it to the inputString:
    // if the incoming character is a newline, set a flag
    // so the main loop will use it:
    if (inChar == '\n') {
      commandReceived = true;
    } 
    else
      inputString += inChar;

  }
  
}



void set_state(int state){

  reset_states();

  if(play && !proximity_lock) {
    switch (state) {
    case SEARCH:
      go_search = true;
      analogWrite(LED_STRIP, 50);
      Serial.println("SEARCH");
      break;
    case OKAY:
      lastFaceTime = millis();
      analogWrite(LED_STRIP, 200);
      Serial.println("OKAY");
      break;
    case LEFT:
      go_left = true;
      lastFaceTime = millis();
      Serial.println("LEFT");
      break;
    case RIGHT:
      go_right = true;
      lastFaceTime = millis();
      Serial.println("RIGHT");
      break;
    case UP:
      go_up = true;
      lastFaceTime = millis();
      Serial.println("UP");
      break;
    case DOWN:
      go_down = true;
      lastFaceTime = millis();
      Serial.println("DOWN");    
      break;
    case FORWARD:
      go_forward = true;
      lastFaceTime = millis();
      Serial.println("FORWARD");
      break;
    case BACK:
      go_back = true;
      lastFaceTime = millis();
      Serial.println("BACK");
      break;
    }
  }

  // voice commands	
  //else if (!play) {
  switch (state) {
  case PLAY:
    play = true;
    Serial.println("PLAY");
    delay(1000);
    break;
  case NOPLAY:
    play = false;
    Serial.println("NOPLAY");
    delay(1000);
    break;
  case LIGHT:
    analogWrite(LED_STRIP, 250);
    Serial.println("LIGHT");
    delay(1000);
    break;
  case NOLIGHT:
    analogWrite(LED_STRIP, 0);
    Serial.println("NOLIGHT");
    delay(1000);
    break;
  case DIMLIGHT:
    Serial.println("DIM");
    analogWrite(LED_STRIP, 80);
    delay(1000);
    break;
  default:
    //Serial.println("Default nothing");
    break;
  }
  //}
}

//-----------------------------------------------------------------------------
// check_state()   -   called in loop(), calls set_target() depending on state
//-----------------------------------------------------------------------------

void check_state() {

  if (go_search)
    search();
  else if(go_left)
    set_target(RESET, RESET, PLUS, RESET);
  else if (go_right)
    set_target(RESET, RESET, MINUS, RESET);
  else if (go_up)
    set_target(RESET, RESET, RESET, PLUS);
  else if (go_down)
    set_target(RESET, RESET, RESET, MINUS);
  else if (go_back)
    set_target(PLUS, MINUS, RESET, RESET);
  else if (go_forward)
    set_target(MINUS, PLUS, RESET, RESET);
  else
    set_target(RESET, RESET, RESET, RESET);
}




void set_target(int t1, int t2, int t3, int  t4) {

  int new_target[4];

  new_target[0]=t1;
  new_target[1]=t2;
  new_target[2]=t3;
  new_target[3]=t4;

  int t;

  for (int i=0; i<4; i++) {
    t = new_target[i];

    if (t==RESET) {
      target[i] = servo[i];
    }
    else if (t==KEEP) {
      ;
    }
    else if (t==MINUS) {
      if(target[i]>SERVO_MIN[i])
        target[i]--;
    }
    else if (t==PLUS) {
      if(target[i]<SERVO_MAX[i]) {
        target[i]++;
      }
    }
    else if (t >= SERVO_MIN[i] && t <= SERVO_MAX[i]) {
      target[i] = t;
    }
    else
      Serial.println("UNDEF set_target()");
  }
  target_equals_actual = false;

}


void go_to_target() {

  target_equals_actual = true;
  int delay_ms=5;

  for (int j=0; j<4; j++) {
    if(servo[j]<target[j] && servo[j] < SERVO_MAX[j]) {
      servo[j]++;
      target_equals_actual = false;
    }
    else if (servo[j]>target[j] && servo[j] > SERVO_MIN[j]) {
      servo[j]--;
      target_equals_actual = false;
    }
    else {
      delay_ms = 0;
    }
    myservo[j].write(servo[j]);
    delay(delay_ms);
  }
}


void search() {

  long sTime = millis();

  // wait 2.5sec before starting a search
  if( (millis() - lastFaceTime) < 2500) {
    Serial.println("WAITING");
    return;
  }

  static int stepper = 0;

  // nothing to do as we are still moving
  if (!target_equals_actual)
    return;
  // MID base position
  if (stepper == 0) {
    //set_target(90,60,30,115);
    stepper++;
  } 
  else if (stepper == 1) {
    //set_target(90,60,30,160);
    stepper++;
  } 
  else if (stepper == 2) {
    //set_target(90,60,30,60);
    stepper++;
  } 
  else if(stepper == 3) {
    //set_target(90,60,30,115);
    stepper++;
    // full left
  } 
  else if(stepper == 4) {
    set_target(20,60,30,115);
    stepper++;
  } 
  else if(stepper == 5) {
    set_target(20,60,155,100); //testing
    stepper++;
  } 
  else if(stepper == 6) {
    set_target(20,60,155,112);
    stepper++;
  } 
  else if(stepper == 7) {
    set_target(20,60,155,100);
    stepper++;
    // full right
  } 
  else if(stepper == 8) {
    set_target(160,60,35,100);
    stepper++;
  } 
  else if(stepper == 9) {
    set_target(160,60,35,112);
    stepper++;
  } 
  else if(stepper == 10) {
    set_target(160,60,35,60);
    stepper++;
  } 
  else if(stepper == 11) {
    set_target(160,60,30,115);		
    stepper=0;
  }

}





void reset_states() {
  go_search = go_left = go_right = go_up = go_down = go_back = go_forward = false;
}



void blink(long interval){

  static long previousMillis = 0;
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;   

    // if the LED is off turn it on and vice-versa:
    led_state = (led_state == LOW) ? HIGH : LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(LED, led_state);
  }

}


int get_ir_sensor()
{
  int sensor_value;
  int sum;

  // continuous sampling 20 times
  for (int i=0; i<20; i++)
  {
    sensor_value = analogRead(IR_SENSOR); 
    sum += sensor_value;
  }
  sensor_value = sum / 20;
  delay(1);
  return sensor_value;
} 






