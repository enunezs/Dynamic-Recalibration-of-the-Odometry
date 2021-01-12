#include "lineSensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "motor.h"
//#include "LinearMotor_c.h" //The beautiful result from my table ;(
#include "beeper.h"
#include "pid.h"
#include "lineSensorsSystem.h"

#include "node.h"
#include "nodeController.h"

#define ROMI_DIAMETER 0.132f //mm
#define WHEEL_DIAMETER 0.07f //mm

#define TICKS_PER_TURNS 12 //12 ticks per revolution
#define TURNS_PER_REV 120 //120 motor turns -> 1 wheel turn
#define TICKS_PER_REV 1440 //1440 ticks per motor revolution

#define TICKS_TO_MM 0.152f
#define MM_TO_TICKS 6.57894f

//Preincluded constants, just as reminder
/*
  #define PI 3.1415926535897932384626433832795
  #define HALF_PI 1.5707963267948966192313216916398
  #define TWO_PI 6.283185307179586476925286766559
  #define DEG_TO_RAD 0.017453292519943295769236907684886
  #define RAD_TO_DEG 57.295779513082320876798154814105
*/

#define BAUD_RATE 9600
#define BUZZER_PIN 25

#define LOOP_DELAY 20

#define LINE_LEFT_PIN A2
#define LINE_CENTER_PIN A3
#define LINE_RIGHT_PIN A4

//*******************//
//***State Machine***//
//*******************//

enum state {e_start, e_drive_forward, e_follow_line, e_explore, e_return_home, e_stop, e_center_on_node, e_found_node, e_go_to_prev_node, e_recalibrate, e_go_to_cur_node};
enum state current_state = e_start;

unsigned long last_timestamp = 0;
float line_confidence = 0;
float node_confidence = 0;

//Time the Romi explores around
int exploration_time = 35; //secs

//*************//
//***Control***//
//*************//

// PIDs Calibrated for PID freq of 50Hz, encoder update freq of 20Hz
// DO NOT CHANGE
float pid_freq = 50; //Hz
long pid_timestamp = 0;
// Left PID
float Kp_left = 0.03f;//.2; //Proportional gain
float Kd_left = -.01f;//6f;//-1; //Derivative gain
float Ki_left = 0.0005f;//20f; //Integral gain
PID_c left_PID(Kp_left, Ki_left, Kd_left); // controller for left wheel
// Right PID
float Kp_right = 0.035f;//.2; //Proportional gain
float Kd_right = -.01f;//6f;//-1; //Derivative gain
float Ki_right = 0.0005f;//20f; //Integral gain
PID_c right_PID(Kp_right, Ki_right, Kd_right); // controller for right wheel
// Navigation PID
float Kp_demand = 2.5f;//.2; //Proportional gain
float Kd_demand = -0.0f; //6f;//-1; //Derivative gain
float Ki_demand = 0.0f;//20f; //Integral gain
PID_c demand_PID(Kp_demand, Ki_demand, Kd_demand); // controller for right wheel

//****************//
//***Kinematics***//
//****************//
int kinematics_freq = 25; //Hz
long kinematics_timestamp = 0;
kinematics_c kinematics;

//********************//
//***Motors & Beeps***//
//********************//

motor_c simple_motor;

//Our dear experiment
//linearMotor_c linear_motor;

beeper_c beeper;

//*************//
//***Sensors***//
//*************//

lineSensorsSystem_c lineSensors(LINE_LEFT_PIN, LINE_CENTER_PIN, LINE_RIGHT_PIN);

//***********//
//***Nodes***//
//***********//

nodeController_c nodeController;

//------------------------------------SETUP------------------------------------//

//***********//
//***Setup***//
//***********//

void setup() {


  // Interrupt setups
  beeper.beep();
  setupEncoder0();
  setupEncoder1();
  setupTimer3();

  // Serial
  Serial.begin(BAUD_RATE);
  delay(1000);
  Serial.println(" ***Reset*** ");

  beeper.tick();

  last_timestamp = millis();

  //beeper.tick();
  //lineSensors.calibrateLineSensor();

  //rotateRobot(-TWO_PI, false);
  //rotateRobot(-TWO_PI, false);

}


//------------------------------------MAIN-LOOP------------------------------------//

//***************//
//***Main Loop***//
//***************//

void loop()
{
  //Where the code actually happens
  stateMachine();
  //Update kinematics
  if (millis() - kinematics_timestamp > 1 / (float)kinematics_freq) {
    kinematics_timestamp  = millis();
    kinematics.update();
  }

  //Update line sensors
  lineSensors.updateSensors();

  //lineSensors.printSensors();

  //lineSensors.printSensors();
  delay(LOOP_DELAY);

}

//*******************//
//***State Machine***//
//*******************//

boolean state_done = false;

void stateMachine() {
  switch (current_state) {

    case e_start:
      state_done = stateCalibrate();
      //current_state = e_center_on_node;
      break;

    case e_drive_forward:
      state_done = stateDriveForward();
      break;

    case e_follow_line:
      state_done = stateFollowLine();
      break;

    case e_explore:
      state_done = stateExplore();
      break;

    case e_return_home:
      state_done = stateReturnHome();
      break;

    case e_go_to_prev_node:
      state_done = stateGoToPrevNode();
      break;

    case e_center_on_node:
      state_done = stateCenterOnNode();
      break;

    case e_recalibrate:
      state_done = stateRecalibrate();
      break;

    case e_go_to_cur_node:
      state_done = stateGoToCurNode();
      break;
  }

  //State change
  if (state_done) {
    state_done = false;
    left_PID.reset();
    right_PID.reset();
  }
}



boolean stateCalibrate() {

  delay(1000);
  lineSensors.calibrateLineSensor();
  left_PID.reset();
  right_PID.reset();

  //Change state
  Serial.println("To drive forward");
  current_state = e_drive_forward;
  return true; //Done
}


//Go forward until line is found
//Returns true if line is found
//Changes state to line following if line is found
boolean stateDriveForward() {

  //int forward_power = e20;
  int demand_speed = 200;
  float confidence_threshold = 0.3f;

  //Update confidence
  line_confidence = lineSensors.updateConfidence(line_confidence);

  if (line_confidence <= confidence_threshold) {
    //If not, just keep driving forward
    //TODO -> Drive straight

    float output  = 0;
    output = left_PID.update(demand_speed, e0_speed);
    simple_motor.leftMotor(output);
    output = right_PID.update(demand_speed, e1_speed);
    simple_motor.rightMotor(output);
  }

  else {
    //if the confidence turns high switch to line following

    //Change state
    //Serial.println("Found line");
    Serial.println("To follow line");

    current_state = e_follow_line;

    left_PID.reset();
    right_PID.reset();
    return true;
  }
  return false;
}


//Follows line until confidence goes below threshold
//Returns true if confidence drops below threshold
//Changes state to ?

float max_conf = 0;
float max_x = 0;
float max_y = 0;
float max_theta = 0;

boolean stateFollowLine() {

  //Update
  unsigned long time_now = millis();

  line_confidence = lineSensors.updateConfidence(line_confidence);
  node_confidence = lineSensors.updateNodeConfidence(node_confidence);

  //*****************************//
  //Refresh PID and update motors//
  //*****************************//

  if (time_now - pid_timestamp > (float)(1000.0 / pid_freq)) {

    pid_timestamp = time_now;
    //Update the light sensor direction
    float demand_error = lineSensors.weighted_sensing(); //weighted_sensing(line_sensor_left._prev_value, line_sensor_center._prev_value, line_sensor_right._prev_value);
    //Use the confidence as a baseline for speed, with a constant term
    float demand_speed = 200 * (line_confidence) + 150;
    float correction = demand_PID.update(0, demand_error);

    //Update PID and motors
    float output  = 0;
    //float base_speed = 100;

    output = left_PID.update((1 - correction ) * demand_speed, e0_speed);
    simple_motor.leftMotor(output);
    output = right_PID.update((1 + correction ) * demand_speed, e1_speed);
    simple_motor.rightMotor(output);
  }



  ///////////////////
  //REGISTER A NODE//
  ///////////////////

  //New approach, following line as usual note events
  //Implement window at a small point
  if (node_confidence > 0.3f) {
    if (max_conf == 0) {
      //Serial.println("Event started: found line");
    }
    //tmimespan?
    if (node_confidence > max_conf) {
      max_conf = node_confidence;
      max_x = kinematics.x_pos;
      max_y = kinematics.y_pos;
      max_theta = kinematics.theta;

    }
  }
  else {
    if (max_conf > 0) {
      //Serial.println("event over");
      if (max_conf >= 0.30f) {
        //Try adding the node
        boolean added = addNode(max_x, max_y, max_theta);
        beeper.smallBeep();

        //If the succesfully added (and is not the first node) change state
        if (added && nodeController.getNodeNumber() > 1) {
          Serial.println("stateFollowLine: Going to prev node");
          current_state = e_go_to_prev_node;
          left_PID.reset();
          right_PID.reset();

          return true;
        }
      }

      //Reset vars for next
      max_conf = 0;
      max_x = 0;
      max_y = 0;
      max_theta = 0;

    }
  }

  ////////////////
  //CHANGE STATE//
  ////////////////
  float confidence_threshold = 0.3f;

  if (line_confidence < confidence_threshold) {


    //if the confidence turns high switch to line following
    //Change state
    Serial.println("stateFollowLine: Lost line");
    current_state = e_explore;
    left_PID.reset();
    right_PID.reset();
    return true;
  }




  return false;
}

//Adds node at current position
//For the initial node

boolean addNode() {

  long * sensor_pos;
  sensor_pos  = kinematics.getSensorPos();

  //Check for repeats
  if (  ! nodeController.checkNode_R(sensor_pos[0] , sensor_pos[1])) {
    nodeController.addNode(sensor_pos[0] , sensor_pos[1], kinematics.theta);

    Serial.print("New node at x ");
    Serial.print(sensor_pos[0]);
    Serial.print(" y ");
    Serial.println(sensor_pos[1]);
    return true;
  } else {
    Serial.println("Node overlap found, NOT added");
    return false;

  }

}


//Adds node at specified robot position accounting for sensor offset
boolean addNode(long _x, long _y, float _theta) {
  long * sensor_pos;
  sensor_pos  = kinematics.getSensorPos(_x, _y, _theta);

  if (  ! nodeController.checkNode_R(sensor_pos[0] , sensor_pos[1])) {
    nodeController.addNode(sensor_pos[0] , sensor_pos[1], _theta);

    Serial.print("New node at x ");
    Serial.print(sensor_pos[0]);
    Serial.print(" y ");
    Serial.println(sensor_pos[1]);
    return true;

  } else {
    Serial.println("Node overlap found, NOT added");
    return false;

  }

}


boolean stateGoToPrevNode() {

  Serial.println("stateGoToPrevNode: Going to prev node");

  int prev_node_num = nodeController.getNodeNumber() - 2;
  Node_c prev_node = nodeController.getNode(prev_node_num);
  long * node_pos = prev_node.getNodeLoc();

  moveToPlace(node_pos[0]*TICKS_TO_MM, node_pos[1]*TICKS_TO_MM);
  moveToPlace(node_pos[0]*TICKS_TO_MM, node_pos[1]*TICKS_TO_MM);

  current_state = e_center_on_node;

  return true;
}


//Center on node in two stages
//First: Rotate to max alignment
//Second: Move to max alignment
enum centering_state {e_scan_start = 0 , e_scan_left = 1, e_scan_right = 2, e_scan_done = 3, e_scan_front, e_scan_back};
int cur_centering_state = e_scan_start;
long turn_timestamp;
int turn_time = 1000; //In millis
int iter = 2;
boolean stateCenterOnNode() {

  //Update
  unsigned long time_now = millis();

  //Update sensors
  line_confidence = lineSensors.updateConfidence(line_confidence);
  node_confidence = lineSensors.updateNodeConfidence(node_confidence);



  //Set motor direction
  int rot_speed = 200;
  switch (cur_centering_state)
  {
    case e_scan_start:
      //rot_speed = 0;
      turn_timestamp = time_now;
      cur_centering_state = e_scan_back;
      Serial.println("Centering on node: start");
      max_conf = 0;
      max_x = 0;
      max_y = 0;
      max_theta = 0;

      simple_motor.rightMotor(0);
      simple_motor.leftMotor(0);
      left_PID.reset();
      right_PID.reset();
      iter -= 1;
      break;

    case e_scan_left:
      break;

    case e_scan_right:
      rot_speed = -rot_speed;
      break;

    case e_scan_front:
      break;

    case e_scan_back:
      rot_speed = -rot_speed;
      break;
  }

  //If in active sub-state
  if (cur_centering_state == e_scan_right || cur_centering_state == e_scan_left || cur_centering_state == e_scan_front || cur_centering_state == e_scan_back) {

    //*************//
    //Motor Methods//
    //*************//

    //Move left and right
    if (cur_centering_state == e_scan_right || cur_centering_state == e_scan_left) {
      if (time_now - pid_timestamp > (float)(1000.0 / pid_freq)) {

        pid_timestamp = time_now;

        float demand_error = 0;
        if (cur_centering_state == e_scan_left) {
          demand_error = (e0_speed - e1_speed) / TICKS_PER_REV;
        } else if (cur_centering_state == e_scan_right) {
          demand_error = -(e0_speed - e1_speed) / TICKS_PER_REV;
        }
        float correction = demand_PID.update(0, demand_error);

        //Update PID and motors
        float output  = 0;
        //float base_speed = 100;

        output = left_PID.update(-rot_speed * (1 + demand_error), e0_speed);
        simple_motor.leftMotor(output);
        output = right_PID.update(rot_speed * (1 + demand_error), e1_speed);
        simple_motor.rightMotor(output);
      }
    }

    //Move backward and forwards
    else if (cur_centering_state == e_scan_front || cur_centering_state == e_scan_back) {
      if (time_now - pid_timestamp > (float)(1000.0 / pid_freq)) {

        pid_timestamp = time_now;

        float demand_error = (e0_speed - e1_speed) / TICKS_PER_REV;
        float correction = demand_PID.update(0, demand_error);

        float output = left_PID.update(rot_speed * (1 + demand_error), e0_speed);
        simple_motor.leftMotor(output);
        output = right_PID.update(rot_speed * (1 - demand_error), e1_speed);
        simple_motor.rightMotor(output);

      }
    }

    //*******************//
    //Update best finding//
    //*******************//

    //If found, take note of better positions
    if (node_confidence > max_conf) {
      max_conf = node_confidence;
      max_x = kinematics.x_pos;
      max_y = kinematics.y_pos;
      max_theta = kinematics.theta;
      Serial.print("Found local max of ");
      Serial.println(max_conf);
    }

    //If very low absolute confidence (exploration over) and time is up
    //OR large drecrease in confidence
    if ( (node_confidence <= 0.05f && (time_now - turn_timestamp) > turn_time )) {

      //*****************//
      //Transition states//
      //*****************//

      if (cur_centering_state == e_scan_back) {
        Serial.println("Centering forwards");
        cur_centering_state = e_scan_front;
        turn_timestamp = time_now;

      } else if (cur_centering_state == e_scan_front) {
        Serial.println("Centering left");
        cur_centering_state = e_scan_left;
        turn_timestamp = time_now;

        //move to max pos
        //moveToPlace(max_x*TICKS_TO_MM, max_y*TICKS_TO_MM);

      } else if (cur_centering_state == e_scan_left) {
        Serial.println("Centering right");
        cur_centering_state = e_scan_right;
        turn_timestamp = time_now;

        //If last state
      } else if (cur_centering_state == e_scan_right && iter > 0) {
        Serial.println("Centering going for a second round");
        cur_centering_state = e_scan_start;
        turn_timestamp = time_now;


      } else if (cur_centering_state == e_scan_right) {

        cur_centering_state = e_scan_done;

        simple_motor.rightMotor(0);
        simple_motor.leftMotor(0);

        //If found a node
        if (max_conf > 0.3f) {
          Serial.println("found node");

          beeper.beep();
          delay(400);
          beeper.beep();

          faceAngle(max_theta);
          delay(1000);
          //Do a second pass


          Serial.println("centering over, found node");
          cur_centering_state = e_scan_start;

          //current_state = e_recalibrate;
          current_state = e_go_to_cur_node;

        }
        else {

          beeper.smallBeep();

          Serial.println("centering over, could not find node");
          cur_centering_state = e_scan_start;
          current_state = e_go_to_cur_node;

        }

        /*
          Serial.print("Found max at x ");
          Serial.print(max_x);
          Serial.print(" y ");
          Serial.print(max_y);
          Serial.print(" conf ");
          Serial.println(max_conf);
        */

        //Reset vars for next iteration
        max_conf = 0;
        max_x = 0;
        max_y = 0;
        iter = 2;

        turn_timestamp = time_now;

        return true;

      }
    }

  } else {
    simple_motor.rightMotor(0);
    simple_motor.leftMotor(0);
    left_PID.reset();
    right_PID.reset();
  }



  return false;
}


//Recalibrate Romi position
boolean stateRecalibrate() {

  float correction_factor = 1.0f;


  int prev_node_num = nodeController.getNodeNumber() - 2;
  Node_c prev_node = nodeController.getNode(prev_node_num);

  //The sensor is centered at the node in this position, therefore we are at node's position plus the sensor offset
  //Node is here
  long * prev_node_pos = prev_node.getNodeLoc();
  long * robot_node_pos;

  //Robot is here according to node
  robot_node_pos = kinematics.sensorToRobot(prev_node_pos[0], prev_node_pos[1]);

  //sensor_pos  = kinematics.getSensorPos(_x, _y, _theta);
  Serial.print("kinematics say robot at x ");
  Serial.print(  kinematics.x_pos);
  Serial.print(" y ");
  Serial.println(  kinematics.y_pos);


  Serial.print("Robot according to node at: x ");
  Serial.print(robot_node_pos[0]);
  Serial.print(" y ");
  Serial.println(robot_node_pos[1]);

  //Get difference
  long delta_x = robot_node_pos[0] - kinematics.x_pos;
  long delta_y = robot_node_pos[1] - kinematics.y_pos;

  //fix current node
  int cur_node_num = nodeController.getNodeNumber() - 1;
  Node_c cur_node = nodeController.getNode(cur_node_num);
  long * cur_node_pos = cur_node.getNodeLoc();

  cur_node.setNode(0, robot_node_pos[0] + delta_x * correction_factor, robot_node_pos[1] + delta_y * correction_factor);

  Serial.print("Current node was at x ");
  Serial.print(cur_node_pos[0]);
  Serial.print(" y ");
  Serial.println(cur_node_pos[1]);

  Serial.print("moved to x ");
  Serial.print(robot_node_pos[0] + delta_x * correction_factor);
  Serial.print(" y ");
  Serial.println(robot_node_pos[1] + delta_y * correction_factor);

  //fix robot

  kinematics.setPose(robot_node_pos[0], robot_node_pos[1]);

  Serial.print("Robot calibrated to: x ");
  Serial.print(robot_node_pos[0]);
  Serial.print(" y ");
  Serial.println(robot_node_pos[1]);

  Serial.println("Recalibration over");
  current_state = e_go_to_cur_node;

  return true;

}

boolean stateGoToCurNode() {

  Serial.println("stateGoToCurNode: Going to current node");

  int cur_node_num = nodeController.getNodeNumber() - 1;
  Node_c cur_node = nodeController.getNode(cur_node_num);
  long * node_pos = cur_node.getNodeLoc();

  moveToPlace(node_pos[0]*TICKS_TO_MM, node_pos[1]*TICKS_TO_MM);
  moveToPlace(node_pos[0]*TICKS_TO_MM, node_pos[1]*TICKS_TO_MM);

  Serial.println("stateGoToCurNode: Looking ahead");

  faceAngle(cur_node.getNodeAngle());
  moveRobot(4, true);


  current_state = e_explore;


  return true;
}


boolean stateExplore() {

  //Check right
  //Serial.println("Exploring");
  //Serial.println("stateExplore: Going right");
  if (rotateRobot(-HALF_PI, true)) {

    //Found it!
    //Serial.println("stateExplore: Line found");
    //Serial.println("To follow line");

    current_state = e_follow_line;
    //delay(500);
    return true;

  } else {

    //Check left
    //Serial.println("stateExplore: Going left");
    if (rotateRobot(PI, true)) {
      //Found it
      //Serial.println("stateExplore: Line found");
      //Serial.println("To follow line");

      current_state = e_follow_line;
      //delay(500);
      return true;

    } else {
      //Check gap
      //Serial.println("stateExplore: Line not found");
      //Serial.println("stateExplore: Going center");

      //Check center, go 6 cm
      rotateRobot(-HALF_PI, false);

      if (moveRobot(80, true)) {
        //Serial.println("stateExplore: Line found");
        //Serial.println("To follow line");

        current_state = e_follow_line;
        //delay(500);
        return true;

        //Are we just starting?
      } else if (millis() < exploration_time * 1000L) {
        //Yes, turn 180 and keep going
        Serial.println("stateExplore: Too early, keep going");
        rotateRobot(-PI, false);
        current_state = e_drive_forward;
        //delay(500);
        return true;

      } else {
        //Nothing found, we are done, go home!
        Serial.println("stateExplore: Line not found, going home");
        current_state = e_return_home;
        //delay(500);
        return true;
      }
    }
  }
  return false;
}



boolean stateReturnHome() {

  Serial.println("stateReturnHome: Going home");
  moveToPlace(0, 0);
  delay(200);
  moveToPlace(0, 0);

  current_state = e_stop;
  delay(500);
  Serial.println("stateReturnHome: Done");
  faceAngle(0);
  return true;

}





//------------------------------------METHODS------------------------------------//

//*********************//
//***Motor Methods***//
//*********************//

boolean rotateRobot(float angle, boolean search_line) {
  int rot_speed = 250;
  if (angle < 0) {
    rot_speed *= -1;
  }

  long current_e1_count = e1_count;
  long current_e0_count = e0_count;

  //1 encoder count equals 0.15mm of wheel travel
  //120 motor turns -> 1 wheel turn
  //12 ticks per revolution
  //1440 ticks per motor revolution

  //long conv_factor = TICKS_PER_REV * (ROMI_DIAMETER / WHEEL_DIAMETER) / TWO_PI;
  int conv_factor = 468; //Experimentally found -> ticks/rad
  //One rad turn is 468 encoder ticks
  long disp = angle * conv_factor;


  long target_e1_count = current_e1_count + disp;
  long target_e0_count = current_e0_count - disp;

  int disp_threshold = 5;

  left_PID.reset();
  right_PID.reset();

  while (abs(target_e1_count - e1_count) > disp_threshold && abs(target_e0_count - e0_count) > disp_threshold ) {
    unsigned long time_now = millis();
    if (time_now - pid_timestamp > (1000 / pid_freq)) {
      pid_timestamp = time_now;

      float demand_error = 0;
      if (angle < 0) {
        demand_error = (e0_speed - e1_speed) / TICKS_PER_REV;
      } else {
        demand_error = -(e0_speed - e1_speed) / TICKS_PER_REV;
      }
      float correction = demand_PID.update(0, demand_error);

      float output = left_PID.update(-rot_speed * (1 + demand_error), e0_speed);
      simple_motor.leftMotor(output);
      output = right_PID.update(rot_speed * (1 + demand_error), e1_speed);
      simple_motor.rightMotor(output);

      //Serial.println();

      if (search_line) {
        line_confidence = lineSensors.updateConfidence(line_confidence);
        if (line_confidence > 0.5f) {
          //Or better use confidence?
          simple_motor.rightMotor(0);
          simple_motor.leftMotor(0);
          kinematics.update();
          delay(LOOP_DELAY);
          return true;
        }
      }
    }
    if (millis() - kinematics_timestamp > 1 / (float)kinematics_freq) {
      kinematics_timestamp  = millis();
      kinematics.update();
    }
    delay(LOOP_DELAY);
  }

  simple_motor.rightMotor(0);
  simple_motor.leftMotor(0);

  return false;
}


//Moves forward distance in mm
//If search_line is true, stops when finding a line
boolean moveRobot(int distance, boolean search_line) {


  long current_e1_count = e1_count;
  long current_e0_count = e0_count;

  //1 encoder count equals 0.15mm of wheel travel

  //120 motor turns -> 1 wheel turn
  //12 ticks per revolution
  //1440 ticks per motor revolution

  //long conv_factor = TICKS_PER_REV / (WHEEL_DIAMETER * TWO_PI);
  // 1 encoder tick is roughly 0.152mm
  long disp = distance * MM_TO_TICKS;
  //long disp =  distance * conv_factor;


  long target_e1_count = current_e1_count + disp;
  long target_e0_count = current_e0_count + disp;

  /*
    int rot_speed = 250;
    if (distance < 0) {
      rot_speed *= -1;
    }
  */
  int rot_speed = 250;
  if (target_e1_count < e1_count) {
    rot_speed *= -1;
  }

  int disp_threshold = 5;

  left_PID.reset();
  right_PID.reset();

  while (abs(target_e1_count - e1_count) > disp_threshold && abs(target_e0_count - e0_count) > disp_threshold ) {
    unsigned long time_now = millis();
    if (time_now - pid_timestamp > (1000 / pid_freq)) {
      pid_timestamp = time_now;

      float demand_error = (e0_speed - e1_speed) / TICKS_PER_REV;
      float correction = demand_PID.update(0, demand_error);

      float output = left_PID.update(rot_speed * (1 + demand_error), e0_speed);
      simple_motor.leftMotor(output);
      output = right_PID.update(rot_speed * (1 - demand_error), e1_speed);
      simple_motor.rightMotor(output);

      //Serial.println();

      if (search_line) {
        line_confidence = lineSensors.updateConfidence(line_confidence);
        if (line_confidence > 0.3f) {
          Serial.println("Line found. Rotation stopped");
          simple_motor.rightMotor(0);
          simple_motor.leftMotor(0);
          return true;
        }
      }
    }
    if (millis() - kinematics_timestamp > 1 / (float)kinematics_freq) {
      kinematics_timestamp  = millis();
      kinematics.update();
    }
    delay(LOOP_DELAY);
  }


  simple_motor.rightMotor(0);
  simple_motor.leftMotor(0);

  return false;
}


//Looks at the global angle
//Referenced from global coordinates
//angle in rads
boolean faceAngle(float target_angle) {

  //Get current angle
  float cur_angle = kinematics.theta;
  float dif_angle = cur_angle - target_angle;
  //cap angle
  if (dif_angle > TWO_PI) {
    dif_angle -= TWO_PI;
  } else   if (dif_angle < 0) {
    dif_angle += TWO_PI;
  }

  //more than pi, reverse
  if ( dif_angle > PI) {
    dif_angle -= TWO_PI;
  }

  rotateRobot(dif_angle, false);
  kinematics.update();

}

//target coordinates in mm!
boolean moveToPlace(long target_x, long target_y) {


  Serial.print("Moving to coordinates ");
  Serial.print("x ");
  Serial.print(target_x * MM_TO_TICKS);
  Serial.print(", y ");
  Serial.println(target_y * MM_TO_TICKS);

  long cur_x = kinematics.x_pos ; //Pos in ticks, need to convert
  long cur_y = kinematics.y_pos ;


  Serial.print("cur x ");
  Serial.println(cur_x);
  Serial.print("cur y ");
  Serial.println(cur_y);

  long delta_x = (cur_x * TICKS_TO_MM) - target_x;
  long delta_y = (cur_y * TICKS_TO_MM) - target_y;

  float distance_to_target = sqrt((float)delta_x * delta_x  + delta_y * delta_y);
  float angle_to_target = atan2(-delta_y, -delta_x);


  Serial.print("Distance ");
  Serial.println(distance_to_target);
  Serial.print("Angle ");
  Serial.println(angle_to_target);

  //Turn to target
  faceAngle(angle_to_target);

  //Move to target
  moveRobot(distance_to_target, false);

}


void printEncoders() {
  Serial.print("Left encoder: ");
  Serial.print(e0_count);
  Serial.print(" and right encoder: ");
  Serial.println(e1_count);
}
