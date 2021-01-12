#ifndef _KINEMATICS_H
#define _KINEMATICS_H


#define ROMI_DIAMETER 0.1425f //Experimentally found
#define WHEEL_DIAMETER 0.07f
#define SENSOR_DIST 26    //roughly, in mm
#define TICKS_TO_MM 0.152f


class kinematics_c {

  private:
    long _prev_e0_count = 0;
    long _prev_e1_count = 0;
    long sensor_pos[2] = {0, 0};
    float x_delta = 0;
    float y_delta = 0;

  public:
    long x_pos = 0;   //In ticks
    long y_pos = 0;   //In ticks
    float theta = 0;   //IN RADS, UNFORTUNATELY

    // Function Prototypes
    kinematics_c();
    void setPose(long _x, long _y, float _theta );
    void setPose(long _x, long _y);
    void update();
    void print();
    long * getSensorPos();
    long * sensorToRobot(long sensor_x, long sensor_y);
    long * getSensorPos(long _x, long _y, float _angle);

};


kinematics_c::kinematics_c() {

  _prev_e0_count = e0_count;
  _prev_e1_count = e1_count;

}

void kinematics_c::setPose( long _x, long _y, float _theta ) {
  x_pos     = _x;
  y_pos    = _y;
  theta = _theta;
}

void kinematics_c::setPose( long _x, long _y) {
  x_pos     = _x;
  y_pos    = _y;
}



// Routine to execute the update to kinematics
void kinematics_c::update() {

  long e0_delta = e0_count - _prev_e0_count;
  long e1_delta = e1_count - _prev_e1_count;
  _prev_e0_count = e0_count;
  _prev_e1_count = e1_count;

  //Serial.print(e0_delta);
  //Serial.print("\n");

  // Find angle

  //Turn counts into mm with our experimental constant, divide by diameter in mm
  theta += ((float)e0_delta - e1_delta) * TICKS_TO_MM / ((float) ROMI_DIAMETER * 1000);

  // Cap angle...

  if (theta > TWO_PI) {
    theta -= TWO_PI;
  } else if (theta < 0) {
    theta += TWO_PI;
  }

  // Forward movement
  float d = (e0_delta + e1_delta) / 2.0; // for now

  // Change can be very small! Using floats
  x_delta += cos(theta) * d;
  y_delta += sin(theta) * d;

  //Add the integer part
  x_pos += (int) x_delta;
  y_pos += (int) y_delta;

  //Remove integer from deltas
  x_delta -= (int) x_delta;
  y_delta -= (int) y_delta;

  //print();

}


void kinematics_c::print() {
  Serial.print(x_pos);
  Serial.print(",");
  Serial.print(y_pos);
  Serial.print(",");
  Serial.print(theta);
  Serial.print("\n");
}

long* kinematics_c::getSensorPos() {
  long sensor_x = x_pos + cos(theta) * SENSOR_DIST / 0.152f;
  long sensor_y = y_pos + sin(theta) * SENSOR_DIST / 0.152f;
  sensor_pos[0] = sensor_x;
  sensor_pos[1] = sensor_y;
  /*
    Serial.print("Sensor at x ");
    Serial.print(sensor_pos[0]);
    Serial.print(" y ");
    Serial.println(sensor_pos[1]);
  */
  return sensor_pos;
}

//Used to get the position of the robot based on a sensor centered at a node
//Simply flip angle
long* kinematics_c::sensorToRobot(long sensor_x, long sensor_y) {
  long robot_x = sensor_x + cos(theta + PI) * SENSOR_DIST / 0.152f;
  long robot_y = sensor_y + sin(theta + PI) * SENSOR_DIST / 0.152f;
  sensor_pos[0] = robot_x;
  sensor_pos[1] = robot_y;

  return sensor_pos;
}


long* kinematics_c::getSensorPos(long _x, long _y, float _angle) {
  long sensor_x = _x + cos(_angle) * SENSOR_DIST / 0.152f;
  long sensor_y = _y + sin(_angle) * SENSOR_DIST / 0.152f;
  sensor_pos[0] = sensor_x;
  sensor_pos[1] = sensor_y;
  /*
    Serial.print("Sensor at x ");
    Serial.print(sensor_pos[0]);
    Serial.print(" y ");
    Serial.println(sensor_pos[1]);
  */
  return sensor_pos;
}






#endif
