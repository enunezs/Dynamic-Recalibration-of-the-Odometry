#ifndef _MOTOR_H
#define _MOTOR_H

// A class to neatly contain commands for the
// motors, to take care of +/- values, a min/max
// power value, & pin setup.

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define R_DIR_FWD  LOW
#define R_DIR_BCD  HIGH
#define L_DIR_FWD  LOW
#define L_DIR_BCD  HIGH

class motor_c {

  private:
    //For now left 0 right 1
    byte _power = 0;

  public:
    //SimpleMotor(int side);
    //void leftMotor(int motor_power);
    //void rightMotor(int motor_power);

    motor_c()
    {
      pinMode( L_PWM_PIN, OUTPUT );
      pinMode( L_DIR_PIN, OUTPUT );
      pinMode( R_PWM_PIN, OUTPUT );
      pinMode( R_DIR_PIN, OUTPUT );

      _power = 0;
    }

    void leftMotor(int motor_power)
    {
      //Check limits
      int left_motor_power_max = 255;
      if (motor_power > left_motor_power_max ) {
        motor_power = left_motor_power_max ;
      } else if (motor_power < -left_motor_power_max ) {
        motor_power = -left_motor_power_max ;
      }

      //TODO: Preprocess speed
      if (motor_power > 0) {
        //Forward
        digitalWrite( L_DIR_PIN, L_DIR_FWD  );
      } else if (motor_power < 0) {
        //Backward
        digitalWrite( L_DIR_PIN, L_DIR_BCD  );
        motor_power = -motor_power;
      }

      //Check direction
      analogWrite( L_PWM_PIN, motor_power);
    }

    void rightMotor(int motor_power) {

      //Check limits
      int right_motor_power_max = 255;
      if (motor_power > right_motor_power_max ) {
        motor_power = right_motor_power_max ;
      } else if (motor_power < -right_motor_power_max ) {
        motor_power = -right_motor_power_max ;
      }

      //TODO: Preprocess speed
      if (motor_power > 0) {
        //Forward
        digitalWrite( R_DIR_PIN, R_DIR_FWD  );
      } else if (motor_power < 0) {
        //Backward
        digitalWrite( R_DIR_PIN, R_DIR_BCD  );
        motor_power = -motor_power;
      }

      //Check direction
      analogWrite( R_PWM_PIN, motor_power);
    }


};

// Needs prototype
//motor_c::motor_c(  ) {
  // ...
//}



#endif
