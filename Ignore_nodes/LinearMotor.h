#ifndef _LINEARMOTOR_h
#define _LINEARMOTOR_h


#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define R_DIR_FWD  LOW
#define R_DIR_BCD  HIGH
#define L_DIR_FWD  LOW
#define L_DIR_BCD  HIGH

class linearMotor_c
{
  private:
    //For now left 0 right 1
    byte _power = 0;
  public:
    //LinearMotor(int side);
    //void leftMotor(int motor_speed);
    //void rightMotor(int motor_speed);

    linearMotor_c()
    {
      pinMode( L_PWM_PIN, OUTPUT );
      pinMode( L_DIR_PIN, OUTPUT );
      pinMode( R_PWM_PIN, OUTPUT );
      pinMode( R_DIR_PIN, OUTPUT );

      _power = 0;
    }

    void leftMotor(int motor_speed)
    {
      //Check limits
      int left_motor_speed_max = 100;
      if (motor_speed > left_motor_speed_max ) {
        motor_speed = left_motor_speed_max ;
      } else if (motor_speed < -left_motor_speed_max ) {
        motor_speed = -left_motor_speed_max ;
      }

      int power = 0;

      //TODO: Preprocess speed
      if (motor_speed > 0) {
        //Forward
        power = 2.183 * motor_speed + 6.99;
        //power = 2.0 * motor_speed + 10.0;

        digitalWrite( L_DIR_PIN, L_DIR_FWD  );
      } else if (motor_speed < 0) {

        //Backward
        power = (-2.217 * motor_speed + 8.67);
        digitalWrite( L_DIR_PIN, L_DIR_BCD  );

      }

      //Check direction
      analogWrite( L_PWM_PIN, power);
    }




    void rightMotor(float motor_speed) {

      //Check limits
      int right_motor_speed_max = 100;
      if (motor_speed > right_motor_speed_max ) {
        motor_speed = right_motor_speed_max ;
      } else if (motor_speed < -right_motor_speed_max ) {
        motor_speed = -right_motor_speed_max ;
      }

      int power = 0;
      //TODO: Preprocess speed
      if (motor_speed > 0) {
        //Forward
        power = 2.0 * motor_speed + 10.0;
        //power = 2.183 * motor_speed + 6.99;

        digitalWrite( R_DIR_PIN, R_DIR_FWD  );

      } else if (motor_speed < 0) {
        //Backward
        power = (-2.141 * motor_speed + 8.394);
        digitalWrite( R_DIR_PIN, R_DIR_BCD  );
      }

      //Check direction
      analogWrite( R_PWM_PIN, power);
    }

};

#endif
