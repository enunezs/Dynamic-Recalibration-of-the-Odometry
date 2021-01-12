#ifndef _LINESENSORSSYSTEM_H
#define _LINESENSORSSYSTEM_H

/*
  #define LINE_LEFT_PIN A2
  #define LINE_CENTRE_PIN A3
  #define LINE_RIGHT_PIN A4
*/
class lineSensorsSystem_c {

  public:
    lineSensor_c line_sensor_left;
    lineSensor_c line_sensor_center;
    lineSensor_c line_sensor_right;

    int _black_threshold = 300;
    int _shiny_threshold = -140;

    lineSensorsSystem_c(int pin_left, int pin_center, int pin_right) {

      //Update sensors
      line_sensor_left.updatePin(pin_left);
      line_sensor_center.updatePin(pin_center);
      line_sensor_right.updatePin(pin_right);

      line_sensor_left._black_threshold = _black_threshold;
      line_sensor_center._black_threshold = _black_threshold;
      line_sensor_right._black_threshold = _black_threshold;

      line_sensor_left._shiny_threshold = _shiny_threshold;
      line_sensor_center._shiny_threshold = _shiny_threshold;
      line_sensor_right._shiny_threshold = _shiny_threshold;

    }

    void calibrateLineSensor() {

      //pinMode( BUZZER_PIN, OUTPUT );
      //beeper.tick();

      //Calibrate line sensor
      line_sensor_left.calibrate();
      //beeper.tick();
      line_sensor_center.calibrate();
      //beeper.tick();
      line_sensor_right.calibrate();
      //beeper.tick();
      //current_state = e_drive_forward;
    }

    void updateSensors() {
      line_sensor_right.readValue();
      line_sensor_center.readValue();
      line_sensor_left.readValue();
    }

    void printSensors() {
      line_sensor_left._prev_value;
      Serial.print(line_sensor_left._prev_value );
      Serial.print( "," );
      Serial.print(line_sensor_center._prev_value);
      Serial.print( "," );
      Serial.print(line_sensor_right._prev_value);
      Serial.print( "," );
      Serial.print(_black_threshold);
      Serial.print( "," );
      Serial.print(_shiny_threshold);
      Serial.print( "\n" );
    }

    float updateConfidence(float confidence_value) {

      //type casting from bool to int :)
      float current_confidence = (line_sensor_center.onLine() + line_sensor_right.onLine() + line_sensor_left.onLine()) / 3.0f;
      //Weighted average
      confidence_value = confidence_value * 0.9 + current_confidence * 0.1;

      //cap final value
      if (confidence_value > 1) {
        confidence_value = 1;
      } else if (confidence_value < 0) {
        confidence_value = 0;
      }
      //This one looks so COOL
      /*
            Serial.print( current_confidence );
            Serial.print( "," );
            Serial.print( confidence_value );
            Serial.print( "\n" );
      */
      return confidence_value;
    }

    float updateNodeConfidence(float confidence_value) {

      //type casting from bool to int :)
      float current_confidence = (line_sensor_center.onNode() + line_sensor_right.onNode() + line_sensor_left.onNode()) / 3.0f;
      //Weighted average
      confidence_value = confidence_value * 0.8 + current_confidence * 0.2;

      //cap final value
      if (confidence_value > 1) {
        confidence_value = 1;
      } else if (confidence_value < 0) {
        confidence_value = 0;
      }

      return confidence_value;
    }


    float weighted_sensing() {

      int l_value = constrain(line_sensor_left._prev_value, 0, 1000 );
      int c_value = constrain(line_sensor_center._prev_value, 0, 1000 );
      int r_value = constrain(line_sensor_right._prev_value, 0, 1000 );

      float total_value = l_value + c_value + r_value;

      if (total_value == 0) {
        return 0 ;
      }

      float l_proportional = l_value / total_value;
      float r_proportional = r_value / total_value;
      /*
        Serial.print( l_value);
        Serial.print( ", " );
        Serial.print( c_value);
        Serial.print( ", " );
        Serial.print( r_value);
        Serial.print( ", " );
      */

      return -l_proportional + r_proportional;
    }


    boolean onLine() {
      return line_sensor_center.onLine() || line_sensor_right.onLine() || line_sensor_left.onLine();
    }
    boolean onMode() {
      return line_sensor_center.onNode() || line_sensor_right.onNode() || line_sensor_left.onNode();
    }

};


#endif
