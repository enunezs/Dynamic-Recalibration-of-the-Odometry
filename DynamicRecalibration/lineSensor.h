#ifndef _LINESENSOR_H
#define _LINESENSOR_H

class lineSensor_c {

  public:

    int _pin;
    int _reference_val = 0;
    int _prev_value = 0;
    int _black_threshold = 300;
    int _shiny_threshold = -50;
    int _max_sensor = 400;


    lineSensor_c() {};

    updatePin( int which_pin ) {
      _pin = which_pin;
      pinMode( _pin, INPUT );
    };


    // to remove bias offset
    void calibrate() {
      long average = 0;
      int samples = 100;
      int sampling_period = 10;//ms

      for (int i = 0; i < samples ; i++) {
        average += readRaw();
        delay(sampling_period); //So roughly 2 seconds

      }
      _reference_val = average / samples;
      //Serial.print("Reference val");
      //Serial.print(_reference_val);
    }


    /////////////////////////TODO: Adapt to colors



    // Write a routine here to check if your
    // sensor is on a line (true or false).
    boolean onLine( float threshold ) {

      if (readValue() > threshold) {
        return true;
      } else if (readValue() < _shiny_threshold) {
        return true;
      } else {
        return false;
      }
    }


    boolean onLine( ) {

      if (readValue() > _black_threshold) {
        return true;
      } else if (readValue() < _shiny_threshold) {
        return true;
      } else {
        return false;
      }
    }


    boolean onNode( ) {

      if (readValue() < _shiny_threshold) {
        return true;
      } else {
        return false;
      }
    }


    //Get raw reading from sensor
    int readRaw() {
      return analogRead(_pin);
    }



    //Read sensor, return calibrated value and store for future reference
    int readValue() {
      int val = readRaw() - _reference_val;
      _prev_value = val;
      return val;
    }


};

#endif
