#ifndef _BEEPER_H
#define _BEEPER_H

#define BUZZER_PIN 25

class beeper_c
{
  private:
    int _volume = 150;
    long _last_timestamp = 0;

  public:

    beeper_c() {
      pinMode( BUZZER_PIN, OUTPUT);
    }

    void soundOn() {
      analogWrite(BUZZER_PIN, _volume);
    }

    void soundOff() {
      analogWrite(BUZZER_PIN, 0);
    }

    void tick() {
      analogWrite(BUZZER_PIN, _volume);
      delay(150);
      analogWrite(BUZZER_PIN, 0);
    }

    void beep() {
      for (int i = 0; i < 30; i++) {
        analogWrite(BUZZER_PIN, 130);
        delay(1);
        analogWrite(BUZZER_PIN, 0);
        delay(1);
      }
    }

    void smallBeep() {
      for (int i = 0; i < 20; i++) {
        analogWrite(BUZZER_PIN, 130);
        delay(1);
        analogWrite(BUZZER_PIN, 0);
        delay(1);
      }
    }


    //UNTESTED. DONT USE
    void other_beep() {

      int buzzer_period = 10000;
      _last_timestamp = micros();
      unsigned long elapsed_time = 0;

      while (elapsed_time < 1000000) {
        unsigned long time_now = micros();
        elapsed_time = time_now - _last_timestamp;

        if ( elapsed_time > buzzer_period ) {
          //buzzer_on(150);
        }

        else if ( elapsed_time > buzzer_period * 2 ) {
          //buzzer_on(0);
          _last_timestamp = micros();
          buzzer_period = buzzer_period * 0.999;

        }
      }
    }
};

#endif
