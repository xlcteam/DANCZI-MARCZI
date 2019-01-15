uint8_t line_sensors[LINE_SENSORS_COUNT] =
{  LINE_FRONT_PIN, LINE_BACK_PIN, LINE_LEFT_PIN_1, LINE_LEFT_PIN_2, LINE_RIGHT_PIN_1, LINE_RIGHT_PIN_2 };
/* all interrupt pins are on the same port, so we can use any one of them
   to determine PCMSK, PCICR and PCIE
*/

void setup_line_sensors()
{
  for (uint8_t i = 0; i < LINE_SENSORS_COUNT; i++) {
    uint8_t pin = line_sensors[i];
    pinMode(pin, INPUT);
    *digitalPinToPCMSK(LINE_FRONT_PIN) |= _BV(digitalPinToPCMSKbit(pin));
  }
  pinMode(LINE_THRESH_PIN, OUTPUT);

  line_sensors_update();
}

inline uint8_t read_line_sensor(uint8_t index)
{
  return digitalRead(line_sensors[index]);
}

void line_sensors_update()
{
  analogWrite(LINE_THRESH_PIN, light_pwm);

  *digitalPinToPCICR(LINE_FRONT_PIN) &=
    ~_BV(digitalPinToPCICRbit(LINE_FRONT_PIN)); // clear
  if (line_use_int) {
    *digitalPinToPCICR(LINE_FRONT_PIN) |=
      _BV(digitalPinToPCICRbit(LINE_FRONT_PIN)); // set
  }
}

/* decide where to move after reaching the white line (processes ws)
   dirs:
   0 - 7: multiplied by 45 gives angle where to move
   255: no line
*/
uint8_t line_last_dir;
uint8_t process_ws()
{
  uint8_t dir;

  MUTEX(1);

  if (!ws[0] && !ws[1] && !ws[2] && !ws[3] && !ws[4] && !ws[5]) {
    dir = 255;
  } else {
    if (moving_tipe == 1) {
      dir = 100;
    }
    else {
      //moving_tipe == 2
      if (ws[0] && !ws[1] && !ws[2] && !ws[3] && !ws[4] && !ws[5]) {
        dir = 0;
      }

      else if (ws[0] && !ws[1] && !ws[2] && !ws[3] && ws[4] && !ws[5]) {
        dir = 1;
      }
      else if (ws[0] && !ws[1] && !ws[2] && !ws[3] && ws[4] && ws[5]) {
        dir = 1;
      }
      else if (ws[0] && !ws[1] && !ws[2] && !ws[3] && !ws[4] && ws[5]) {
        dir = 1;
      }

      else if (!ws[0] && !ws[1] && !ws[2] && !ws[3] && !ws[4] && ws[5]) {
        dir = 2;
      }
      else if (!ws[0] && !ws[1] && !ws[2] && !ws[3] && ws[4] && ws[5]) {
        dir = 2;
      }
      else if (!ws[0] && !ws[1] && !ws[2] && !ws[3] && ws[4] && !ws[5]) {
        dir = 2;
      }

      else if (!ws[0] && ws[1] && !ws[2] && !ws[3] && ws[4] && !ws[5]) {
        dir = 3;
      }
      else if (!ws[0] && ws[1] && !ws[2] && !ws[3] && ws[4] && ws[5]) {
        dir = 3;
      }
      else if (!ws[0] && ws[1] && !ws[2] && !ws[3] && !ws[4] && ws[5]) {
        dir = 3;
      }

      else if (!ws[0] && ws[1] && !ws[2] && !ws[3] && !ws[4] && !ws[5]) {
        dir = 4;
      }

      else if (!ws[0] && ws[1] && ws[2] && !ws[3] && !ws[4] && !ws[5]) {
        dir = 5;
      }
      else if (!ws[0] && ws[1] && ws[2] && ws[3] && !ws[4] && !ws[5]) {
        dir = 5;
      }
      else if (!ws[0] && ws[1] && !ws[2] && ws[3] && !ws[4] && !ws[5]) {
        dir = 5;
      }

      else if (!ws[0] && !ws[1] && !ws[2] && ws[3] && !ws[4] && !ws[5]) {
        dir = 6;
      }
      else if (!ws[0] && !ws[1] && ws[2] && ws[3] && !ws[4] && !ws[5]) {
        dir = 6;
      }
      else if (!ws[0] && !ws[1] && ws[2] && !ws[3] && !ws[4] && !ws[5]) {
        dir = 6;
      }

      else if (ws[0] && !ws[1] && ws[2] && !ws[3] && !ws[4] && !ws[5]) {
        dir = 7;
      }
      else if (ws[0] && !ws[1] && ws[2] && ws[3] && !ws[4] && !ws[5]) {
        dir = 7;
      }
      else if (ws[0] && !ws[1] && !ws[2] && ws[3] && !ws[4] && !ws[5]) {
        dir = 7;
      }
    }

  }


  MUTEX(0);

  line_last_dir = dir;

  return dir;
}

uint8_t line_sensors_dir()
{
  uint32_t time = micros();

  // get data to process
  if (!line_use_int) {
    for (uint8_t i = 0; i < LINE_SENSORS_COUNT; i++) {
      ws_tmp[i] = read_line_sensor(i) * time;
    }
  }

  MUTEX(1);
  for (uint8_t i = 0; i < LINE_SENSORS_COUNT; i++) {
    ws[i] = ws_tmp[i];
  }
  MUTEX(0);

  bool print_dbg = (ws[0] || ws[1] || ws[2] || ws[3] || ws[4] || ws[5]);

  if (print_dbg) {
    Serial.println("line");
    Serial.print(ws[0]);
    Serial.print(" "); Serial.print(ws[1]);
    Serial.print(" "); Serial.print(ws[2]);
    Serial.print(" "); Serial.print(ws[3]);
    Serial.print(" "); Serial.print(ws[4]);
    Serial.print(" "); Serial.println(ws[5]);
  }

  uint16_t dir = process_ws();

  // fill ws_tmp with actual values
  if (line_use_int) {
    MUTEX(1);
    for (uint8_t i = 0; i < LINE_SENSORS_COUNT; i++) {
      ws_tmp[i] = read_line_sensor(i) * micros();
    }
    MUTEX(0);
  }

  if (print_dbg) {
    Serial.print("dir "); Serial.println(dir);
  }

  return dir;
}
