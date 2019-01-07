//reading data from hitechnic compass:http://www.hitechnic.com/cgi-bin/commerce.cgi?preadd=action&key=NMC1034
int compass() {

  int compass;
  int last;
  compass = compass_angle();

  if (compass_angle() > 180) {
    compass = 360 - compass_angle();
    compass = compass * -1;
  }
     return compass*-1 ;
}

int16_t _north = 0;


uint16_t compass_raw()
{
    uint16_t a, b;
    Wire.beginTransmission(0x02 >> 1);
    Wire.write(0x44);
    Wire.endTransmission();
    Wire.requestFrom(0x02 >> 1, 2);
    if (Wire.available() >= 2) {
        a = Wire.read();
        b = Wire.read();
        while (Wire.available()) Wire.read();
    }

    return a + b * 256;
}

void compass_set_north()
{

    _north = compass_raw();
}

uint16_t compass_angle()
{
    uint16_t relative_angle = (compass_raw() - _north + 360) % 360;

    if (relative_angle < 0)
        relative_angle += 360;

    return relative_angle;
}

uint16_t compass_real_north()
{
    return _north;
}
