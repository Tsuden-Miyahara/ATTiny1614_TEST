
#include <Arduino.h>

//#include <SoftwareSerial.h>
#include "CH9329.h"

/*
Keyboard::Keyboard(Serial *ser) {
  _ser = ser;
}

int Keyboard::SendPacket(char *buf, char blen) {
  _ser->write(buf,blen);
  while (_ser->available() > 0) {
    _ser->read();
  }
  return 1;
}

void Keyboard::Press(byte ckey, byte ukey) {
  byte sum = 0x10C + ckey + ukey;
  byte keyPress[14] = {
    0x57,0xAB,0x00,0x02,0x08,
    ckey,
    0x00,
    ukey,0x00,0x00,0x00,0x00,0x00,
    sum
  };
  byte keyRelease[14] = {0x57,0xAB,0x00,0x02,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C};
  _ser->write(keyPress, 14);
  //while(_ser->available() > 0) {_ser->read();}
  delay(5);
  _ser->write(keyRelease, 14);
  //while(_ser->available() > 0) {_ser->read();}
  delay(5);
}
*/