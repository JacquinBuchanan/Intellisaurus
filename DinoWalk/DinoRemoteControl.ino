#include "DinoRemoteControl.h"

boolean RemoteControlCodeAvailable()
{
  if (IRLremote.available())
    return true;

  if( Serial.available())
    return true;
  return false;
}

int  RemoteControlCodeRead()
{

  if (IRLremote.available())
  {

    // Get the new data from the IR remote
    auto data = IRLremote.read();

    switch(data.command)
    {
      case 0x19: return BUTTON_0;
      case 0x45: return BUTTON_1;
      case 0x46: return BUTTON_2;
      case 0x47: return BUTTON_3;
      case 0x44: return BUTTON_4;
      case 0x40: return BUTTON_5;
      case 0x43: return BUTTON_6;
      case 0x7: return BUTTON_7;
      case 0x15: return BUTTON_8;
      case 0x9: return BUTTON_9;
      case 0xd: return BUTTON_HASH;
      case 0x16: return BUTTON_STAR;
      case 0x18: return BUTTON_UP;
      case 0x52: return BUTTON_DOWN;
      case 0x1c: return BUTTON_CENTER;
      case 0x5a: return BUTTON_RIGHT;
      case 0x8: return BUTTON_LEFT;
    }

  }

  // Be sure the same address is used in Raspberry Pi code.
  #define RASPBERRY_PI_I2C_ADDRESS 0x13

  // Look to see if Raspberry Pi has any commands
  Wire.requestFrom(RASPBERRY_PI_I2C_ADDRESS, 6);    // request 1 byte from slave device RASPBERRY_PI_I2C_ADDRESS

  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print("FROM PI "); Serial.println(c);         // print the character
    
    switch(c)
    {
      case '0': return BUTTON_0;
      case '1': return BUTTON_1;
      case '2': return BUTTON_2;
      case '3': return BUTTON_3;
      case '4': return BUTTON_4;
      case '5': return BUTTON_5;
      case '6': return BUTTON_6;
      case '7': return BUTTON_7;
      case '8': return BUTTON_8;
      case '9': return BUTTON_9;
      case 'p': return BUTTON_HASH;
      case 'o': return BUTTON_STAR;
      case 'w': return BUTTON_UP;
      case 'x': return BUTTON_DOWN;
      case 's': return BUTTON_CENTER;
      case 'd': return BUTTON_RIGHT;
      case 'a': return BUTTON_LEFT;
    }
    
  }

  // this is the serial debug port in Arduino IDE
  if ( Serial.available())
  {
    int inByte = Serial.read();

    switch(inByte)
    {
      case '0': return BUTTON_0;
      case '1': return BUTTON_1;
      case '2': return BUTTON_2;
      case '3': return BUTTON_3;
      case '4': return BUTTON_4;
      case '5': return BUTTON_5;
      case '6': return BUTTON_6;
      case '7': return BUTTON_7;
      case '8': return BUTTON_8;
      case '9': return BUTTON_9;
      case 'p': return BUTTON_HASH;
      case 'o': return BUTTON_STAR;
      case 'w': return BUTTON_UP;
      case 'x': return BUTTON_DOWN;
      case 's': return BUTTON_CENTER;
      case 'd': return BUTTON_RIGHT;
      case 'a': return BUTTON_LEFT;
    }
  }
  return 0;
}
