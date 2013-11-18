#ifndef _AQ_SerialLCD_H_
#define _AQ_SerialLCD_H_

// OSD notification system on SparkFun serial enabled LCD (2x16)
//
// void notifyOSD(byte flags, char *fmt, ...)
//   - display notification string on OSD
//
// void notifyOSDmenu(byte flags, byte cursorLeft, byte cursorRight, char *fmt, ...)
//   - display notification with blinking region = 'cursor'
//   - characters between cursorLeft and cursorRight will blink if OSD_CURSOR flag is used
//
//   fmt == NULL will clear
//   flags -- message priority and options i.e. (OSD_CRIT|OSD_BLINK|OSD_CENTER)

#define OSD_INFO    0x00
#define OSD_WARN    0x40
#define OSD_ERR     0x80
#define OSD_CRIT    0xc0
#define OSD_NOCLEAR 0x20 // do not clear the message after ~5s
#define OSD_CURSOR  0x10 // enable cursor
#define OSD_BLINK   0x08 // blinking message
#define OSD_INVERT  0x04 // inverted message
#define OSD_NOW     0x02 // show message immediately (do not wait until OSD update)
#define OSD_CENTER  0x01 // Justify at center

#define notifyOSD(flags,fmt,args...) notifyOSDmenu(flags,255,255,fmt, ## args)

#include <stdio.h>
#include <stdarg.h>

void hideOSD()   // dummy functions to avoid modifying menu code
{
}
void unhideOSD()
{
}

void InitSerialLCD() {
  SERIAL_LCD.begin(9600);
  SERIAL_LCD.write(0xFE);
  SERIAL_LCD.write(0x01); // Clear
}

byte notifyOSDmenu(byte flags, byte cursorLeft, byte cursorRight, const char *fmt, ...) {

  va_list ap;

  char buf[32];

  SERIAL_LCD.write(0xfe);
  SERIAL_LCD.write(0x01); // clear

  if (fmt == NULL) {
    SERIAL_LCD.write(0xFE);
    SERIAL_LCD.write(0x0C); // disable cursor
  }
  else {
    va_start(ap, fmt);
    byte len=vsnprintf(buf, 29, fmt, ap);
    va_end(ap);
    if (len > 28) {
      len = 28;
    }
    memset(buf+len, ' ', 28-len);
    if (flags & OSD_CENTER) {
      byte i = (28 - len) / 2;
      if (i) {
        // move text right to center it
        memmove(buf + i, buf, strlen(buf));
        memset(buf, ' ', i);
        // adjust cursor position also if needed
        if (flags & OSD_CURSOR) {
          cursorLeft  += i;
          cursorRight += i;
        }
      }
    }

    for (byte i=0; i<28; i++) {
      SERIAL_LCD.write(buf[i]);
    }

    if (flags & OSD_CURSOR) {
      if (cursorLeft > 27) cursorLeft = 27;
      if (cursorLeft > 15) cursorLeft += 48; // adjust for 2 line display

      SERIAL_LCD.write(0xFE);
      SERIAL_LCD.write(0x80 + cursorLeft); // set cursor position

      SERIAL_LCD.write(0xFE);
      SERIAL_LCD.write(0x0D); // enable block cursor
    }
    else {
      SERIAL_LCD.write(0xFE);
      SERIAL_LCD.write(0x0C); // disable cursor
    }
  }

  return 0;
}

#endif
