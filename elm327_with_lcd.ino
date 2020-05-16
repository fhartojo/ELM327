/* Arduino terminal for PL2303 USB to serial converter and XBee radio. */
/* Inserts linefeed after carriage return in data sent to and received from Xbee */
/* USB support */
#include <avrpins.h>
#include <max3421e.h>
#include <usbhost.h>
#include <usb_ch9.h>
#include <Usb.h>
#include <usbhub.h>
#include <avr/pgmspace.h>
#include <address.h>
/* CDC support */
#include <cdcacm.h>
#include <cdcprolific.h>
/* Debug support */
#include <printhex.h>
#include <message.h>
#include <hexdump.h>
#include <parsetools.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

// These #defines make it easy to set the backlight color
#define OFF 0x0
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7 

#define BUFSIZ 256

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

class PLAsyncOper: public CDCAsyncOper {
  public:
    virtual uint8_t OnInit(ACM *pacm);
};

uint8_t PLAsyncOper::OnInit(ACM *pacm) {
  uint8_t res;
  uint8_t buf;

  //Copied from the Linux driver which in turn was sniffed from Windows.
  //Without these this sketch wouldn't talk to the USB device.
  pacm->VendorRead(0x8484, 0, &buf);
  pacm->VendorWrite(0x0404, 0);
  pacm->VendorRead(0x8484, 0, &buf);
  pacm->VendorRead(0x8383, 0, &buf);
  pacm->VendorRead(0x8484, 0, &buf);
  pacm->VendorWrite(0x0404, 1);
  pacm->VendorRead(0x8484, 0, &buf);
  pacm->VendorRead(0x8383, 0, &buf);
  pacm->VendorWrite(0, 1);
  pacm->VendorWrite(1, 0);
  pacm->VendorWrite(2, 0x44);

  LINE_CODING lc;
  lc.dwDTERate = 38400;	
  lc.bCharFormat = 0;
  lc.bParityType = 0;
  lc.bDataBits = 8;	

  res = pacm->SetLineCoding(&lc);

  if (res) {
    ErrorMessage<uint8_t>(PSTR("SetLineCoding"), res);

    return res;
  }

  // Set DTR = 1
  res = pacm->SetControlLineState(1);

  if (res) {
    ErrorMessage<uint8_t>(PSTR("SetControlLineState"), res);

    return res;
  }

  res = pacm->VendorWrite(0, 0);

  if (res) {
      ErrorMessage<uint8_t>(PSTR("VendorWrite"), res);
  }

  return res;
}

USB Usb;
//USBHub Hub(&Usb);
PLAsyncOper AsyncOper;
PL2303 Pl(&Usb, &AsyncOper);

void setup() {
  char responseBuf[BUFSIZ + 1];
  uint16_t responseBufLength = BUFSIZ;
  uint8_t rcode;

  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(ON);

  Serial.begin(115200);
  Serial.println("Start");

  if (Usb.Init() == -1)
    Serial.println("OSCOKIRQ failed to assert");

  delay(200); 

  rcode = requestResponse("ATWS\r", responseBuf, &responseBufLength);

  if (rcode) {
    ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

    lcdDisplayResponse("ATWS:" + rcode);
  } else {
    if (responseBufLength > 0) { //more than zero bytes received
      lcdDisplayResponse(responseBuf);
    }
  }

  rcode = requestResponse("ATE0\r", responseBuf, &responseBufLength);

  if (rcode) {
    ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

    lcdDisplayResponse("ATE0:" + rcode);
  }

  rcode = requestResponse("ATSP0\r", responseBuf, &responseBufLength);

  if (rcode) {
    ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

    lcdDisplayResponse("ATSP0:" + rcode);
  }

  do {
    rcode = requestResponse("0100\r", responseBuf, &responseBufLength);
  
    if (rcode) {
      ErrorMessage<uint8_t>(PSTR("Ret"), rcode);

      lcdDisplayResponse("0100:" + rcode);
    } else {
      if (responseBufLength > 0) { //more than zero bytes received
        lcdDisplayResponse(responseBuf);
      }
    }

    delay(1000);
  } while (!isResponseOK("0100\r", responseBuf));
}

void loop() {
  int mph = -1;
  int rpm = -1;
  char responseBuf[BUFSIZ + 1];
  uint16_t responseBufLength = BUFSIZ;
  uint8_t rcode;
  byte hexBuf[64];

  rcode = requestResponse("010C\r", responseBuf, &responseBufLength);

  if (rcode) {
    ErrorMessage<uint8_t>(PSTR("Ret"), rcode);
  }

  if (responseBufLength > 0) {
    bufToHex(hexBuf, responseBuf);
    rpm = ((hexBuf[0] << 8) | hexBuf[1]) >> 2;
  }

  rcode = requestResponse("010D\r", responseBuf, &responseBufLength);

  if (rcode) {
    ErrorMessage<uint8_t>(PSTR("Ret"), rcode);
  }

  if (responseBufLength > 0) {
    bufToHex(hexBuf, responseBuf);
    mph = (hexBuf[0] * 5) >> 3;
    mph = hexBuf[0];
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(mph);
  lcd.print(" @ ");
  lcd.print(rpm);
  lcd.setCursor(0, 1);
}

void lcdDisplayResponse(const char *buf) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(buf);
}

uint8_t requestResponse(char *inBuf, char *outBuf, uint16_t *outBufLength) {
  do {
    Usb.Task();
  } while (Usb.getUsbTaskState() != USB_STATE_RUNNING);

  uint8_t res;
  boolean responseComplete = false;

  res = Pl.SndData(strlen(inBuf), (uint8_t *) inBuf);

  if (res) {
    ErrorMessage<uint8_t>(PSTR("SndData"), res);

    return res;
  }

  uint16_t i;
  uint16_t j = 0;
  uint16_t bufLength;
  char buf[65]; //bufLength + 1

  do {
    bufLength = 64;
    res = Pl.RcvData(&bufLength, (uint8_t *) buf);

    buf[bufLength] = 0;

    Serial.print("bufLength = ");
    Serial.print(bufLength);
    Serial.print("; buf:  ");
    Serial.println(buf);

    if (res && res != hrNAK) {
      ErrorMessage<uint8_t>(PSTR("Ret"), res);

      responseComplete = true;
    } else {
      for (i = 0; j < BUFSIZ && i < bufLength; i++) {
        if (buf[i] >= ' ') {
          outBuf[j] = buf[i];
  
          if ((char) outBuf[j] == '>') {
            outBuf[j] = 0;
            responseComplete = true;
  
            break;
          }

          j++;
        }
      }
    }
  } while (!responseComplete);

  *outBufLength = j;

  Serial.print("res = ");
  Serial.print(res);
  Serial.print("; outBufLength = ");
  Serial.print(*outBufLength);
  Serial.print("; outBuf:  ");
  Serial.println(outBuf);

  return res;
}

boolean isResponseOK(char *inBuf, char *outBuf) {
  if (
    inBuf[0] + 4 != outBuf[0]
    || inBuf[1] != outBuf[1]
    || inBuf[2] != outBuf[3]
    || inBuf[3] != outBuf[4]
  ) {
    return false;
  }

  return true;
}

void bufToHex(byte *outBuf, char *buf) {
  byte i = 0;
  char *inBuf = buf + 6;

  // start at 6 which is the first hex byte after header
  // ex: "41 0C 1A F8"
  // return buf: 0x1AF8

  while (*inBuf != 0) {
    outBuf[i++] = strtoul(inBuf, &inBuf, 16);
  }
}

