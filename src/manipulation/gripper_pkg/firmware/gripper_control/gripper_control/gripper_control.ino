#include <Dynamixel2Arduino.h>

#define DXL_BUS Serial1
#define USB_SERIAL Serial

const uint8_t DXL_ID = 1;
const float OPEN_POS  = 170;
// const float CLOSE_POS = 0; # fully close
const float CLOSE_POS = 100; 


Dynamixel2Arduino dxl(DXL_BUS);

void setup() {
  USB_SERIAL.begin(57600);
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  dxl.begin(57600);
  dxl.setProtocol(DXL_ID, 2.0);
  USB_SERIAL.println("Pinging servo...");
  if (dxl.ping(DXL_ID)) {
    USB_SERIAL.println("Servo found.");
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.torqueOn(DXL_ID);
    USB_SERIAL.println("Type O to open, C to close.");
  } else {
    USB_SERIAL.println("Servo not found.");
  }
}

void loop() {
  if (USB_SERIAL.available() > 0) {
    String command = USB_SERIAL.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("O")) {
      dxl.setGoalPosition(DXL_ID, OPEN_POS, UNIT_DEGREE);
      USB_SERIAL.println("Moving (opening)");
      while (dxl.readControlTableItem(ControlTableItem::MOVING, DXL_ID)) {
        delay(20);
      }
      USB_SERIAL.println("DONE");
    } else if (command.equalsIgnoreCase("C")) {
      dxl.setGoalPosition(DXL_ID, CLOSE_POS, UNIT_DEGREE);
      USB_SERIAL.println("Moving (closing)");
      while (dxl.readControlTableItem(ControlTableItem::MOVING, DXL_ID)) {
        delay(20);
      }
      USB_SERIAL.println("DONE");
    } else {
      USB_SERIAL.println("INVALID");
    }
  }
}