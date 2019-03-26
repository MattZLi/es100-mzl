/* **********************************************************************
   Code for interrogating, configuring, and then reading measured data
   from multiple TI FDC1004 Cap to Digital Converter Chip
   FDC1004 uses I2C communications with hardcoded I2C address of 0x50 Hex

*/

//#include <stdio.h>
#include <Wire.h>   // Include the I2C communications "Wire" library (Arduino IDE supplied)
#include <QueueArray.h> // Include library for queues

#include <bluefruit.h> // Bluetooth by Adafruit

BLEDis bledis;
BLEHidAdafruit blehid;

bool hasKeyPressed = false;

#define INT_MAX 10000;

//*****************************************************************************************
// Initialize CONSTANTS and declare some global Variables
//
uint8_t I2C_ADDR     = 0x50; // FDC1004 Hardcoded I2C addr
uint8_t Pointer;             // Pointer to Select which Register within FDC1004 to access
uint8_t NumSens      = 0;    // Number of FDC1004 on I2C Mux

// FDC Register Addresses (Select using Pointer)
//
uint8_t FDC_DEV_ID   = 0xFE; // Pointer to Device_ID Register        Read Only = 0x5449
uint8_t FDC_MAN_ID   = 0xFF; // Pointer to Manufacturer_ID Register  Read Only = 0x1004

uint8_t FDC_CONFIG   = 0x0C; // Pointer to Overall Measurement Configuration Register
uint8_t FDC_CONM1    = 0x08; // Pointer to set Measurement 1's Input wiring Configuration
uint8_t FDC_CONM2    = 0x09; // Pointer to set Measurement 2's Input wiring Configuration
uint8_t FDC_CONM3    = 0x0A; // Pointer to set Measurement 3's Input wiring Configuration
uint8_t FDC_CONM4    = 0x0B; // Pointer to set Measurement 4's Input wiring Configuration

uint8_t FDC_RD_M[]   = {1, 0, 3, 2, 5, 4, 7, 6}; // Addr's to read meas. value from (MSB 1st)

bool    toggle       = true;
float   gain         = (15.0) / (8388608.0);  // Nominal Gain in units of (pf / count)

// define length of queue
int queue_length = 5;

QueueArray <float> mag_queue;

// raw capacitance values
float capA = (float) 0.0;
float capB = (float) 0.0;
float capC = (float) 0.0;
float capD = (float) 0.0;

// magnitude and threshold
float mag = (float) 0.0;
float mag_threshold = 3.5;

float filt_mag = (float) 0.0;
float abs_mag = (float) 0.0;

// min and max, used to normalize caps
float minA = (float) INT_MAX;
float maxA = (float) 0.0;
float minB = (float) INT_MAX;
float maxB = (float) 0.0;
float minC = (float) INT_MAX;
float maxC = (float) 0.0;
float minD = (float) INT_MAX;
float maxD = (float) 0.0;

// timer to track when to refresh min/max
unsigned long last_t = 0;

// buffer for previous min/mix
float temp_minA = (float) INT_MAX;
float temp_maxA = (float) 0.0;
float temp_minB = (float) INT_MAX;
float temp_maxB = (float) 0.0;
float temp_minC = (float) INT_MAX;
float temp_maxC = (float) 0.0;
float temp_minD = (float) INT_MAX;
float temp_maxD = (float) 0.0;

// normalized
float normA = (float) 0.0;
float normB = (float) 0.0;
float normC = (float) 0.0;
float normD = (float) 0.0;

// vertical position tracking
float normAC = (float) 0.0;
float normBD = (float) 0.0;

float pos_vert = (float) 0.0;
float prev_pos_vert = (float) 0.0;

float mag_vert = (float) 0.0;

float deriv_sum_vert = (float) 0.0;
float integral_vert = (float) 0.0;


// horizontal position tracking
float normAB = (float) 0.0;
float normCD = (float) 0.0;

float pos_horz = (float) 0.0;
float prev_pos_horz = (float) 0.0;

float mag_horz = (float) 0.0;

float deriv_sum_horz = (float) 0.0;
float integral_horz = (float) 0.0;

// thresholds for directions
float down_thresh = -0.4;
float up_thresh = 0.4;
float left_thresh = -0.4;
float right_thresh = 0.4; 

// ****************************************************************************************
// Initialize FDC1004 device
//
void setup() {
  Wire.begin();              // join i2c bus (address optional for master)
  Serial.begin(115200);      // start serial communication at 115200 bps

  // Set up I2C for the FDC1004
  write16(FDC_CONFIG, 0x8000);  // Issue Reset Command to chip

  // Setup the MEASx Registers for single ended measurements, CINx to MEASx
  //
  write16(FDC_CONM1, 0x1C00);
  write16(FDC_CONM2, 0x3C00);
  write16(FDC_CONM3, 0x5C00);
  write16(FDC_CONM4, 0x7C00);

  // Setup FDC_Config Register for
  //     400 Samples/sec Rate,  (2.5mSec / sample)
  //     Repeating,
  //     all 4 MEASx's enabled.
  //
  write16(FDC_CONFIG, 0x0DF0);      //D = 400Hz, 5 = 100Hz
  
  // enqueue each queue full of 0s
  for (int i = 0; i < queue_length; i++){
    mag_queue.enqueue(0);
  }

  // delay(10000);

  // Set up BLE
  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Bluefruit52");

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather 52");
  bledis.begin();

  /* Start BLE HID
   * Note: Apple requires BLE device must have min connection interval >= 20m
   * ( The smaller the connection interval the faster we could send data).
   * However for HID and MIDI device, Apple could accept min connection interval 
   * up to 11.25 ms. Therefore BLEHidAdafruit::begin() will try to set the min and max
   * connection interval to 11.25  ms and 15 ms respectively for best performance.
   */
  blehid.begin();

  // Set callback for set LED from central
  blehid.setKeyboardLedCallback(set_keyboard_led);

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEHidAdafruit::begin() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms 
   */
  /* Bluefruit.setConnInterval(9, 12); */

  // Set up and start advertising
  startAdv();
}

// helper function for BLE
void startAdv(void)
{  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  
  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for the dev name in the advertising packet
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

// *********************************************************************
// Main loop, measure from all 4 channels forever
//
void loop() {
  uint16_t value = 0;             // var to hold measurement
  write16(FDC_CONFIG, 0x0DF0);    // trigger all 4 measurements
  delay(3 * 4);                   // wait for all 4 measurements to complete
  read_meas(toggle);              // read the 4 measurements
  toggle = !toggle;               // for some reasons every other trigger is bogus
}

// **************************************************************
// Functions

// ************** Read in all 4 measurements **********************
void read_meas(bool toggle) {
  uint32_t val = 0;    // 32 bit binary measurement (only 24 bits used)
  uint32_t msb = 0;    // temp storage of measurement's 16 bit MSByte Resigister
  uint32_t lsb = 0;    // temp storage of measurement's 16 bit LSByte Resigister
  float    cap = 0;    // floating point value of measured capacitance (in pf)

  int j = 0;
  for (int i = 0; i <= 3; i++) {
    lsb = read16(FDC_RD_M[j++]);     // MEASj LSB
    msb = read16(FDC_RD_M[j++]);     // MEASj MSB
    val = ((msb << 16) + lsb) >> 8;  // 24 bit combined MSB and LSB
    cap = (float)val * gain;         // convert to pf
    if (toggle) {

      char strcap [10];
      // add measurements to buffer for each MEASx
      switch (i) {
        case 0:
          blehid.keySequence("cap0 ", 10);
          
          itoa((int) cap, strcap, 10);
          blehid.keySequence(strcap, 10);
          blehid.keyPress(' ');
          break;
        case 1:
          blehid.keySequence("cap1 ", 10);
          
          itoa((int) cap, strcap, 10);
          blehid.keySequence(strcap, 10);
          blehid.keyPress(' ');
          break;
        case 2:
          blehid.keySequence("cap2 ", 10);
          
          itoa((int) cap, strcap, 10);
          blehid.keySequence(strcap, 10);
          blehid.keyPress(' ');
          break;
        case 3:
          blehid.keySequence("cap3 ", 10);
          
          itoa((int) cap, strcap, 10);
          blehid.keySequence(strcap, 10);
          blehid.keyPress(' ');
          break;
        default:
          break;
      }

    }
  }
}

// **********  Read/Write 16 bit value from an FDC1004 register *******************

unsigned int read16(uint8_t Pointer) {
  uint16_t value;                       // declare var to hold read reg value
  Wire.beginTransmission(I2C_ADDR);     // Que up I2C 7 bit addr and R/W* bit = R
  Wire.write(Pointer);                  // Que up Pointer addr
  Wire.endTransmission();               // Send the Addresses

  Wire.requestFrom(I2C_ADDR, 2);        // Que Request for 2 bytes from I2C addr slave
  value = Wire.read();                  // Read in MSByte byte
  value <<= 8;                          // shift up to MSByte in 16 bit value
  value |= Wire.read();                 // Read in LSByte, OR into 16 bit value
  return value;
}

void write16(uint8_t Pointer, uint16_t data) {
  Wire.beginTransmission(I2C_ADDR);    // Que up I2C 7 bit addr and R/W* bit = W*
  Wire.write(Pointer);                 // Que up Pointer addr
  Wire.write((uint8_t) (data >> 8));   // Que up MSbyte of 16 bit reg value
  Wire.write((uint8_t) data);          // Que up LSbyte of 16 bit reg value
  Wire.endTransmission();              // Send Queued bytes
}

void QueueCopy(QueueArray <int>* A, QueueArray <int>* B) {
  QueueArray <int> temp;

  while (!A->isEmpty()) {
    temp.enqueue(A->dequeue());
  }
  while (!temp.isEmpty()) {
    A->enqueue(temp.front());
    B->enqueue(temp.dequeue());
  }
}

bool QueueEqual(QueueArray <int>* A, QueueArray <int>* B) {
  QueueArray <int> tempA;
  QueueArray <int> tempB;

  QueueCopy(A, &tempA);
  QueueCopy(B, &tempB);
  
  while (!tempA.isEmpty() && !tempB.isEmpty()) {
    if (tempA.dequeue() != tempB.dequeue()) {
      return false;
    }
  }
  return true;
}

/**
 * Callback invoked when received Set LED from central.
 * Must be set previously with setKeyboardLedCallback()
 *
 * The LED bit map is as follows: (also defined by KEYBOARD_LED_* )
 *    Kana (4) | Compose (3) | ScrollLock (2) | CapsLock (1) | Numlock (0)
 */
void set_keyboard_led(uint8_t led_bitmap)
{
  // light up Red Led if any bits is set
  if ( led_bitmap )
  {
    ledOn( LED_RED );
  }
  else
  {
    ledOff( LED_RED );
  }
}
