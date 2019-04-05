/* **********************************************************************
   Code for interrogating, configuring, and then reading measured data
   from multiple TI FDC1004 Cap to Digital Converter Chip
   FDC1004 uses I2C communications with hardcoded I2C address of 0x50 Hex

*/

//#include <stdio.h>
#include <Wire.h>       // Include the I2C communications "Wire" library (Arduino IDE supplied)
#include <QueueArray.h> // Include library for queues
#include <bluefruit.h>  // Bluetooth by Adafruit
#define FLOAT_MAX 10000.

#define BLE_ON true

BLEDis bledis;
BLEHidAdafruit blehid;

bool hasKeyPressed = false;

//*****************************************************************************************
// Initialize CONSTANTS and declare some global Variables
//
uint8_t I2C_ADDR = 0x50; // FDC1004 Hardcoded I2C addr
uint8_t Pointer;         // Pointer to Select which Register within FDC1004 to access
uint8_t NumSens = 0;     // Number of FDC1004 on I2C Mux

// FDC Register Addresses (Select using Pointer)
//
uint8_t FDC_DEV_ID = 0xFE; // Pointer to Device_ID Register        Read Only = 0x5449
uint8_t FDC_MAN_ID = 0xFF; // Pointer to Manufacturer_ID Register  Read Only = 0x1004

uint8_t FDC_CONFIG = 0x0C; // Pointer to Overall Measurement Configuration Register
uint8_t FDC_CONM1 = 0x08;  // Pointer to set Measurement 1's Input wiring Configuration
uint8_t FDC_CONM2 = 0x09;  // Pointer to set Measurement 2's Input wiring Configuration
uint8_t FDC_CONM3 = 0x0A;  // Pointer to set Measurement 3's Input wiring Configuration
uint8_t FDC_CONM4 = 0x0B;  // Pointer to set Measurement 4's Input wiring Configuration

bool toggle = true;
float gain = (15.0) / (8388608.0); // Nominal Gain in units of (pf / count)

// define length of queue
int queue_length = 5;

QueueArray<float> mag_queue;

// raw capacitance values
float capacitances[] = {0., 0., 0., 0.};

// magnitude and threshold
float mag = (float)0.0;
float mag_threshold = 3.5;

float filt_mag = (float)0.0;
float abs_mag = (float)0.0;

// min and max, used to normalize caps
float mins[] = {FLOAT_MAX, FLOAT_MAX, FLOAT_MAX, FLOAT_MAX};
float maxes[] = {0., 0., 0., 0.};

// timer to track when to refresh min/max
unsigned long last_t = 0;
unsigned long last_print = 0;

// buffer for previous min/max
float temp_mins[] = {FLOAT_MAX, FLOAT_MAX, FLOAT_MAX, FLOAT_MAX};
float temp_maxes[] = {0., 0., 0., 0.};

// normalized
float norms[] = {0., 0., 0., 0.};

// vertical position tracking
float normAC = 0.0;
float normBD = 0.0;

float pos_vert = 0.0;
float prev_pos_vert = 0.0;

float mag_vert = 0.0;

float deriv_sum_vert = 0.0;
float integral_vert = 0.0;

// horizontal position tracking
float normAB = 0.0;
float normCD = 0.0;

float pos_horz = 0.0;
float prev_pos_horz = 0.0;

float mag_horz = 0.0;

float deriv_sum_horz = 0.0;
float integral_horz = 0.0;

// thresholds for directions
float down_thresh = -0.7;
float up_thresh = 0.7;
float left_thresh = -0.7;
float right_thresh = 0.7;

// float down_thresh = -0.4;
// float up_thresh = 0.4;
// float left_thresh = -0.4;
// float right_thresh = 0.4;

// ****************************************************************************************
// Initialize FDC1004 device
//
void setup()
{
  
  Serial.begin(115200); // start serial communication at 115200 bps
  Wire.begin();         // join i2c bus (address optional for master)

  if (BLE_ON)
  { // Set up BLE
    Bluefruit.begin();
    // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
    Bluefruit.setTxPower(4);
    Bluefruit.setName("Bluefruit52");

    // Configure and Start Device Information Service
    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather 52");
    bledis.begin();
    blehid.begin();

    // Set callback for set LED from central
    blehid.setKeyboardLedCallback(set_keyboard_led);

    // Set up and start advertising
    startAdv();
  }

  delay(500);

  write16(FDC_CONFIG, 0x8000); // Issue Reset Command to chip
  delay(50);
  // Setup the MEASx Registers for single ended measurements, CINx to MEASx
  //
  write16(FDC_CONM1, 0x1C00);
  delay(50);
  write16(FDC_CONM2, 0x3C00);
  delay(50);
  write16(FDC_CONM3, 0x5C00);
  delay(50);
  write16(FDC_CONM4, 0x7C00);
  delay(50);

  // Setup FDC_Config Register for
  //     400 Samples/sec Rate,  (2.5mSec / sample)
  //     Repeating,
  //     all 4 MEASx's enabled.
  //
  write16(FDC_CONFIG, 0x0DF0); //D = 400Hz, 5 = 100Hz

  // enqueue each queue full of 0s
  for (int i = 0; i < queue_length; i++)
  {
    mag_queue.enqueue(0);
  }

  
  delay(3000);
}

// *********************************************************************
// Main loop, measure from all 4 channels forever
//
void loop()
{
  write16(FDC_CONFIG, 0x0DF0); // trigger all 4 measurements
  write16(FDC_CONFIG, 0x0DF0); // trigger all 4 measurements
  delay(3 * 4);                // wait for all 4 measurements to complete
  // check if the measurements are ready
  uint32_t done;
  do
  {
    uint32_t state = read16(FDC_CONFIG);
    done = (state & 0xF);
  } while (done != 0xF);
  read_meas();
}

// **************************************************************
// Functions

// ************** Read in all 4 measurements **********************
void read_meas()
{
  uint32_t val = 0; // 32 bit binary measurement (only 24 bits used)
  uint32_t msb = 0; // temp storage of measurement's 16 bit MSByte Resigister
  uint32_t lsb = 0; // temp storage of measurement's 16 bit LSByte Resigister
  float cap = 0;    // floating point value of measured capacitance (in pf)

  int j = 0;
  for (int i = 0; i <= 3; i++)
  {
    msb = read16(j++);              // MEASj MSB
    lsb = read16(j++);              // MEASj LSB
    val = ((msb << 16) + lsb) >> 8; // 24 bit combined MSB and LSB
    cap = (float)val * gain;        // convert to pf

    // add measurements to buffer for each MEASx
    capacitances[i] = cap;

    mins[i] = min(mins[i], capacitances[i]);
    maxes[i] = max(maxes[i], capacitances[i]);
    temp_mins[i] = min(temp_mins[i], capacitances[i]);
    temp_maxes[i] = max(temp_maxes[i], capacitances[i]);

    if (maxes[i] <= mins[i])
    {
      norms[i] = 0;
    }
    else
    {
      norms[i] = (capacitances[i] - mins[i]) / (maxes[i] - mins[i]);
    }
  }
  // unsigned long current_t = millis();
  // if ((current_t - last_print) > 250)
  // {
    // graphCaps();
  // }
  // graphNorms();

  // calculate 2-dimensional position
  // A (-1, 1); B (-1, -1); C (1, 1); D(1, -1)
  pos_horz = (-norms[sensor('a')] - norms[sensor('b')] + norms[sensor('c')] + norms[sensor('d')]) / 2.0;
  pos_vert = (norms[sensor('a')] - norms[sensor('b')] + norms[sensor('c')] - norms[sensor('d')]) / 2.0;

  // low-pass filter absolute magnitude of vertical sensors
  abs_mag = capacitances[sensor('a')] + capacitances[sensor('b')] + capacitances[sensor('c')] + capacitances[sensor('d')];

  filt_mag = filt_mag - mag_queue.dequeue() + abs_mag;
  mag_queue.enqueue(abs_mag);

  // normalized magnitude of vertical sensors
  normAC = max(norms[sensor('a')], norms[sensor('c')]);
  normBD = max(norms[sensor('b')], norms[sensor('d')]);
  mag_vert = (normAC + normBD) / 2.0;

  // normalized magnitude of horizontal sensors
  normAB = max(norms[sensor('a')], norms[sensor('b')]);
  normCD = max(norms[sensor('c')], norms[sensor('d')]);
  mag_horz = (normAB + normCD) / 2.0;

  // classify direction of gesture
  if ((filt_mag / (float)queue_length) > mag_threshold)
  {
    // vertical component
    deriv_sum_vert += mag_vert * (pos_vert - prev_pos_vert);
    integral_vert += deriv_sum_vert;
    // horizontal component
    deriv_sum_horz += mag_horz * (pos_horz - prev_pos_horz);
    integral_horz += deriv_sum_horz;
  }
  else
  {
    bool vertical = abs(integral_vert) > abs(integral_horz);
    if (vertical && integral_vert < down_thresh)
    {
      // swipe('D');
      swipe('n');
    }
    else if (vertical && integral_vert > up_thresh)
    {
      // swipe('U');
      swipe('n');
    }
    else if (!vertical && integral_horz < left_thresh)
    {
      // swipe('L');
      swipe('n');
    }
    else if (!vertical && integral_horz > right_thresh)
    {
      // swipe('R');
      swipe('n');
    }
    else if (integral_vert != 0 && integral_horz != 0)
    {
      // swipe('T');
      swipe('p');
    }
  }

  // refresh mins and maxes every 10 seconds
  unsigned long curr_t = millis();
  if ((curr_t - last_t) > 10000)
  {
    for (size_t i = 0; i < 4; i++)
    {
      mins[i] = (mins[i] + temp_mins[i]) / 2.0;
      maxes[i] = (maxes[i] + temp_maxes[i]) / 2.0;
      temp_mins[i] = FLOAT_MAX;
      temp_maxes[i] = 0.;
    }
    last_t = curr_t;
  }
  prev_pos_vert = pos_vert;
  prev_pos_horz = pos_horz;
}

// **********  Read/Write 16 bit value from an FDC1004 register *******************

unsigned int read16(uint8_t Pointer)
{
  uint16_t value;                   // declare var to hold read reg value
  Wire.beginTransmission(I2C_ADDR); // Que up I2C 7 bit addr and R/W* bit = R
  Wire.write(Pointer);              // Que up Pointer addr
  Wire.endTransmission();           // Send the Addresses

  Wire.requestFrom(I2C_ADDR, 2); // Que Request for 2 bytes from I2C addr slave
  value = Wire.read();           // Read in MSByte byte
  value <<= 8;                   // shift up to MSByte in 16 bit value
  value |= Wire.read();          // Read in LSByte, OR into 16 bit value
  return value;
}

void write16(uint8_t Pointer, uint16_t data)
{
  Wire.beginTransmission(I2C_ADDR); // Que up I2C 7 bit addr and R/W* bit = W*
  Wire.write(Pointer);              // Que up Pointer addr
  Wire.write((uint8_t)(data >> 8)); // Que up MSbyte of 16 bit reg value
  Wire.write((uint8_t)data);        // Que up LSbyte of 16 bit reg value
  Wire.endTransmission();           // Send Queued bytes
}

void QueueCopy(QueueArray<int> *A, QueueArray<int> *B)
{
  QueueArray<int> temp;

  while (!A->isEmpty())
  {
    temp.enqueue(A->dequeue());
  }
  while (!temp.isEmpty())
  {
    A->enqueue(temp.front());
    B->enqueue(temp.dequeue());
  }
}

bool QueueEqual(QueueArray<int> *A, QueueArray<int> *B)
{
  QueueArray<int> tempA;
  QueueArray<int> tempB;

  QueueCopy(A, &tempA);
  QueueCopy(B, &tempB);

  while (!tempA.isEmpty() && !tempB.isEmpty())
  {
    if (tempA.dequeue() != tempB.dequeue())
    {
      return false;
    }
  }
  return true;
}

// int sensor(char c)
// {
//   switch (c)
//   {
//   case 'a':
//     return 1;
//   case 'b':
//     return 2;
//   case 'c':
//     return 3;
//   case 'd':
//     return 0;
//   default:
//     return -1;
//   }
// }

int sensor(char c)
{
  switch (c)
  {
  case 'a':
    return 0;
  case 'b':
    return 1;
  case 'c':
    return 2;
  case 'd':
    return 3;
  default:
    return -1;
  }
}

void swipe(char dir)
{
<<<<<<< HEAD
  if (BLE_ON) {
    blehid.keyPress(dir);
    delay(100);
    // blehid.keyPress(' ');
    // char buffer[15];
    // float_to_str(buffer, 15, integral_horz);
    // blehid.keySequence(buffer, 50);
    // blehid.keyPress(' ');
    // float_to_str(buffer, 15, integral_vert);
    // blehid.keySequence(buffer, 50);
    // blehid.keyPress('\n');
    blehid.keyRelease();
    delay(100);
  }
  else {
    Serial.print({dir});
    Serial.print(" ");
    Serial.print(integral_horz);
    Serial.print(" ");
    Serial.print(integral_vert);
    Serial.print(" ");
    Serial.println();
  }


  
=======
  blehid.keyPress(dir);
  delay(300);
  // blehid.keyPress(' ');
  // char buffer[15];
  // float_to_str(buffer, 15, integral_horz);
  // blehid.keySequence(buffer, 50);
  // blehid.keyPress(' ');
  // float_to_str(buffer, 15, integral_vert);
  // blehid.keySequence(buffer, 50);
  // blehid.keyPress('\n');

  blehid.keyRelease();
  delay(300);

  // Serial.print({dir});
  // Serial.print(" ");
  // Serial.print(integral_horz);
  // Serial.print(" ");
  // Serial.print(integral_vert);
  // Serial.print(" ");
  // Serial.println();
>>>>>>> 53e7889a4e1b98458f81d7b0fbc455cc1ac3ae23
  deriv_sum_vert = 0.;
  integral_vert = 0.;
  deriv_sum_horz = 0.;
  integral_horz = 0.;
}

void graphCaps()
{
  if (BLE_ON)
  {
    for (size_t i = 0; i < 1; i++)
    {
      char strcap[10];
      float_to_str(strcap, 10, norms[i]);
      blehid.keySequence(strcap, 50);
      blehid.keyPress(' ');
      blehid.keyRelease();
    }
  }
  else
  {
    for (size_t i = 0; i < 4; i++)
    {
      Serial.print(norms[i]);
      Serial.print(" ");
    }
    // Serial.print(pos_horz);
    // Serial.print(" ");
    // Serial.print(pos_vert);
    Serial.println();
  }
}

void graphNorms()
{
  if (!BLE_ON)
  {
    for (size_t i = 0; i < 4; i++)
    {
      Serial.print(norms[i]);
      Serial.print(" ");
    }
    Serial.print(pos_horz);
    Serial.print(" ");
    Serial.print(pos_vert);
    Serial.print(" ");
    // Serial.print(mag_horz);
    // Serial.print(" ");
    // Serial.print(mag_vert);
    // Serial.print(" ");
    // Serial.print(deriv_sum_horz);
    // Serial.print(" ");
    // Serial.print(deriv_sum_vert);
    // Serial.print(" ");
    // Serial.print(integral_horz);
    // Serial.print(" ");
    // Serial.print(integral_vert);
    // Serial.print(" ");

    Serial.println();
  }
}

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
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
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
  if (led_bitmap)
  {
    ledOn(LED_RED);
  }
  else
  {
    ledOff(LED_RED);
  }
}

void float_to_str(char *out, int buffsize, float val)
{
  itoa(val, out, 10);
  int i = 0;
  while (out[i])
  {
    i++;
  }
  if (i < buffsize - 5)
  {
    out[i] = '.';
    out[i + 1] = '0' + ((int)(abs(val) * 10) % 10);
    out[i + 2] = '0' + ((int)(abs(val) * 100) % 10);
    out[i + 3] = '\0';
  }
}