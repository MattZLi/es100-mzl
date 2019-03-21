/* **********************************************************************
   Code for interrogating, configuring, and then reading measured data
   from multiple TI FDC1004 Cap to Digital Converter Chip
   FDC1004 uses I2C communications with hardcoded I2C address of 0x50 Hex

*/

//#include <stdio.h>
#include <Wire.h>   // Include the I2C communications "Wire" library (Arduino IDE supplied)
#include <QueueArray.h> // Include library for queues
#include <Keyboard.h>
#include <Mouse.h>

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
float mag_threshold = (float) 17.0;

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

// ****************************************************************************************
// Initialize FDC1004 device
//
void setup() {
  Wire.begin();              // join i2c bus (address optional for master)
  Serial.begin(115200);      // start serial communication at 115200 bps
  
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
      //      Serial.print(cap);
      //      Serial.print(" ");    // needed to delienate between next series
      // add measurements to buffer for each MEASx
      switch (i) {
        case 0:
          // record capacitance of sensor A
          capA = cap;
          // dynamically sets min and max
          minA = min(minA, capA);
          maxA = max(maxA, capA);
          // records min and max value over refresh period
          temp_minA = min(temp_minA, capA);
          temp_maxA = max(temp_maxA, capA);
          // normalize cap value between 0 and 1
          if (maxA == minA) {
            normA = 0;
            // Serial.print("maxA == minA");
          } else {
            normA = (capA - minA)/(maxA - minA);
          }
          
          break;
        case 1:
          capB = cap;

          minB = min(minB, capB);
          maxB = max(maxB, capB);
          temp_minB = min(temp_minB, capB);
          temp_maxB = max(temp_maxB, capB);

          if (maxB == minB) {
            // Serial.print("maxB == minB");
            normB = 0;
          } else {
            normB = (capB - minB)/(maxB - minB);
          }
          
          
          break;
        case 2:
          capC = cap;

          minC = min(minC, capC);
          maxC = max(maxC, capC);
          temp_minC = min(temp_minC, capC);
          temp_maxC = max(temp_maxC, capC);

          if (maxC == minC) {
            // Serial.print("maxC == minC");
            normC = 0;
          } else {
            normC = (capC - minC)/(maxC - minC);
          }

          break;
        case 3:
          capD = cap;

          minD = min(minD, capD);
          maxD = max(maxD, capD);
          temp_minD = min(temp_minD, capD);
          temp_maxD = max(temp_maxD, capD);

          if (maxD == minD) {
            // Serial.print("maxD == minD");
            normD = 0;
          } else {
            normD = (capD - minD)/(maxD - minD);
          }

          break;
        default:
          break;
      }

      // calculate 2-dimensional position
      // A (-1, 1); B (-1, -1); C (1, 1); D(1, -1)
      pos_horz = (normC + normD - normA - normB)/2.0;
      pos_vert = (normA - normB + normC - normD)/2.0;

      // low-pass filter absolute magnitude of vertical sensors
      abs_mag = capA + capB + capC + capD;
      // Serial.print("abs_mag");
      // Serial.print(" ");
      // Serial.print(abs_mag);
      // Serial.print(" ");

      mag = (normA + normB + normC + normD)/4.0;

      filt_mag = filt_mag - mag_queue.dequeue() + abs_mag;
      mag_queue.enqueue(abs_mag);
      // Serial.print("filt_mag");
      // Serial.print(" ");
      Serial.print(filt_mag/(float) queue_length);
      Serial.print(" ");

      // normalized magnitude of vertical sensors
      // mag_vert = (normAC + normBD)/2.0;
      // Serial.print("mag_vert");
      // Serial.print(" ");
      // Serial.print(mag_vert - 2.0); // shift the graph down
      // Serial.print(" ");

      // normalized magnitude of horizontal sensors
      // mag_horz = (normAB + normCD)/2.0;
      // Serial.print("mag_horz");
      // Serial.print(" ");
      // Serial.print(mag_horz - 4.0); // shift the graph down
      // Serial.print(" ");

      // classify direction of gesture
      if ((filt_mag/(float) queue_length) > mag_threshold) {
        // vertical component
        deriv_sum_vert += mag*(pos_vert - prev_pos_vert);
        integral_vert += deriv_sum_vert;
        // horizontal component
        deriv_sum_horz += mag*(pos_horz - prev_pos_horz);
        integral_horz += deriv_sum_horz;
      } else {
        if (abs(integral_vert) > abs(integral_horz)) { // vertical
          if (integral_vert < -0.3) {
            Keyboard.write('D');
            // Keyboard.println(integral_vert);
            deriv_sum_vert = (float) 0;
            integral_vert = (float) 0;
            deriv_sum_horz = (float) 0;
            integral_horz = (float) 0;
          } else if (integral_vert > 0.3) {
            Keyboard.write('U');
            // Keyboard.println(integral_vert);
            deriv_sum_vert = (float) 0;
            integral_vert = (float) 0;
            deriv_sum_horz = (float) 0;
            integral_horz = (float) 0;
          } else if (integral_vert != 0) {
            Keyboard.write('T');
            // Keyboard.println(integral_vert);
            deriv_sum_vert = (float) 0;
            integral_vert = (float) 0;
          }
        } else if (abs(integral_vert) < abs(integral_horz)) { // horizontal
          if (integral_horz < -0.3) {
            Keyboard.write('L');
            // Keyboard.println(integral_horz);
            deriv_sum_horz = (float) 0;
            integral_horz = (float) 0;

          } else if (integral_horz > 0.3) {
            Keyboard.write('R');
            // Keyboard.println(integral_horz);
            deriv_sum_horz = (float) 0;
            integral_horz = (float) 0;

          } else if (integral_horz != 0) {
            Keyboard.write('T');
            // Keyboard.println(integral_horz);
            deriv_sum_horz = (float) 0;
            integral_horz = (float) 0;
          }
        }
      }

      // Serial.print("deriv");
      // Serial.print(" ");
      // Serial.print(deriv_sum_vert);
      // // Serial.print((float) deriv_sum_vert);
      // Serial.print(" ");

      // Serial.print("integ");
      // Serial.print(" ");
      // Serial.print(integral_vert);
      // Serial.print((float) integral_vert);
      // Serial.print(" ");

      // Serial.print("deriv");
      // Serial.print(" ");
      // Serial.print(deriv_sum_horz);
      // Serial.print((float) deriv_sum_horz);
      // Serial.print(" ");

      // Serial.print("integ");
      // Serial.print(" ");
      // Serial.print(integral_horz);
      // Serial.print((float) integral_horz);
      // Serial.print(" ");

      // refresh mins and maxes every 10 seconds
      unsigned long curr_t = millis();
      if ((curr_t - last_t) > 10000) {
        last_t = curr_t;
        minA = (minA + temp_minA)/2.0;
        maxA = (maxA + temp_maxA)/2.0;
        minB = (minB + temp_minB)/2.0;
        maxB = (maxB + temp_maxB)/2.0;
        minC = (minC + temp_minC)/2.0;
        maxC = (maxC + temp_maxC)/2.0;
        minD = (minD + temp_minD)/2.0;
        maxD = (maxD + temp_maxD)/2.0;
        
        temp_minA = (float) INT_MAX;
        temp_maxA = (float) 0.0;
        temp_minB = (float) INT_MAX;
        temp_maxB = (float) 0.0;
        temp_minC = (float) INT_MAX;
        temp_maxC = (float) 0.0;
        temp_minD = (float) INT_MAX;
        temp_maxD = (float) 0.0;
      }

      prev_pos_vert = pos_vert;
      prev_pos_horz = pos_horz;

      Serial.println();
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
