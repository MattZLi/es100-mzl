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

// create 4 queues of integers corresponding with MEASx
QueueArray <float> queueA;
QueueArray <float> queueB;
QueueArray <float> queue3;
QueueArray <float> queue4;

// define length of queue
//int queue_length = 10;
int queue_length = 5;

// create 4 floats to track the sum of each queue
float filteredA = 0;
float filteredB = 0;
float sum3 = 0;
float sum4 = 0;

// create 4 ints to store threshold of touch
// should be more like cap value * (float) queue_length
//float threshold1 = 70;
//float threshold2 = 60;
//float threshold3 = 45;
//float threshold4 = 45;
float threshold1 = 5 * (float) queue_length;
float threshold2 = 5 * (float) queue_length;
float threshold3 = 6 * (float) queue_length;
float threshold4 = 4.5 * (float) queue_length;

// min and max
float minA = (float) INT_MAX;
float maxA = (float) 0.0;
float minB = (float) INT_MAX;
float maxB = (float) 0.0;

// normalized
float normA = (float) 0.0;
float normB = (float) 0.0;

// create 4 bools to track whether sensor is touched or not
int touchA = 0;
int touchB = 0;
int touch3 = 0;
int touch4 = 0;

int prev_touchA;
int prev_touchB;

signed long riseA = 0;
signed long fallA = 0;
signed long riseB = 0;
signed long fallB = 0;

QueueArray <int> up_queue;
QueueArray <int> down_queue;
QueueArray <int> state_queue;

int state = 0;
int prev_state = 0;


int pos;

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
  for (int i = 0; i < queue_length; i++) {
    queueA.enqueue(0);
    queueB.enqueue(0);
    queue3.enqueue(0);
    queue4.enqueue(0);
  }

  for (int i = 0; i < 4; i++) {
    state_queue.enqueue(0);
  }
  Serial.print(state_queue.count());

  // up: 0, 1, 3, 2
  up_queue.enqueue(0);
  up_queue.enqueue(1);
  up_queue.enqueue(3);
  up_queue.enqueue(2);

  Serial.print(up_queue.count());

  // down: 0, 2, 3, 1
  down_queue.enqueue(0);
  down_queue.enqueue(2);
  down_queue.enqueue(3);
  down_queue.enqueue(1);

  Serial.print(down_queue.count());
  
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
          // add to array of cap values
          // if array is full, shift the window like a FIFO queue
          
          minA = min(minA, cap);
          maxA = max(maxA, cap);

          normA = (cap - minA)/(maxA - minA);
//          Serial.print("normA");
//          Serial.print(" ");
//          Serial.print(normA);
//          Serial.print(" ");
          
          break;
        case 1:

          minB = min(minB, cap);
          maxB = max(maxB, cap);

          normB = (cap - minB)/(maxB - minB);
//          Serial.print("normB");
//          Serial.print(" ");
//          Serial.print(normB);
//          Serial.print(" ");
          
          break;
        case 2:
//          Serial.print("sensor3");
//          Serial.print(" ");
//          Serial.print(cap);
//          Serial.print(" ");
          sum3 = sum3 - queue3.dequeue() + cap;
          queue3.enqueue(cap);
          if (sum3 > threshold3) {
            touch3 = 3;
          } else {
            touch3 = 0;
          }
//          Serial.print(touch3);
//          Serial.print(" ");
          break;
        case 3:
//          Serial.print("sensor3");
//          Serial.print(" ");
//          Serial.print(cap);
//          Serial.print(" ");
          sum4 = sum4 - queue4.dequeue() + cap;
          queue4.enqueue(cap);
          if (sum4 > threshold4) {
            touch4 = 4;
          } else {
            touch4 = 0;
          }
//          Serial.print(touch4);
//          Serial.print(" ");
          break;
        default:
          break;
      }

      switch(state) {
        case 0: // A = 0, B = 0
          if (touchA == 1 && prev_touchA == 0) { // rising edge on A
            state = 2;
//            Serial.print("state: ");
//            Serial.print(state);
            state_queue.enqueue(state);
            state_queue.dequeue();
          }
          if (touchB == 1 && prev_touchB == 0) { // rising edge on B
            state = 1;
//            Serial.print("state: ");
//            Serial.print(state);
            state_queue.enqueue(state);
            state_queue.dequeue();
          }
          break;
    
        case 1: // A = 0, B = 1
          if (touchA == 1 && prev_touchA == 0) { // rising edge on A
            state = 3;
//            Serial.print("state: ");
//            Serial.print(state);
            state_queue.enqueue(state);
            state_queue.dequeue();
          }
          if (touchB == 0 && prev_touchB == 1) { // falling edge on B
            state = 0;
//            Serial.print("state: ");
//            Serial.print(state);
            state_queue.enqueue(state);
            state_queue.dequeue();
          }
          break;
        
        case 2: // A = 1, B = 0
          if (touchA == 0 && prev_touchA == 1) { // falling edge on A
            state = 0;
//            Serial.print("state: ");
//            Serial.print(state);
            state_queue.enqueue(state);
            state_queue.dequeue();
          }
          if (touchB == 1 && prev_touchB == 0) { // rising edge on B
            state = 3;
//            Serial.print("state: ");
//            Serial.print(state);
            state_queue.enqueue(state);
            state_queue.dequeue();
          }
          break;
    
        case 3: // A = 1, B = 1
          if (touchA == 0 && prev_touchA == 1) { // falling edge on A
            state = 1;
//            Serial.print("state: ");
//            Serial.print(state);
            state_queue.enqueue(state);
            state_queue.dequeue();
          }
          if (touchB == 0 && prev_touchB == 1) { // falling edge on B
            state = 2;
//            Serial.print("state: ");
//            Serial.print(state);
            state_queue.enqueue(state);
            state_queue.dequeue();
          }
          break;
    
        default:
          break;
      }

      float pos_vert = (normA - normB + 1.0)/2.0;
      Serial.print("pos_vert");
      Serial.print(" ");
      Serial.print(pos_vert);
      Serial.print(" ");

      Serial.print("center");
      Serial.print(" ");
      Serial.print(0.5);
      Serial.print(" ");

//      Serial.print("riseA - riseB");
      int delta_t = abs(riseA - riseB);
//      Serial.print((riseA - riseB));
//      Serial.println();
//      if (((riseB - riseA) < 100) && ((riseB - riseA) > 0)) {
      if ((delta_t < 100) && (delta_t > 0)) {
//        Serial.print("TOUCH");
//        Mouse.click(MOUSE_LEFT);
        riseA = 0;
        riseB = 0;
        // clear buffer
        state_queue.enqueue(0);
        state_queue.dequeue();
      } else if (QueueEqual(&state_queue, &up_queue)) {
//         Serial.print("UP");
//         Keyboard.write(KEY_UP_ARROW);
         state_queue.enqueue(0);
         state_queue.dequeue();
      } else if (QueueEqual(&state_queue, &down_queue)) {
//         Serial.print("DOWN");
//         Keyboard.write(KEY_DOWN_ARROW);
         state_queue.enqueue(0);
         state_queue.dequeue();
      }

      // update prev_touch
      prev_touchA = touchA;
  
      // update prev_touch
      prev_touchB = touchB;
    }
  }

  Serial.println();
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




