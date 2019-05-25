#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(10, 11); // RX, TX

/*
 * Matrix 64
 */
 
byte outputs64[] = {40, 41, 42, 43, 44, 45, 46, 47};
const int outputs64Count = 8;
 
byte inputs64[] = {32, 33, 34, 35, 36, 37, 38, 39};
const int inputs64Count = 8;

byte keys64State[outputs64Count][inputs64Count];
const char* keys64Value[outputs64Count][inputs64Count];

/*
 * Matrix 16
 */
 
byte outputs16[] = {A12, A13, A14, A15};
const int outputs16Count = 4;

byte inputs16[] = {A5, A9, A10, A11};
const int inputs16Count = 4;

byte keys16State[outputs16Count][inputs16Count];
const char* keys16Value[outputs16Count][inputs16Count];


void setup() {

  // R, G, U (blue), Y, W
  // 0123456789ABCDEF

  /*
   * Matrix 64
   */
   
  // red
  keys64Value[0][0] = "+r0.";
  keys64Value[0][1] = "+r1.";
  keys64Value[0][2] = "+r2.";
  keys64Value[0][3] = "+r3.";
  keys64Value[1][0] = "+r4.";
  keys64Value[1][1] = "+r5.";
  keys64Value[1][2] = "+r6.";
  keys64Value[1][3] = "+r7.";
  keys64Value[2][0] = "+r8.";
  keys64Value[2][1] = "+r9.";
  keys64Value[2][2] = "+rA.";
  keys64Value[2][3] = "+rB.";
  keys64Value[3][0] = "+rC.";
  keys64Value[3][1] = "+rD.";
  keys64Value[3][2] = "+rE.";
  keys64Value[3][3] = "+rF.";

  // yellow
  keys64Value[4][0] = "+y0.";
  keys64Value[4][1] = "+y1.";
  keys64Value[4][2] = "+y2.";
  keys64Value[4][3] = "+y3.";
  keys64Value[5][0] = "+y4.";
  keys64Value[5][1] = "+y5.";
  keys64Value[5][2] = "+y6.";
  keys64Value[5][3] = "+y7.";
  keys64Value[6][0] = "+y8.";
  keys64Value[6][1] = "+y9.";
  keys64Value[6][2] = "+yA.";
  keys64Value[6][3] = "+yB.";
  keys64Value[7][0] = "+yC.";
  keys64Value[7][1] = "+yD.";
  keys64Value[7][2] = "+yE.";
  keys64Value[7][3] = "+yF.";

  // white
  keys64Value[0][4] = "+wF.";
  keys64Value[0][5] = "+wE.";
  keys64Value[0][6] = "+wD.";
  keys64Value[0][7] = "+wC.";
  keys64Value[1][4] = "+wB.";
  keys64Value[1][5] = "+wA.";
  keys64Value[1][6] = "+w9.";
  keys64Value[1][7] = "+w8.";
  keys64Value[2][4] = "+w7.";
  keys64Value[2][5] = "+w6.";
  keys64Value[2][6] = "+w5.";
  keys64Value[2][7] = "+w4.";
  keys64Value[3][4] = "+w3.";
  keys64Value[3][5] = "+w2.";
  keys64Value[3][6] = "+w1.";
  keys64Value[3][7] = "+w0.";

  // blue (u)
  keys64Value[4][4] = "+uC.";
  keys64Value[4][5] = "+u8.";
  keys64Value[4][6] = "+u4.";
  keys64Value[4][7] = "+u0.";
  keys64Value[5][4] = "+uD.";
  keys64Value[5][5] = "+u9.";
  keys64Value[5][6] = "+u5.";
  keys64Value[5][7] = "+u1.";
  keys64Value[6][4] = "+uE.";
  keys64Value[6][5] = "+uA.";
  keys64Value[6][6] = "+u6.";
  keys64Value[6][7] = "+u2.";
  keys64Value[7][4] = "+uF.";
  keys64Value[7][5] = "+uB.";
  keys64Value[7][6] = "+u7.";
  keys64Value[7][7] = "+u3.";

  for (int i = 0; i < outputs64Count; i++) {
    pinMode(outputs64[i], OUTPUT);
    digitalWrite(outputs64[i], HIGH);
  }
  
  for (int i = 0; i < inputs64Count; i++) {
    pinMode(inputs64[i], INPUT_PULLUP);
    digitalWrite(inputs64[i], HIGH);
  }

  for (int i = 0; i < outputs64Count; i++) {
    for (int j = 0; j < inputs64Count; j++) {
      keys64State[i][j] = 1;
    }
  }

  /* 
   * Matrix 16 
   */
  
  // green (matrix 16)
  keys16Value[0][0] = "+g3.";
  keys16Value[0][1] = "+g7.";
  keys16Value[0][2] = "+gB.";
  keys16Value[0][3] = "+gF.";
  keys16Value[1][0] = "+g2.";
  keys16Value[1][1] = "+g6.";
  keys16Value[1][2] = "+gA.";
  keys16Value[1][3] = "+gE.";
  keys16Value[2][0] = "+g1.";
  keys16Value[2][1] = "+g5.";
  keys16Value[2][2] = "+g9.";
  keys16Value[2][3] = "+gD.";
  keys16Value[3][0] = "+g0.";
  keys16Value[3][1] = "+g4.";
  keys16Value[3][2] = "+g8.";
  keys16Value[3][3] = "+gC.";

  for (int i = 0; i < outputs16Count; i++) {
    pinMode(outputs16[i], OUTPUT);
    digitalWrite(outputs16[i], HIGH);
  }
  
  for (int i = 0; i < inputs16Count; i++) {
    pinMode(inputs16[i], INPUT_PULLUP);
    digitalWrite(inputs16[i], HIGH);
  }

  for (int i = 0; i < outputs16Count; i++) {
    for (int j = 0; j < inputs16Count; j++) {
      keys16State[i][j] = 1;
    }
  }
  
  // start serial connection
  bluetoothSerial.begin(9600);
}

void readPrintMatrix64() {
  for (int i = 0; i < outputs64Count; i++) {
    digitalWrite(outputs64[i], LOW);
    for (int j = 0; j < inputs64Count; j++) {
      int value = digitalRead(inputs64[j]);
      if (value != keys64State[i][j]) {
        bluetoothSerial.write(keys64Value[i][j]);
      }
      keys64State[i][j] = value;
    }
    digitalWrite(outputs64[i], HIGH);
  }
}

void readPrintMatrix16() {
  for (int i = 0; i < outputs16Count; i++) {
    digitalWrite(outputs16[i], LOW);
    for (int j = 0; j < inputs16Count; j++) {
      int value = digitalRead(inputs16[j]);
      if (value != keys16State[i][j]) {
        bluetoothSerial.write(keys16Value[i][j]);
      }
      keys16State[i][j] = value;
    }
    digitalWrite(outputs16[i], HIGH);
  }
}

void loop() {
  readPrintMatrix64();
  readPrintMatrix16();
}
