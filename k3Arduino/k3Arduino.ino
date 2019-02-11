#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(10, 11); // RX, TX

/*
 * Matrix 64
 */
 
byte outputs64[] = {42, 46, 43, 47, 40, 44, 41, 45};
const int outputs64Count = 8;
 
byte inputs64[] = {34, 38, 35, 39, 32, 36, 33, 37};
const int inputs64Count = 8;

byte keys64State[outputs64Count][inputs64Count];
const char* keys64Value[outputs64Count][inputs64Count];

/*
 * Matrix 16
 */
 
byte outputs16[] = {28, 29, 30, 31};
const int outputs16Count = 4;

byte inputs16[] = {24, 25, 26, 27};
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
  keys64Value[0][0] = "+r6";
  keys64Value[0][1] = "+rE";
  keys64Value[0][2] = "+r2";
  keys64Value[0][3] = "+rA";
  keys64Value[1][0] = "+r4";
  keys64Value[1][1] = "+rC";
  keys64Value[1][2] = "+r0";
  keys64Value[1][3] = "+r8";
  keys64Value[2][0] = "+r7";
  keys64Value[2][1] = "+rF";
  keys64Value[2][2] = "+r3";
  keys64Value[2][3] = "+rB";
  keys64Value[3][0] = "+r5";
  keys64Value[3][1] = "+rD";
  keys64Value[3][2] = "+r1";
  keys64Value[3][3] = "+r9";

  // green
  keys64Value[4][0] = "+g6";
  keys64Value[4][1] = "+gE";
  keys64Value[4][2] = "+g2";
  keys64Value[4][3] = "+gA";
  keys64Value[5][0] = "+g4";
  keys64Value[5][1] = "+gC";
  keys64Value[5][2] = "+g0";
  keys64Value[5][3] = "+g8";
  keys64Value[6][0] = "+g7";
  keys64Value[6][1] = "+gF";
  keys64Value[6][2] = "+g3";
  keys64Value[6][3] = "+gB";
  keys64Value[7][0] = "+g5";
  keys64Value[7][1] = "+gD";
  keys64Value[7][2] = "+g1";
  keys64Value[7][3] = "+g9";

  // blue (u)
  keys64Value[0][4] = "+u6";
  keys64Value[0][5] = "+uE";
  keys64Value[0][6] = "+u2";
  keys64Value[0][7] = "+uA";
  keys64Value[1][4] = "+u4";
  keys64Value[1][5] = "+uC";
  keys64Value[1][6] = "+u0";
  keys64Value[1][7] = "+u8";
  keys64Value[2][4] = "+u7";
  keys64Value[2][5] = "+uF";
  keys64Value[2][6] = "+u3";
  keys64Value[2][7] = "+uB";
  keys64Value[3][4] = "+u5";
  keys64Value[3][5] = "+uD";
  keys64Value[3][6] = "+u1";
  keys64Value[3][7] = "+u9";

  // yellow
  keys64Value[4][4] = "+y5";
  keys64Value[4][5] = "+y7";
  keys64Value[4][6] = "+y4";
  keys64Value[4][7] = "+y6";
  keys64Value[5][4] = "+yD";
  keys64Value[5][5] = "+yF";
  keys64Value[5][6] = "+yC";
  keys64Value[5][7] = "+yE";
  keys64Value[6][4] = "+y1";
  keys64Value[6][5] = "+y3";
  keys64Value[6][6] = "+y0";
  keys64Value[6][7] = "+y2";
  keys64Value[7][4] = "+y9";
  keys64Value[7][5] = "+yB";
  keys64Value[7][6] = "+y8";
  keys64Value[7][7] = "+yA";

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
  
  // white (matrix 16)
  keys16Value[0][0] = "+wF";
  keys16Value[0][1] = "+wE";
  keys16Value[0][2] = "+wD";
  keys16Value[0][3] = "+wC";
  keys16Value[1][0] = "+wB";
  keys16Value[1][1] = "+wA";
  keys16Value[1][2] = "+w9";
  keys16Value[1][3] = "+w8";
  keys16Value[2][0] = "+w7";
  keys16Value[2][1] = "+w6";
  keys16Value[2][2] = "+w5";
  keys16Value[2][3] = "+w4";
  keys16Value[3][0] = "+w3";
  keys16Value[3][1] = "+w2";
  keys16Value[3][2] = "+w1";
  keys16Value[3][3] = "+w0";

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
