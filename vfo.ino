/*******************************************************************************
Title: VFO for ILER-20
Author: Corrado Gerbaldo - IU1BOW

-------------------------------------------------------------------------------
TODO: get attenuator ?! or external switch?
TODO: get meter
TODO: get voltage
TODO: code optimization
--------------------------------------------------------------------------------
External compiling
# arduino --verify vfo.ino
# arduino --upload vfo.ino
see: https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc

-------------------------------------------------------------------------------
External serial monitor - picocom
# picocom -b 9600 /dev/ttyUSBO
CTRL+A+X for exit: https://www.youtube.com/watch?v=cKcg9utFi5A
manual: https://linux.die.net/man/8/picocom
xxx

*******************************************************************************/

/*------------------------------------------------------------------------------
Includes
------------------------------------------------------------------------------*/
#include <Wire.h>
#include "U8glib.h"
#include <math.h>


/*------------------------------------------------------------------------------
Constants
------------------------------------------------------------------------------*/
//#define DEBUG 1  //Comment/uncomment this line to toggle debug

//AD9850
const int W_CLK = 8;       // Pin 8 - connect to AD9850 module word load clock pin (CLK)
const int FQ_UD = 9;       // Pin 9 - connect to freq update pin (FQ)
const int DATA = 10;      // Pin 10 - connect to serial data load pin (DATA)
const int RESET = 11;      // Pin 11 - connect to reset pin (RST).

//encoder
const int C_ENCODER_PIN_2 = 2; //D2
const int C_ENCODER_PIN_1 = 3; //D3
const int C_ENCODER_PIN_SW = 4;
const byte C_BOUNCE_TIME = 50;
const int C_HOLD_TIME = 1500;
const int C_DOUBLE_TIME = 500;
const int C_RESET_ENCODER_TIME=1000;

//PTT
const int PTT_PIN = A1;

//Voltage
const int VIN_PIN = A6;
const float R1 = 47200.0;
const float R2 = 9910.0;

//S-meter
const int SMETER_PIN = A7;

//frequency
const unsigned long MAX_FREQ = 14350000; //Max Frequency
const unsigned long MIN_FREQ =14000000; // Minimum Frequency
//const unsigned long START_FREQ =14220000; // Start Frequency
const unsigned long START_FREQ =14195000; // Start Frequency
const double IF=3276000;
//const double FREQ_CORRECTION = 200; //correction in hz
const double FREQ_CORRECTION = 400; //correction in hz

//GUI
const byte C_GUI_FQ_X = 20;
const byte C_GUI_FQ_Y = 12;
const byte C_GUI_VFO_X = 2;
const byte C_GUI_VFO_Y = 8;
#define C_METER "S 1   .   5   .   9  ++"
//#define C_METER "S 1 2 3 4 5 6 7 8 9 10 +"

//sleep: time after that the screen go in sleep mode
const double C_TIME_SLEEP = 900000;

//Initial initial screen
const byte SCREEN_WIDTH=128;
const byte SCREEN_HEIGHT=32;

static unsigned char initial_screen_bits[] U8G_PROGMEM = {
   0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x18, 0x06, 0x03,
   0x07, 0x7e, 0x00, 0x0f, 0x0c, 0x87, 0x03, 0x00, 0x80, 0x3b, 0x00, 0x00,
   0x00, 0x18, 0x06, 0xc3, 0x07, 0xfe, 0xc1, 0x3f, 0x1c, 0x87, 0x03, 0x00,
   0xc0, 0x7b, 0x00, 0x00, 0x00, 0x18, 0x06, 0x43, 0x06, 0xc6, 0xe1, 0x70,
   0x1c, 0x8f, 0x01, 0x00, 0xe0, 0xd6, 0x00, 0x00, 0x00, 0x18, 0x06, 0x03,
   0x06, 0x86, 0x71, 0x70, 0x18, 0x8f, 0x01, 0x00, 0xe0, 0xea, 0x00, 0x00,
   0x00, 0x18, 0x06, 0x03, 0x06, 0xc6, 0x71, 0x60, 0x18, 0xcd, 0x01, 0x00,
   0xe0, 0xf5, 0x01, 0x00, 0x00, 0x18, 0x06, 0x03, 0x06, 0xfe, 0x70, 0xe0,
   0xb8, 0xc9, 0x01, 0x00, 0xc0, 0xfb, 0x01, 0x00, 0x00, 0x18, 0x06, 0x03,
   0x06, 0xfe, 0x71, 0x60, 0xb8, 0xd9, 0x00, 0x00, 0x80, 0x9f, 0x03, 0x00,
   0x00, 0x18, 0x06, 0x03, 0x06, 0x86, 0x73, 0xe0, 0xb0, 0xd9, 0x00, 0x00,
   0x00, 0x9e, 0x01, 0xf8, 0x01, 0x18, 0x86, 0x03, 0x06, 0x86, 0x73, 0x70,
   0xf0, 0xf8, 0x00, 0x00, 0x00, 0xfe, 0x01, 0xfe, 0x07, 0x18, 0x8e, 0x03,
   0x06, 0x86, 0xe3, 0x70, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0xf8, 0x03, 0x0f,
   0x0e, 0x18, 0xfc, 0xc1, 0x3f, 0xfe, 0xc1, 0x3f, 0xf0, 0x70, 0x00, 0x00,
   0x00, 0x00, 0xef, 0x03, 0x0c, 0x18, 0x78, 0xc0, 0x3f, 0xff, 0x80, 0x0f,
   0xf0, 0x70, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x1c, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00,
   0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x80, 0xaa, 0xaa, 0xaa, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff,
   0x07, 0xcc, 0xc0, 0xcf, 0x07, 0xc0, 0xc0, 0x00, 0x1c, 0x7c, 0xf8, 0x00,
   0xe0, 0x5a, 0x1b, 0x00, 0x07, 0xcc, 0xc0, 0xc0, 0x0c, 0x10, 0x23, 0x03,
   0x62, 0xcc, 0x18, 0x03, 0xe0, 0x4a, 0x1d, 0x00, 0x07, 0xcc, 0xc0, 0xc0,
   0x18, 0x00, 0x33, 0x02, 0xc3, 0x8c, 0x19, 0x03, 0xe0, 0x5a, 0x1b, 0x00,
   0x07, 0xcc, 0xc0, 0xc0, 0x08, 0x00, 0x33, 0x02, 0xc3, 0x8c, 0x18, 0x03,
   0xe0, 0x5a, 0xb9, 0x24, 0x07, 0xcc, 0xc0, 0xcf, 0x0f, 0x80, 0x31, 0x06,
   0x83, 0xfc, 0x18, 0x03, 0xe0, 0x52, 0xfb, 0xff, 0x07, 0xcc, 0xc0, 0xc0,
   0x8c, 0x83, 0x31, 0x02, 0xc3, 0xcc, 0xf8, 0x00, 0xe0, 0x5a, 0x3d, 0x1f,
   0x07, 0xcc, 0xc0, 0xc0, 0x98, 0xc3, 0x30, 0x03, 0xc3, 0x8c, 0x19, 0x00,
   0xe0, 0x5a, 0x1b, 0x1e, 0x07, 0xcc, 0xc0, 0xc0, 0x18, 0x60, 0x20, 0x03,
   0x46, 0x8c, 0x19, 0x00, 0xe0, 0x4a, 0x39, 0x1e, 0x07, 0xcc, 0xcf, 0xcf,
   0x30, 0xf0, 0xe3, 0x00, 0x78, 0x0c, 0x1b, 0x00, 0xe0, 0xff, 0xff, 0xff,
   0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
   0xc0, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x40, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*------------------------------------------------------------------------------
Global variables
------------------------------------------------------------------------------*/

//frequency
double frequency_VFOA, frequency_VFOB;
double last_frequency_VFOA,last_frequency_VFOB= 14195000;
double frequency_step;

//encoder
unsigned long sv_timer_encoder;
enum t_vfo {
  VFOA,
  VFOB
};
t_vfo VFO=VFOA;
volatile int lastEncoded=0;
int lastReading = LOW;
int hold = 0;
int single = 0;
long onTime = 0;
long lastSwitchTime = 0;
byte speed_array[4]={1,1,1,1};

//split
bool split = 0;

//ptt
bool ptt = 0;

//sleep
bool sleep = 0;

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/
#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }
#define AVG4(A, B, C, D)     (A + B + C + D)/4
U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_NONE);  // I2C / TWI

/*==============================================================================
Functions
==============================================================================*/

/*------------------------------------------------------------------------------
AD9850
transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
------------------------------------------------------------------------------*/
void tfr_byte(byte data)
{
 for (int i=0; i<8; i++, data>>=1) {
   digitalWrite(DATA, data & 0x01);
   pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
 }
}

/*------------------------------------------------------------------------------
AD9850
Frequency send to AD9850.
Receive the frequency of VFOA, VFOB, the VFO selected and the split.

Calculate the frequency using the selected frequency and, sum the "correction"
and subtract the IF.
Then  <sys clock> * <frequency tuning word>/2^32

------------------------------------------------------------------------------*/
void sendFrequency(double fa, double fb, t_vfo vfo,bool sp) {
  double f;

  static double sv_f = 0;

  //manage split
  if (( sp == 1) && ( ptt == 1)) {
    if ( vfo == VFOA) {
      vfo=VFOB;
    } else {
      vfo=VFOA;
    };
  };

  //manage VFO freq
  if ( vfo == VFOA) {
    f=fa;
  } else {
    f=fb;
  };

  if (sv_f != f) {
    sv_f = f;
    f = f+FREQ_CORRECTION;
    #if defined(DEBUG)
     Serial.print("Freq: ");
     Serial.print(f);
     Serial.print(" corrected to: ");
     Serial.print(f);
    #endif

    f = f-IF;

    int32_t freq = f * 4294967295/125000000;  // note 125 MHz clock on 9850
    #if defined(DEBUG)
     Serial.print(" send to AD9850: ");
     Serial.println(freq);
    #endif

    for (int b=0; b<4; b++, freq>>=8) {
     tfr_byte(freq & 0xFF);
    }
    tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
    pulseHigh(FQ_UD);  // Done!  Should see output
  };
};

/*------------------------------------------------------------------------------
ENCODER INTERRUPT
Assign the frequency value to the encoder
At every thik of the encoder check the speed and get the Step
So increase or decrease the freq
------------------------------------------------------------------------------*/
void updateEncoder(){

  double encoderValue;

  //assign the frequency to encoder value
  if (VFO==VFOA) {
    encoderValue=frequency_VFOA;
   } else {
    encoderValue=frequency_VFOB;
   };

   //get the encoder value
  int MSB = digitalRead(C_ENCODER_PIN_1); //MSB = most significant bit
  int LSB = digitalRead(C_ENCODER_PIN_2); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  //CV
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b1011) {
    encoderValue = encoderValue +frequency_step;
  //if an entire tick so get the frequency step basing on the AVG of the lasts
  // 4 checks
} else if (sum == 0b0010) {
    frequency_step=getSpeedAVG(sv_timer_encoder);
    encoderValue = encoderValue +frequency_step;
    #if defined(DEBUG)
      Serial.print ("Step CV : ");
      Serial.println (frequency_step);
    #endif
  //CCV
  } else if (sum == 0b1110 || sum == 0b0001 || sum == 0b1000) {
    encoderValue = encoderValue - frequency_step;
    //if an entire tick so get the frequency step basing on the AVG of the lasts
    // 4 checks
  } else if (sum == 0b0111) {
    frequency_step=getSpeedAVG(sv_timer_encoder);
    encoderValue = encoderValue-frequency_step;
    #if defined(DEBUG)
      Serial.print ("Step CCV: ");
      Serial.println (frequency_step);
    #endif
  };

  lastEncoded = encoded; //store this value for next time

  //set Freq
  if (encoderValue > MAX_FREQ) {
     encoderValue = MAX_FREQ;
  } else if (encoderValue <MIN_FREQ) {
    encoderValue = MIN_FREQ;
  };

  if (VFO==VFOA) {
    frequency_VFOA=encoderValue;
  } else {
    frequency_VFOB=encoderValue;
  };

};

/*------------------------------------------------------------------------------
ENCODER
shift the array by 1
then at the last byte put a value (1,2,3) basing on the speed.
If its pass too time from last encoding value, then put all bytes to 1

Retur 10^avg of all array elements
------------------------------------------------------------------------------*/
double getSpeedAVG(unsigned long &svt){

  memmove(speed_array, speed_array+1,3*sizeof(*speed_array));

  unsigned long timer_encoder = millis();
  unsigned timer_diff = timer_encoder - svt;

  if (timer_diff > C_RESET_ENCODER_TIME) {
    speed_array[0]=1;
    speed_array[1]=1;
    speed_array[2]=1;
    speed_array[3]=1;
    //memset(spar,1,3*sizeof(*spar));
  } else if (timer_diff > 400){
    speed_array[3]=1;
  } else if (timer_diff < 120) {
    speed_array[3]=3;
  } else {
    speed_array[3]=2;
  };

  svt=timer_encoder;

  //wake up (if it's sleeping)
  if (sleep == 1) {
    sleep = 0;
    u8g.sleepOff();
  //  delay(100);
    #if defined(DEBUG)
     Serial.println("Sleep mode OFF");
    #endif
    //delay(50);
  }

  //return pow(10,(round((double)((speed_array[0]+speed_array[1]+speed_array[2]+speed_array[3])/4))));
  return pow(10,(round((double)(AVG4(speed_array[0],speed_array[1],speed_array[2],speed_array[3])))));

};

/*------------------------------------------------------------------------------
Encoder: Release button
------------------------------------------------------------------------------*/
//release button
void onRelease() {

 if ((millis() - lastSwitchTime) >= C_DOUBLE_TIME) {
   single = 1;
   lastSwitchTime = millis();
   return;
 }

    //double press VFOA/B switch
 if ((millis() - lastSwitchTime) < C_DOUBLE_TIME) {
   single = 0;
   lastSwitchTime = millis();
   #if defined(DEBUG)
      Serial.println("double press");
   #endif
 }

};

/*------------------------------------------------------------------------------
GUI
Format the frequency using dots
------------------------------------------------------------------------------*/
String formattedFreq(double f) {
  String sf (f,0);
  return sf.substring(0,2)+"."+sf.substring(2,5)+"."+sf.substring(5,7);
};

/*------------------------------------------------------------------------------
GUI
when invoched refresh all informations on display
- VFO A/B
- Frequency
------------------------------------------------------------------------------*/

void GUIRefresh(double fa,double fb, t_vfo v, bool sp, float vin, byte smeter) {

  char buf[10];

  //manage split
  if (( sp == 1) && ( ptt == 1)) {
    if ( v == VFOA) {
      v=VFOB;
    } else {
      v=VFOA;
    };
  };

  u8g.firstPage();
  do {

    //erase VFO A/B indicator
    u8g.drawBox(0,0,9,9);
    u8g.setFont(u8g_font_6x12);
    u8g.setColorIndex(0);

    //display VFO A/B indicator
    if (v==VFOA){
      formattedFreq(fa).toCharArray(buf,10);
      u8g.drawStr(C_GUI_VFO_X,C_GUI_VFO_Y,"A");   //display VFOA indicator
    } else {
      formattedFreq(fb).toCharArray(buf,10);
      u8g.drawStr(C_GUI_VFO_X,C_GUI_VFO_Y,"B");   //display VFOB indicator
    }

    //display frequency
    u8g.setColorIndex(1);
    u8g.setFont(u8g_font_courB14);
    u8g.drawStr(C_GUI_FQ_X,C_GUI_FQ_Y,buf);  //display frequency

    u8g.setFont(u8g_font_profont11);
    //u8g.drawStr(0,20,"13.8 S A TX");

    //voltage
    u8g.setColorIndex(1);
    String vinFormat (vin,1);
    if (vin < 10.0) {
      vinFormat="0"+vinFormat;
    };
    vinFormat="+"+vinFormat+"V";
    vinFormat.toCharArray(buf,10);
    u8g.drawStr(0,20,buf);

    //split indicator
    if (sp==0){
      u8g.setColorIndex(0);
      u8g.drawStr(40,20,"S");
    } else {
      u8g.setColorIndex(1);
      u8g.drawStr(40,20,"S");
    }

    //tx indicator
    if (ptt==0){
      u8g.setColorIndex(1);
      u8g.drawStr(50,20,"RX");
    } else {
      u8g.setColorIndex(1);
      u8g.drawStr(50,20,"TX");
    }

    //display meter bar
    u8g.setColorIndex(1);
    u8g.setFont(u8g_font_profont10);
    u8g.drawStr(0,29,C_METER);

    //if ptt is pressed the smeter is zero
    if (ptt == 1) {
        u8g.setColorIndex(0);
        u8g.drawBox(0,SCREEN_HEIGHT-2,SCREEN_WIDTH,SCREEN_HEIGHT);
    } else {
        u8g.drawBox(0,SCREEN_HEIGHT-2,smeter*SCREEN_WIDTH/11,SCREEN_HEIGHT);
    };

    u8g.setColorIndex(1);

  } while ( u8g.nextPage() );

};

/******************************************************************************
SETUP
*******************************************************************************/
void setup() {

  #if defined(DEBUG)
    Serial.begin(9600);
    Serial.println("DEBUG");
  #endif

  // initialize and clear display
  u8g.begin();

/*
  const int W_CLK = 8;       // Pin 8 - connect to AD9850 module word load clock pin (CLK)
  const int FQ_UD = 9;       // Pin 9 - connect to freq update pin (FQ)
  const int DATA = 10;      // Pin 10 - connect to serial data load pin (DATA)
  const int RESET = 11;      // Pin 11 - connect to reset pin (RST).

  //encoder
  // A logical OR against zero or a logical AND against one
  // will not change the status of a bit.
  //encoder
  /*pinMode(C_ENCODER_PIN_1,INPUT_PULLUP);
  pinMode(C_ENCODER_PIN_2,INPUT_PULLUP);
  pinMode(C_ENCODER_PIN_SW,INPUT_PULLUP);*/
  DDRD=DDRD & B11100011;   //pinmode input to D2, D3, D4
  PORTD = PORTD | B00011100; //enable pullup on D2, D3, D4
  //DDRD=B00011100;

  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  // configure arduino data pins for output to AD9850 DDS
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);
//  DDRB = DDRB | B00001111;

  //enable AD9850 DDS
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // this pulse enables serial mode - Datasheet page 12 figure 10

  //enable pin to read PTT
  //pinMode(PTT_PIN, INPUT);

  //enable pin to read voltage
  //pinMode(VIN_PIN, INPUT);

#if not defined(DEBUG)
  //display initial screen/logo
  u8g.firstPage();
  do {
    u8g.drawXBMP( 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, initial_screen_bits);
  } while ( u8g.nextPage() );
  delay(3000);
#endif

  frequency_VFOA=START_FREQ;
  frequency_VFOB=START_FREQ;

  //GUIRefresh(frequency_VFOA,frequency_VFOB, VFO,split,getVIN());
  GUIRefresh(frequency_VFOA,frequency_VFOB, VFO,split,0,0);

  #if defined(DEBUG)
    Serial.println("READY");
  #endif

};

/*------------------------------------------------------------------------------
getVIN: get the voltage
------------------------------------------------------------------------------*/
float getVIN() {

  int value = analogRead(VIN_PIN);
  float v1 = (value * 4.98) / 1024.0;
  //float vout = (v1/(R2/(R1+R2)))+1.4;
  float vout = (v1/(R2/(R1+R2)));

  return vout;
}

/*------------------------------------------------------------------------------
getSMeter: get the s-meter value
------------------------------------------------------------------------------*/
byte getSMeter(float v) {

  float value = analogRead(SMETER_PIN)*13.3/v;
  Serial.print (v);
  Serial.print (" ");

  byte vout;
  //float vout = 5-((value*5)/1024);
  //TODO: check the value and return 1, 2, 3....
  //TODO: considerare anche il voltaggio V-- valori impostati a 13,5v
  Serial.print ("Smeter: ");
  Serial.print(value);
  if (value > 1020 ) {
    vout = 0;
  } else if (value > 930 ){
    vout = 1;
  } else if (value > 800 ){
    vout = 2;
  } else if (value > 650 ){
    vout = 3;
  } else if (value > 600 ){
    vout = 4;
  } else if (value > 550 ){
    vout = 5;
  } else if (value > 450 ){
    vout = 6;
  } else if (value > 350 ){
    vout = 7;
  } else if (value > 250 ){
    vout = 8;
  } else if (value > 150 ){
    vout = 9;
  } else if (value > 100 ){
    vout = 10;
  } else {
    vout = 11;
  } ;
  Serial.print ( " --> ");
  Serial.println(vout);
  return vout;
}

/******************************************************************************
MAIN LOOP
*******************************************************************************/
void loop() {

  long int timer_diff = millis()- sv_timer_encoder;

// you now have the values of all eight pins in the PIND register
// contained in a variable. The only pin we care about is pin 2.
// So we do a logical AND on the button variable to isolate the
// bit we want.

//  int reading = !digitalRead(C_ENCODER_PIN_SW);
  int reading = !(PIND & B00010000);

  //first pressed
   if (reading == HIGH && lastReading == LOW) {
     onTime = millis();
   }

  //Hold for more time
   if (reading == HIGH && lastReading == HIGH) {
     if ((millis() - onTime) > C_HOLD_TIME) {
       #if defined(DEBUG)
        Serial.println("hold");
       #endif
       hold = 1;
     }
   }

  //released
   if (reading == LOW && lastReading == HIGH) {
     if (((millis() - onTime) > C_BOUNCE_TIME) && hold != 1) {
       onRelease();
     }
     if (hold == 1) {
       if (split == 0){
          split=1;
        } else {
          split=0;
        };
       #if defined(DEBUG)
        Serial.println("released");
       #endif
       hold = 0;
     }
   }
   lastReading = reading;

  //single press
   if (single == 1 && (millis() - lastSwitchTime) > C_DOUBLE_TIME) {
     #if defined(DEBUG)
        Serial.println("single press");
     #endif
     single = 0;
     if (VFO==VFOA) {
      VFO=VFOB;
     } else {
      VFO=VFOA;
     };

    #if defined(DEBUG)
      Serial.print("VFO changed to: ");
      Serial.println(VFO);
    #endif
   //  GUIRefresh(frequency_VFOA,frequency_VFOB, VFO,split);
   }

  if (analogRead(PTT_PIN)>200) {
    #if defined(DEBUG)
      Serial.println("PTT pressed");
    #endif
    ptt = 1;
    //sendFrequency(frequency_VFOA,frequency_VFOB,VFO,split);
  } else {
    ptt = 0;
  }


  //GUIRefresh(frequency_VFOA,frequency_VFOB,VFO,split,getVIN());
  float vin=getVIN();
  GUIRefresh(frequency_VFOA,frequency_VFOB,VFO,split,vin,getSMeter(vin));
  sendFrequency(frequency_VFOA,frequency_VFOB,VFO,split);

  if (frequency_VFOA != last_frequency_VFOA) {
  //   GUIRefresh(frequency_VFOA,frequency_VFOB,VFO);
    //sendFrequency(frequency_VFOA,frequency_VFOB,VFO,split);
    last_frequency_VFOA = frequency_VFOA;
  };

  if (frequency_VFOB != last_frequency_VFOB) {
  //   GUIRefresh(frequency_VFOA,frequency_VFOB,VFO);
    //sendFrequency(frequency_VFOA,frequency_VFOB,VFO,split);
    last_frequency_VFOB = frequency_VFOB;
  };

/*

   if (timer_diff > C_RESET_ENCODER_TIME) {
     speed_array[0]=1;
     speed_array[1]=1;
     speed_array[2]=1;
     speed_array[3]=1;
     frequency_step=10;
   };

*/
   //go to bed

  if ((timer_diff > C_TIME_SLEEP) && (sleep == 0 )) {
    sleep = 1;
    u8g.sleepOn();
    #if defined(DEBUG)
      Serial.println("Sleep mode ON");
    #endif
   //reset speeed control rotarty C_ENCODER_PIN_1
 }  else if (timer_diff > C_RESET_ENCODER_TIME) {
    speed_array[0]=1;
    speed_array[1]=1;
    speed_array[2]=1;
    speed_array[3]=1;
    frequency_step=10;
  };


};
