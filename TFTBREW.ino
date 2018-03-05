// TFT Brew
//
// 2018 By brewzone-systems.co.uk
//
// To utilize a Mega2560 and its additional pins
// allowing a 320*480 TFT LCD colour touch screen.
// 
// Bluetooth LE using a HM10/CC41-A or similar.
// 
// SD card reading/writing of PID parameters for analysis
// either on the Mega2560 or PC/Andriod etc.
//
// Also the Max31865 and PT100 allows for higher precision
//
// This is based on e.Brew
// A PID controller for single-kettle brewing. 
// by Richard Oostindie
//
// Which is again based on:
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
//
// This is based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//------------------------------------------------------------------

//==============================================//
// Include the various libraries                //
//==============================================//

#include <includes_mega.h>                      // Putting the libraries here helps during compile time

//==============================================//
// PID Stuff
//==============================================//

double Input;
double Input0;
double Input1;
double Output;
double Output0;
double Output1;
double ratio0;
double ratio1;
double Kp;
double Ki;
double Kd;
double PIDsetpoint;
//long DC;
PID myPID(&Input, &Output, &PIDsetpoint, Kp, Ki, Kd,P_ON_E, DIRECT); //Specify the links and initial tuning parameters
int WindowSize = 1000; // 1 second Time Proportional Output window

unsigned long windowStartTime;
byte ATuneModeRemember=2;
double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;
boolean tuning = false;
PID_ATune aTune(&Input, &Output);
unsigned long lastInput = 0; // last button press
int tt;
String StrKettleSensor;
int IntKettleSensor;

// Timer stuff

elapsedMillis timeElapsed; //declare global if you don't want it reset every time loop runs

unsigned int interval = 60000; // 1 minute interval

int ElapsedMin = 0;

int ElapsedMin = 0;

long time_frame = 0;

long loop_count = 0;

//EEPROM

const int AlarmSpAddress = 0+33;
const int KpAddress = 8+33;
const int KiAddress = 16+33;
const int KdAddress = 24+33;
const int TMarginAddress = 32+33;

//Bluetooth CC41-A stuff

//const int mybaud = 19200;

//==============================================//
// Assign Max31865 pins + assign reference val  //b
//==============================================//

Adafruit_MAX31865 max0 = Adafruit_MAX31865(48, 51, 50, 52);
Adafruit_MAX31865 max1 = Adafruit_MAX31865(49, 51, 50, 52);

#define RREF 430.0

//====================================================//
// Set minimum and maximum values for averaging temps //
//====================================================//

float mnf0 = 120.0;
float mnf1 = 120.0;
float mxf0 = -20.0;
float mxf1 = -20.0;

//==========================================================================================//
// Set up values to virtualize a given vessels heating and cooling characteristics          //
// Makes finding Kp Ki Kd settings much easier as can be done from comfort of Your armchair //
//==========================================================================================//

double starttemp;
double tune = 0;
double heatingrate = 0;
double coolingrate = 0;
double heatrate = 0.02915;      // 17.49C divided by 600 seconds
double coolrate = 0.000685185;  // 0.411111111C divided by 600 seconds
double ramp = 0;
 
//int    reduction1 = 0;  // Used to apply drops in temperature to test PID's reaction to them
//int    reduction2 = 0;
//int    reduction3 = 0;

byte value; // Used by Eeprom write/read routines

byte sensors_on_off = 0;  // These added to provide Virtual functionality, graph displays and loop time testing
byte graph = 0;
volatile long looptime = 0;


// SDcard Stuff

//File Serial;

//const int chipSelect = 47;

//=====================================================================//
// Set up the arrays to enable timers for mash, boil hop additions etc //
//=====================================================================//

double TimerMargin;
int stage = 0;
double Setpoint[6] = {0,0,0,0,0,0};
int SpPos = 0;
double timer[6] = {0,0,0,0,0,0};
int TmPos = 0;
double hop[6] = {0,0,0,0,0,0};
int HopPos = 0;

int AlarmRaised = 0;
int BoilTimer = 0;
volatile long onTime = 0;

double AlarmSetpoint;

int NumSens = 2;

float increment;

byte update_text = 0;

//==============================================//
// States for state machine
//==============================================//

byte opState, OFF = 0,RUN = 1;

//==============================================//
// Buzzer                                       //
//==============================================//

#define buzzerPin A9
#define NOTE_C7  2093
#define NOTE_E7  2637
#define NOTE_G7  3136

//==============================================//
// SSR Relays                                   //
//==============================================//

#define Relaypin 43

//==============================================//
// TFT Stuff                                    //
//==============================================//

MCUFRIEND_kbv tft = MCUFRIEND_kbv();

// Set up the touch screen pins

uint8_t YP = A1;  // must be an analog pin, use "An" notation!
uint8_t XM = A2;// must be an analog pin, use "An" notation!
uint8_t XP = 6;   // can be a digital pin
uint8_t YM = 7;   // can be a digital pin

uint16_t TS_LEFT =  931;
uint16_t TS_RT   =  172;
uint16_t TS_TOP  =  913;
uint16_t TS_BOT  =  179;

uint8_t SwapXY = 0;

byte drawkeypad = 0;

byte sim = 0;

#define PRESSED   true
#define RELEASED  false

boolean  _btn_state = RELEASED;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;

#define SWAP(a, b) {uint16_t tmp = a; a = b; b = tmp;}

#define MINPRESSURE 10
#define MAXPRESSURE 1000

uint8_t buttons;

uint16_t identifier;
uint8_t Orientation = 3;    //Portrait

#define BUTTON_NONE     0
#define BUTTON_1        1
#define BUTTON_2        2
#define BUTTON_3        3
#define BUTTON_4        4
#define BUTTON_5        5
#define BUTTON_6        6
#define BUTTON_UP       7
#define BUTTON_DOWN     8
#define BUTTON_BACK     9
#define BUTTON_FORWARD  10
#define BUTTON_RESET    11
#define GETMEOUTOFHERE  12

uint8_t  _btn_last = BUTTON_NONE;

uint16_t xpos, ypos;

uint16_t loopstart;

double mash = 65.0;

double ave0 = 0.0;
double ave1 = 0.0;

uint16_t x0 = 39;
uint16_t x1 = 39;
uint16_t x2 = 39;

uint16_t xx0 = 39;
uint16_t xx1 = 39;
uint16_t xx2 = 39;

uint16_t y0 = 24;

uint16_t yy0 = 24;

//==============================================//
// Define some colours
//==============================================//

#define BLACK           0x0000      /*   0,   0,   0 */
#define WHITE           0x0FFF      /* 255, 255, 255 */
#define NAVY            0x001F      /*   0,   0, 128 */
#define GREEN           0x03E0      /*   0, 128,   0 */
#define DarkCyan        0x03EF      /*   0, 128, 128 */
#define VIOLET          0x7800      /* 128,   0,   0 */
#define Maroon          0x780F      /* 128,   0, 128 */
#define Olive           0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY       0xC618      /* 192, 192, 192 */
#define DarkGrey        0x7BEF      /* 128, 128, 128 */
#define BLUE            0x000F      /*   0,   0, 255 */
#define DarkGreen       0x07E0      /*   0, 255,   0 */
#define CYAN            0x07FF      /*   0, 255, 255 */
#define RED             0xF800      /* 255,   0,   0 */
#define MAGENTA         0xF81F      /* 255,   0, 255 */
#define YELLOW          0xAFE5      /* 255, 255,   0 */
#define ORANGE          0xFD20      /* 255, 165,   0 */
#define GreenYellow     0xAFE5      /* 173, 255,  47 */
#define Pink            0xF81F

//==============================================//
// DiSplay Variables and constants
//==============================================//

void setup()
{

Serial.begin(19200);

tft.reset();
identifier = tft.readID();
tft.begin(identifier);        // May nedd to adjust the display driver setting to 0x55 from 0x66 for correct colour settings (0x3A, 1, 0x55,      //Interlace Pixel Format [XX])
tft.setRotation(Orientation); // Which is found in MCUFRIEND_kbv.cpp

// Initialize the PID and related variables
LoadParameters();
   
//==============================================//
// PID Stuff
//==============================================//
      
myPID.SetTunings(Kp,Ki,Kd);
myPID.SetSampleTime(1000);
myPID.SetOutputLimits(0, WindowSize);   

//==============================================// 
// Start the MAX31865 Temperature Sensors //
//==============================================//

max0.begin(MAX31865_3WIRE);
max1.begin(MAX31865_3WIRE);

// Initialize Relay and Buzzer Control:

pinMode(Relaypin, OUTPUT);    // Output mode to drive relay
digitalWrite(Relaypin, LOW);  // make sure it is off to start

pinMode (buzzerPin, OUTPUT) ;   // Output mode to drive buzzer

// SDcard stuff

//pinMode(chipSelect, OUTPUT);

//SD.begin(chipSelect);

buzz(buzzerPin, NOTE_C7, 300,3);


//==================================================================================//
//Initialize Timer5 for sampling frequency 400Hz Changed to timer 5 as it is 16bit  //
//==================================================================================//
  
TCCR5B |= (1 << CS51);    //start timer5 with prescaler=8
TIMSK5 = (1 << TOIE5);    //enable timer overflow interrupt for Timer5  
TCNT5 = 60535;            //set counter to 60535, 5000 clicks will be 2.5 ms
         
}

//==============================================//
// Timer Interrupt Handler
// Changed to timer 5 as it is 16bit
//==============================================//

SIGNAL(TIMER5_OVF_vect) 
{

// Reset the timer5 count for 10ms
TCNT5 = 60535;
    
if (opState == OFF) {digitalWrite(Relaypin, LOW);} // make sure relay is off
  
else {DriveOutput();}

looptime ++;
   
}

//==============================================//

void loop()
{ 
  
  drawmainscreen();
  Reset();
  Off();
  reboot();  
  
}

//==============================================//

void Off()
{

draw_keypad1();
off_keys();
tft.fillRect(152,  2,  326,  316,    BLUE);

myPID.SetMode(MANUAL);

digitalWrite(Relaypin, LOW);  // make sure it is off
       
opState = OFF;

while (true)
{

  ReadButtons();

  if (buttons == BUTTON_1)
  {

    /*    Serial.close();
    delay(600000);
    */
    
    reboot(); // Obvious what this does

  }

  if (buttons == BUTTON_2) {Logging();} // Not being used due to Bluetooth being more flexible,
    
  if (buttons == BUTTON_FORWARD) {pid_menu();}

}

}

void reboot() 
{
  
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}

}

//==============================================//

void pid_menu()
{

draw_keypad1();
pid_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);    

while (true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {Tune_Sp();}

  if (buttons == BUTTON_2) {TuneD();}

  if (buttons == BUTTON_3) {TuneP();}
    
  if (buttons == BUTTON_4) {TuneI();}
    
  if (buttons == BUTTON_5) {Runrealvirt();} // Choose between realtime and virtual settings
    
  if (buttons == BUTTON_6) {set_time(); ShowTemp(); drawmainscreen(); draw_keypad1(); pid_keys();} // Continuously show temperature

  if (buttons == BUTTON_UP) {simulation();} // Turn virtualisation on/off

  if (buttons == BUTTON_DOWN) {Favourites();} // Some popular temperature settings

  if (buttons == BUTTON_BACK) {Off();}

  if (buttons == BUTTON_FORWARD) {timer_menu();} // Set boil mash and hop timers

  DoControl();
    
} // End of while loop
  
}

//==============================================//

void Tune_Sp()
{

draw_keypad1();
tunesp_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);    

tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,10);
tft.print(F("Mash Temp : "));tft.print(mash);tft.print(F(" C   "));
            
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,20);
tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F("   "));
  
if (Setpoint[SpPos] == 0 && SpPos >= 1) {Setpoint[SpPos] = Setpoint[(SpPos-1)];}

increment = 0;

while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {increment = 0.01;}

  if (buttons == BUTTON_2) {increment = 1;}

  if (buttons == BUTTON_3) {increment = 5;}

  if (buttons == BUTTON_4) {increment = 10;}

  if (buttons == BUTTON_5) {mash += increment; update_text = 1; if (mash > 68) {mash = 62;} update_text = 1;}

  if (buttons == BUTTON_6) {SpPos++; update_text = 1; if (SpPos > 5) {SpPos = 0;}}

  if (buttons == BUTTON_BACK) { if (SpPos == 0) {TmPos = SpPos; pid_menu();} else {TmPos = SpPos - 1; pid_menu();}}
                  
  if (buttons == BUTTON_UP &&  (Setpoint[SpPos] + increment) <= 115) {Setpoint[SpPos] += increment; update_text = 1;}

  if (buttons == BUTTON_DOWN && (Setpoint[SpPos] - increment) >= -15 && (Setpoint[SpPos] -increment >= Setpoint[(SpPos-1)])) {Setpoint[SpPos] -= increment; update_text = 1;}

  if (update_text == 1)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,10);
    tft.print(F("Mash Temp : "));tft.print(mash);tft.print(F(" C   "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F("   "));   

    update_text = 0;

  }

  DoControl();
    
} // End of while loop

}

//==============================================//

void TuneP()
{

draw_keypad1();
tunep_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
    
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,40);
tft.print(F("Set Kp : "));tft.print(Kp);tft.print(F("     "));

increment = 0;
  
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {increment = 0.01;}

  if (buttons == BUTTON_2) {increment = 1;}

  if (buttons == BUTTON_3) {increment = 10;}

  if (buttons == BUTTON_4) {increment = 100;}

  if (buttons == BUTTON_5) {increment = 500;}

  if (buttons == BUTTON_6) {increment = 1000;}

  if (buttons == BUTTON_UP) {Kp += increment; update_text = 1;}

  if (buttons == BUTTON_DOWN) {Kp -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {pid_menu();}
    
  if (update_text == 1)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Set Kp : "));tft.print(Kp);tft.print(F("     "));

    update_text = 0;

  }

  DoControl();

} // End of while loop

}

//==============================================//

void TuneI()
{

draw_keypad1();
tunei_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
    
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,50);
tft.print(F("Set Ki : "));tft.print(Ki);tft.print(F("     "));
    
//      delay(500);
   
while(true)
{

  ReadButtons();
  
  if (buttons == BUTTON_1) {increment = 0.01;}

  if (buttons == BUTTON_2) {increment = 1;}

  if (buttons == BUTTON_3) {increment = 10;}

  if (buttons == BUTTON_4) {increment = 100;}

  if (buttons == BUTTON_5) {increment = 500;}

  if (buttons == BUTTON_6) {increment = 1000;}

  if (buttons == BUTTON_UP) {Ki += increment; update_text = 1;}

  if (buttons == BUTTON_DOWN) {Ki -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {pid_menu();}

  if (update_text == 1)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Set Ki : "));tft.print(Ki);tft.print(F("     "));

    update_text = 0;

  }    

  DoControl();

} // End of while loop

}

//==============================================//

void TuneD()
{

draw_keypad1();
tuned_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
    
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,60);
tft.print(F("Set Kd : "));tft.print(Kd);tft.print(F("     "));

increment = 0;
   
while(true)
{

  ReadButtons();
      
  if (buttons == BUTTON_1) {increment = 0.01;}

  if (buttons == BUTTON_2) {increment = 1;}

  if (buttons == BUTTON_3) {increment = 10;}

  if (buttons == BUTTON_4) {increment = 100;}

  if (buttons == BUTTON_5) {increment = 500;}

  if (buttons == BUTTON_6) {increment = 1000;}

  if (buttons == BUTTON_UP) {Kd += increment; update_text = 1;}

  if (buttons == BUTTON_DOWN) {Kd -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {pid_menu();}
    
  if (update_text == 1)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Set Kd : "));tft.print(Kd);tft.print(F("     ")); 

    update_text = 0;

   }
        
   DoControl();
      
} // End of while loop

}

//==============================================//
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// LEFT - OFF
//==============================================//

void Run()
{

tft.setTextSize(1);

if (graph == 0)
{

  draw_keypad1();
  run_keys();
  tft.fillRect(152,  2 , 326,  316, BLUE);

}

// 2 short beeps for starting program
buzz(buzzerPin, NOTE_C7, 200, 2);

AlarmRaised = 0;

int TempReached = 0;
    
if (stage == 0)
{
  stage = 1;
  SpPos = 0;
  TmPos = 0;
  ElapsedMin = timer[TmPos];
  timeElapsed = 0;
  PIDsetpoint = Setpoint[SpPos];

}

if (graph == 0)
{

  tft.setTextColor(WHITE,BLUE);
  tft.setCursor(160,20);
  tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);

}            
            
myPID.SetTunings(Kp,Ki,Kd);

if (graph == 0)
{

  tft.setTextColor(WHITE,BLUE);
  tft.setCursor(160,40);
  tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

  tft.setTextColor(WHITE,BLUE);
  tft.setCursor(160,50);
  tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

  tft.setTextColor(WHITE,BLUE);
  tft.setCursor(160,60);
  tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

}

//    Serial = SD.open("TEST.TXT", FILE_WRITE);

increment = 0;

while(true)
{

  ReadButtons();
      
  if ((buttons == BUTTON_2) && (abs(Input - Setpoint[SpPos]) < 0.5))  {StartAutoTune();}

  if (buttons == BUTTON_3)
  {

    graph = 1;

    tft.setTextSize(1);

    // x0 = 0;
    y0 = 24;
    yy0 = 24;
    // x1 = 0; 

 
    tft.fillScreen(BLACK);

    tft.drawLine(39, 24, 39 ,289, BLUE);

    tft.drawLine((39+mash*4),25, (39+mash*4), 289, ORANGE);

    tft.drawLine(439, 24, 439 ,289, RED);

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 22,  290 );tft.print("Current");
    tft.setCursor( 2, 300 );
    tft.print(F("A: "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 2, 310 );
    tft.print(F("B: "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 140,  290 );tft.print("Lowest");
    tft.setCursor( 120, 300 );
    tft.print(F("A: "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 120, 310 );
    tft.print(F("B: "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 260,  290 );tft.print("Average");
    tft.setCursor( 240, 300 );
    tft.print(F("A: "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 240, 310 );
    tft.print(F("B: "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 380,  290 );tft.print("Highest");
    tft.setCursor( 360, 300 );
    tft.print(F("A: "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 360, 310 );
    tft.print(F("B: "));

    //tft.setTextColor(WHITE, VIOLET);
    //tft.setCursor( 430, 290 );
    //tft.print(F("Exit"));

  }

  if (buttons == BUTTON_BACK) {opState = OFF; pid_menu();}

  if (buttons == BUTTON_UP) {Setpoint[SpPos] += increment; PIDsetpoint = Setpoint[SpPos]; if (Setpoint[SpPos] >=100) {Setpoint[SpPos]=100;}}

  if (buttons == BUTTON_DOWN) {Setpoint[SpPos] -= increment; PIDsetpoint = Setpoint[SpPos]; if (Setpoint[SpPos] <=0){Setpoint[SpPos]=0;}}

  if (buttons == 5) {increment = 0.01;}

  if (buttons == 6) {increment = 1;}
            
  float FloatInput = Input;
  float FloatSetpoint = Setpoint[SpPos];
  float FloatTimerMargin = TimerMargin;
      
  if ( (FloatInput*1000) >= ( (FloatSetpoint*1000) - (FloatTimerMargin*1000) ) && TempReached == 0)
  {

    // first time temp reached. Set var and reset timer
    TempReached = 1;
    timeElapsed = 0;

    //Setpoint reached, 2 short beeps
    buzz(buzzerPin, NOTE_C7, 100, 2);
    delay(50);;
    buzz(buzzerPin, NOTE_C7, 100, 2);
    delay(50);

  }
      
  if ( (FloatInput*1000) >= (FloatSetpoint*1000) || TempReached == 1)
  {

  if (graph == 0)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   "));

  }

  if (timeElapsed > interval) 
  {

    ElapsedMin --;
    timeElapsed = 0;              // reset the counter to 0 so the counting starts over...

  }}

  if (ElapsedMin == 0 && SpPos <= 5)
  {

    if (Setpoint[(SpPos+1)] != 0 && timer[(TmPos+1)] != 0 && stage <= 6)
    {
      stage++;
      SpPos++;
      TmPos++;
      TempReached = 0;

      PIDsetpoint = Setpoint[SpPos];
      ElapsedMin = timer[TmPos];

      if (graph == 0)
      {

        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,40);
        tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,50);
        tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,60);
        tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

      }}

      if (stage == 6 || Setpoint[SpPos] == 100 || Setpoint == 0) {opState = OFF; reboot();}

      if (graph == 0)
      {

        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,20);
        tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);

      }}

      if (Setpoint[SpPos] != 100) {DoControl();}
      
      else if (Setpoint[SpPos] >= 100) {digitalWrite(Relaypin,HIGH); TakeTemp();}

      if (graph == 0)
      {

        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,30);
        tft.print(F("PV : "));tft.print(Input);tft.print(F(" C   "));
        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,150);
        tft.print(F("Sensor 1 : "));tft.print(ratio0);tft.print(F(" Max : "));tft.print(mxf0);tft.print(F(" C   "));
        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,160);
        tft.print(F("Sensor 2 : "));tft.print(ratio1);tft.print(F(" Max : "));tft.print(mxf1);tft.print(F(" C   "));

      }
      
      if (tuning)
      {

        if (graph == 0)
        {

          tft.setTextColor(WHITE,BLACK);
          tft.setCursor(2,20);
          tft.print(F("          Tuning    "));
       
          tft.setTextColor(WHITE,BLUE);
          tft.setCursor(160,80);
          tft.print(F("Output : "));tft.print((Output/WindowSize)*100);tft.print(F(" % "));tft.print(F(": "));tft.print(ramp);tft.print(F(" C   "));

          if (Setpoint[SpPos] < Input)
          {
            
            tft.setTextColor(WHITE,BLUE);
            tft.setCursor(160,90);
            tft.print(F("Overshoot : "));tft.print(((mxf1-Setpoint[SpPos])/Setpoint[SpPos])*100);tft.print(F(" % : "));tft.print((mxf1-Setpoint[SpPos]));tft.print(F(" C   "));

          }}

      sdcard_sub();

      }

      else

      {

      if (graph == 0)
      {

        tft.setTextColor(WHITE,BLACK);
        tft.setCursor(2,20);
        tft.print(F("         Running    "));

        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,80);
        tft.print(F("Output : "));tft.print((Output/WindowSize)*100);tft.print(F(" % "));tft.print(F(": "));tft.print(ramp);tft.print(F(" C   "));
        
        if (Setpoint[SpPos] < Input)
        {

          tft.setTextColor(WHITE,BLUE);
          tft.setCursor(160,90);
          tft.print(F("Overshoot : "));tft.print(((mxf1-Setpoint[SpPos])/Setpoint[SpPos])*100);tft.print(F(" % : "));tft.print((mxf1-Setpoint[SpPos]));tft.print(F(" C   "));

        }}

      sdcard_sub();

      }


      CheckAlarm();

      if(AlarmRaised == 1) {opState = OFF; pid_menu();}

      if (graph == 1)
      {

        mash = Setpoint[SpPos];

        tft.setTextColor(WHITE, BLACK);

        tft.setCursor( 22,  300 );tft.print(ratio0,2);tft.print(F(" C "));

        tft.setCursor( 22,  310 );tft.print(ratio1,2);tft.print(F(" C "));

        tft.setTextColor(WHITE, BLACK);

        tft.setCursor( 140,  300 );tft.print(mnf0,2);tft.print(F(" C "));

        tft.setCursor( 140,  310 );tft.print(mnf1,2);tft.print(F(" C "));

        tft.setTextColor(WHITE, BLACK);
     
        tft.setCursor( 260, 300 );tft.print(ave0,2);tft.print(F(" C "));

        tft.setCursor( 260, 310 );tft.print(ave1,2);tft.print(F(" C "));

        tft.setTextColor(WHITE, BLACK);

        tft.setCursor( 380, 300 );tft.print(mxf0,2);tft.print(F(" C "));

        tft.setCursor( 380, 310 );tft.print(mxf1,2);tft.print(F(" C "));

        x0 = 39 + (ratio0 * 4);
        x1 = 39 + (ratio1 * 4);
        x2 = 39 + (((Output/WindowSize)*100)*4);

        tft.drawLine(0, y0+1, 479 ,y0+1, WHITE);
        tft.drawLine(0, y0, 479 ,y0, BLACK);
        tft.drawLine(39, y0, 39 ,y0, BLUE);
        tft.drawLine((39+mash*4),y0, (39+mash*4), y0, ORANGE);
        tft.drawLine(439, y0, 439 ,y0, RED);
        tft.drawLine(x2, y0, xx2 , yy0 , MAGENTA);
        tft.drawLine(x0, y0, xx0, yy0, YELLOW);
        tft.drawLine(x1, y0, xx1, yy0 ,GREEN);

        xx0 = x0;
        xx1 = x1;
        xx2 = x2;

        yy0 = y0;

        y0 ++ ;

        if (y0 >= 289) //319    
        {

          tft.drawLine(0, y0, 479 ,y0, BLACK);

          y0 = 24;

          yy0 = y0-1;

         } 

        ReadButtons();
    
        if(buttons == GETMEOUTOFHERE)
        {

          drawmainscreen();
          draw_keypad1();
          run_keys();

          tft.fillRect(152,  2 , 326,  316, BLUE);
          tft.setTextColor(WHITE,BLUE);
          tft.setCursor(160,40);
          tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

          tft.setTextColor(WHITE,BLUE);
          tft.setCursor(160,50);
          tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

          tft.setTextColor(WHITE,BLUE);
          tft.setCursor(160,60);
          tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     ")); 

          tft.setTextColor(WHITE,BLUE);
          tft.setCursor(160,20);
          tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);
   
          graph = 0;
    
        }

      }

   }

}

//==============================================//
// Start the Auto-Tuning cycle                  //
//==============================================//

void StartAutoTune()
{

// Remember the mode we were in
ATuneModeRemember = myPID.GetMode();

// set up the auto-tune parameters
aTune.SetNoiseBand(aTuneNoise);
aTune.SetOutputStep(aTuneStep);
aTune.SetLookbackSec((int)aTuneLookBack);
tuning = true;

}

//==============================================//
// Return to normal control                     //
//==============================================//

void FinishAutoTune()
{

tuning = false;

// Extract the auto-tune calculated parameters

Kp = aTune.GetKp();
Ki = aTune.GetKi();
Kd = aTune.GetKd();

Serial.print(F("Finished"));Serial.println(F(",EOL"));

Serial.print(Kp);Serial.print(F(","));Serial.print(Ki);Serial.print(F(","));Serial.print(Kd);Serial.print(F(","));Serial.println(F(",EOL"));

//   Serial.close();

// Re-tune the PID and revert to normal control mode
myPID.SetTunings(Kp,Ki,Kd);
myPID.SetMode(ATuneModeRemember);
   
// Persist any changed parameters to EEPROM
SaveParameters();

}

void Runrealvirt()
{

draw_keypad1();
run_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);

increment = 0;
   
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1 && Setpoint[0] != 0 && timer[0] != 0 && NumSens >= 1) // Run program)
  {

    // Prepare to transition to the RUN state

    //turn the PID on
  
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = RUN; // start control
    ramp = 0;
    tune = 1.282441769;
    Run(); 

  }

  if (buttons == BUTTON_3 && Setpoint[0] != 0 && timer[0] != 0 && NumSens >= 1) // Run program))
  {

    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = RUN; // start control
    graph = 1;
    ramp = 0;
    tune = 2.08717398;
    ShowTemp();
    Run();
    graph = 0;
    drawmainscreen();
    draw_keypad1();
    pid_keys();    
    
  }

  if (buttons == BUTTON_BACK) {pid_menu();}

  DoControl();

} // End of while loop

}

void Favourites()

{

draw_keypad1();
Favourite_keys();
     
tft.fillRect(152,  2 , 326,  316, BLUE);

increment = 0;

SpPos = 0;
TmPos = 0;
ElapsedMin = timer[TmPos];
timeElapsed = 0;
PIDsetpoint = Setpoint[SpPos];
   
while(true)
{
    
  ReadButtons();

  if (buttons == BUTTON_1)
  {
      
    SpPos = 0;
    TmPos = 0;
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
    Setpoint[0] = 63.0;
    timer[0] = 90;
    hop[0] = 0;
    ElapsedMin = timer[TmPos];
        
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F(" "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   "));
  
    //    pid_menu();
        
  }

  if (buttons == BUTTON_2)
  {

    SpPos = 0;
    TmPos = 0;
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
    Setpoint[0] = 64.0;
    timer[0] = 90;
    hop[0] = 0;
    ElapsedMin = timer[TmPos];
    
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F(" "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   "));
    
    //    pid_menu();
         
  }

  if (buttons == BUTTON_3)
  {

    SpPos = 0;
    TmPos = 0;
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
    Setpoint[0] = 65.0;
    timer[0] = 90;
    hop[0] = 0;
    ElapsedMin = timer[TmPos];
    
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F(" "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   ")); 
    
    //    pid_menu();
         
  }

  if (buttons == BUTTON_4)
  {

    SpPos = 0;
    TmPos = 0;
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
    Setpoint[0] = 66.0;
    timer[0] = 90;
    hop[0] = 0;
    ElapsedMin = timer[TmPos];

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F(" "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   ")); 
    
    //    pid_menu();
         
  }

  if (buttons == BUTTON_5)
  {

    SpPos = 0;
    TmPos = 0;
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
    Setpoint[0] = 67.0;
    timer[0] = 90;
    hop[0] = 0;
    ElapsedMin = timer[TmPos];

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F(" "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   ")); 
    
    //    pid_menu();
         
  }

  if (buttons == BUTTON_6)
  {

    SpPos = 0;
    TmPos = 0;
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
    Setpoint[0] = 68.0;
    timer[0] = 90;
    hop[0] = 0;
    ElapsedMin = timer[TmPos];

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F(" "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   ")); 
    
    //    pid_menu();
         
  }

  if (buttons == BUTTON_UP)
  {

    SpPos = 0;
    TmPos = 0;
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
    Setpoint[0] = 74.0;
    timer[0] = 90;
    hop[0] = 0;
    ElapsedMin = timer[TmPos];

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);tft.print(F(" "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   ")); 
    
    //    pid_menu();
  
  }

  if (buttons == BUTTON_DOWN)
  {

    SpPos = 0;
    TmPos = 0;
    timeElapsed = 0;
    PIDsetpoint = Setpoint[SpPos];
    Setpoint[0] = 100.0;
    timer[0] = 90;
    hop[0] = 0;
    ElapsedMin = timer[TmPos];
    
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,20);
    tft.print(F("SP : "));tft.print(Setpoint[SpPos]);tft.print(F(" C   "));tft.print(F("SP Pos : "));tft.print(SpPos);

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,40);
    tft.print(F("Kp : "));tft.print(Kp);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,50);
    tft.print(F("Ki : "));tft.print(Ki);tft.print(F("     ")); 

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,60);
    tft.print(F("Kd : "));tft.print(Kd);tft.print(F("     "));

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Count Down Timer : "));tft.print(ElapsedMin);tft.print(F(" Mins   ")); 
    
    //    pid_menu();

  }

  if (buttons == BUTTON_BACK) {pid_menu();}

  if (buttons == BUTTON_FORWARD) {pid_menu();}

  DoControl();

} // End of while loop
  
}

void set_time()
{

draw_keypad1();
time_frame_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);

increment = 0;
   
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {time_frame = 0; ShowTemp(); time_frame_keys();}

  if (buttons == BUTTON_2) {time_frame = 6792; ShowTemp(); time_frame_keys();}

  if (buttons == BUTTON_3) {time_frame = 13584; ShowTemp(); time_frame_keys();}

  if (buttons == BUTTON_4) {time_frame = 163018; ShowTemp(); time_frame_keys();}

  if (buttons == BUTTON_5) {time_frame = 326037; ShowTemp(); time_frame_keys();}

  if (buttons == BUTTON_6) {time_frame = 2282264; ShowTemp(); time_frame_keys();}

  if (buttons == BUTTON_UP) {time_frame = 4564528; ShowTemp(); time_frame_keys();}

  if (buttons == BUTTON_DOWN) {time_frame = 9129056; ShowTemp(); time_frame_keys();}

  if (buttons == BUTTON_BACK) {pid_menu(); time_frame_keys();}

  if (buttons == BUTTON_FORWARD) {pid_menu(); time_frame_keys();}

  DoControl();

} // End of while loop
  
}

//==============================================//

void ShowTemp()
{

tft.setTextSize(1);

y0 = 24;

 
tft.fillScreen(BLACK);

tft.drawLine(39, 24, 39 ,289, BLUE);

tft.drawLine((39+mash*4),25, (39+mash*4), 289, ORANGE);

tft.drawLine(439, 24, 439 ,289, RED);

tft.drawLine(39, 0, 39 ,14, WHITE);

tft.drawLine(79, 0, 79 ,9, WHITE);

tft.drawLine(119, 0, 119 ,14, WHITE);

tft.drawLine(159, 0, 159 ,9, WHITE);

tft.drawLine(199, 0, 199 ,14, WHITE);

tft.drawLine(239, 0, 239 ,9, WHITE);

tft.drawLine(279, 0, 279 ,14, WHITE);

tft.drawLine(319, 0, 319 ,9, WHITE);

tft.drawLine(359, 0, 359 ,14, WHITE);

tft.drawLine(399, 0, 399 ,9, WHITE);

tft.drawLine(439, 0, 439 ,14, WHITE);

tft.setTextColor(WHITE, BLACK);
tft.setCursor( 22,  290 );tft.print("Current");
tft.setCursor( 2, 300 );
tft.print(F("A: "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor( 2, 310 );
tft.print(F("B: "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor( 140,  290 );tft.print("Lowest");
tft.setCursor( 120, 300 );
tft.print(F("A: "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor( 120, 310 );
tft.print(F("B: "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor( 260,  290 );tft.print("Average");
tft.setCursor( 240, 300 );
tft.print(F("A: "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor( 240, 310 );
tft.print(F("B: "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor( 380,  290 );tft.print("Highest");
tft.setCursor( 360, 300 );
tft.print(F("A: "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor( 360, 310 );
tft.print(F("B: "));

//tft.setTextColor(WHITE, VIOLET);
//tft.setCursor( 430, 290 );
//tft.print(F("Exit"));

loopstart=millis();

if (graph == 0)
{

  while (true)
  {
   
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 22,  300 );tft.print(ratio0,2);tft.print(F(" C "));
    tft.setCursor( 22,  310 );tft.print(ratio1,2);tft.print(F(" C "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 140,  300 );tft.print(mnf0,2);tft.print(F(" C "));
    tft.setCursor( 140,  310 );tft.print(mnf1,2);tft.print(F(" C "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 260, 300 );tft.print(ave0,2);tft.print(F(" C "));
    tft.setCursor( 260, 310 );tft.print(ave1,2);tft.print(F(" C "));

    tft.setTextColor(WHITE, BLACK);
    tft.setCursor( 380, 300 );tft.print(mxf0,2);tft.print(F(" C "));
    tft.setCursor( 380, 310 );tft.print(mxf1,2);tft.print(F(" C "));

    x0 = 39 + (ratio0 * 4);
    x1 = 39 + (ratio1 * 4);

    tft.drawLine(0, y0+1, 479 ,y0+1, WHITE);
    tft.drawLine(0, y0, 479 ,y0, BLACK);
    tft.drawLine(39, y0, 39 ,y0, BLUE);
    tft.drawLine((39+mash*4),y0, (39+mash*4), y0, ORANGE);
    tft.drawLine(439, y0, 439 ,y0, RED);
    tft.drawLine(x0, y0, x0, y0, YELLOW);
    tft.drawLine(x1, y0, x1, y0 ,GREEN);

    // Input = ratio0;

    // Output = 0;

    // sdcard_sub();

    // Input = ratio1;

    // Output = 2.55;

    sdcard_sub();

    y0 ++ ;

    if (y0 >= 289) //319    
    {

      tft.drawLine(0, y0, 479 ,y0, BLACK);

      y0 = 24;

    }

    for (long loop_count = 0; loop_count <= time_frame; loop_count++) {ReadButtons(); if(buttons == GETMEOUTOFHERE) {drawmainscreen(); draw_keypad1(); return;}}
    
    DoControl();

  }

} // End of while loop

}

void simulation()
{

draw_keypad1();
simulation_keys();

tft.fillRect(152,  2 , 326,  316, BLUE);
    
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,80);
tft.print(F("Tune    : "));tft.print(tune);

tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,100);
tft.print(F("Mins    : "));tft.print(mins);

tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,120);
tft.print(F("Heating : "));tft.print(heatrate);

tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,140);
tft.print(F("Cooling : "));tft.print(coolrate); 

increment = 0;
   
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {simheat();}

  if (buttons == BUTTON_2) {simcool();}

  if (buttons == BUTTON_3) {simtime();}

  if (buttons == BUTTON_4) {simtune();}

  if (buttons == BUTTON_5)
  {

    heatingrate = heatrate;

    coolingrate = coolrate;

    sim = 1;

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,160);
    tft.print(F("On ")); 

  }

  if (buttons == BUTTON_6)
  {

    heatingrate = 0;

    coolingrate = 0;

    sim = 0;    

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,160);
    tft.print(F("Off"));

   }

   if (buttons == BUTTON_BACK) {pid_menu();}
   
   if (buttons == BUTTON_FORWARD) {pid_menu();}

   DoControl();

} // End of while loop

}

void simheat()
{

draw_keypad1();
simheating_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
    
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,120);
tft.print(F("Heating : "));tft.print(heatrate);

increment = 0;
   
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {increment = 0.01;}

  if (buttons == BUTTON_2) {increment = 1;}

  if (buttons == BUTTON_3) {increment = 10;}

  if (buttons == BUTTON_5) {starttemp += increment; update_text = 1;}
  
  if (starttemp > 100) {starttemp = 0;}

  if (update_text == 1 ) 
  {
        
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,140);
    tft.print(F("Start   : "));tft.print(starttemp);tft.print(F(" C   "));

    update_text = 0;
  
  }
      
  if (buttons == BUTTON_UP ) {heatrate += increment; update_text = 1;}
   
  if (buttons == BUTTON_DOWN ) {heatrate -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {simulation();}

  if (buttons == BUTTON_FORWARD) {pid_menu();}

  if (update_text == 1 ) 
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("Heating : "));tft.print(heatrate);

    update_text = 0;

  }
       
   DoControl();

} // End of while loop

}

void simcool()
{

draw_keypad1();
simcooling_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
    
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,140);
tft.print(F("Cooling : "));tft.print(coolrate); 

increment = 0;
   
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {increment = 0.01;}

  if (buttons == BUTTON_2) {increment = 1;}

  if (buttons == BUTTON_3) {increment = 10;}

  if (buttons == BUTTON_UP ) {coolrate += increment; update_text = 1;}
   
  if (buttons == BUTTON_DOWN ) {coolrate -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {simulation();}

  if (buttons == BUTTON_FORWARD) {pid_menu();}

  tft.setTextColor(WHITE,BLUE);
  tft.setCursor(160,140);
  tft.print(F("Cooling : "));tft.print(coolrate); 

  DoControl();

} // End of while loop

}

void simtime()
{

draw_keypad1();
simtime_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
        
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,100);
tft.print(F("Minutes    : "));tft.print(mins);

increment = 0;
   
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {increment = 0.01;}

  if (buttons == BUTTON_2) {increment = 1;}

  if (buttons == BUTTON_3) {increment = 10;}

  if (buttons == BUTTON_UP ) {mins += increment; update_text = 1;}
   
  if (buttons == BUTTON_DOWN ) {mins -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {simulation();}

  if (buttons == BUTTON_FORWARD) {pid_menu();}

  tft.setTextColor(WHITE,BLUE);
  tft.setCursor(160,100);
  tft.print(F("Minutes    : "));tft.print(mins);

  DoControl();

} // End of while loop

}

void simtune()
{

draw_keypad1();
simtune_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
        
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,80);
tft.print(F("Tune    : "));tft.print(tune);

increment = 0;
   
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {increment = 0.01;}

  if (buttons == BUTTON_2) {increment = 0.1;}

  if (buttons == BUTTON_3) {increment = 1;}

  if (buttons == BUTTON_4) {increment = 10;}   

  if (buttons == BUTTON_UP ) {tune += increment; update_text = 1;}
   
  if (buttons == BUTTON_DOWN ) {tune -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {simulation();}

  if (buttons == BUTTON_FORWARD) {pid_menu();}


  tft.setTextColor(WHITE,BLUE);
  tft.setCursor(160,80);
  tft.print(F("Tune    : "));tft.print(tune);

  DoControl();

} // End of while loop

}

void Logging()
{

draw_keypad1();
Log_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
    
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,100);
    tft.print(F("SD On ")); 

  }

  if (buttons == BUTTON_2)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,100);
    tft.print(F("SD Off"));

  } 

  if (buttons == BUTTON_3)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("BT On ")); 

  }

  if (buttons == BUTTON_4)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,120);
    tft.print(F("BT Off"));

  }

  if (buttons == BUTTON_BACK) {Off();}

  if (buttons == BUTTON_FORWARD) {pid_menu();}

  DoControl();

} // End of while loop
  
}

//==============================================//

void timer_menu()
{

draw_keypad1();
timers_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);
      
while (true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {Tune_Time();}

  if (buttons == BUTTON_2) {Tune_TimerMargin();}

  if (buttons == BUTTON_3) {SelectKettleSensor();}

  if (buttons == BUTTON_4) {Hop_Alarm();}

  if (buttons == BUTTON_5) {SetAlarm();}

  if (buttons == BUTTON_BACK) {pid_menu();}

  if (buttons == BUTTON_FORWARD) {Off();}

  DoControl();

} // End of while loop

}

void Tune_Time()
{
  
draw_keypad1();
time_keys();

TmPos = 0;   

tft.fillRect(152,  2 , 326,  316, BLUE);
    
tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,120);
tft.print(F("Timer : "));tft.print(timer[TmPos]);tft.print(F(" Mins   "));tft.print(F(" TP : "));tft.print(TmPos);

increment = 0;
   
while(true)
{

  ReadButtons();
      
  if (buttons == BUTTON_1) {increment = 1;}

  if (buttons == BUTTON_2) {increment = 5;}
   
  if (buttons == BUTTON_3) {increment = 10;}

  if (buttons == BUTTON_4) {increment = 15;}

  if (buttons == BUTTON_5) {increment = 30;}

  if (buttons == BUTTON_6) {increment = 60;}
   
  if (buttons == BUTTON_UP) {timer[TmPos] += increment; update_text = 1;}
   
  if (buttons == BUTTON_DOWN && (timer[TmPos] - increment) >= 0) {timer[TmPos] -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {SpPos = TmPos; timer_menu();}
   
  if (buttons == BUTTON_FORWARD) {TmPos++; SpPos = TmPos + 1; update_text = 1; if (TmPos >  5) {TmPos = 0;}}

  if (update_text == 1)
  {

  tft.setTextColor(WHITE,BLUE);
  tft.setCursor(160,120);
  tft.print(F("Timer : "));tft.print(timer[TmPos]);tft.print(F(" Mins   "));tft.print(F(" TP : "));tft.print(TmPos);

  update_text = 0;

  }
       
  DoControl();

} // End of while loop

}

//==============================================//

void Tune_TimerMargin()
{

draw_keypad1();
tunetmmenu();
tft.fillRect(152,  2 , 326,  316, BLUE);

tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,130);
tft.print(F("Timer Margin : "));tft.print(TimerMargin);

while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {SaveParameters();} 

  if (buttons == BUTTON_3) {increment = 0.1; TimerMargin += increment; if (TimerMargin > 5) {TimerMargin = 5.0;} update_text = 1;}

  if (buttons == BUTTON_4) {increment = 0.1; TimerMargin -= increment; if (TimerMargin < 0) {TimerMargin = 0.0;} update_text = 1;}

  if (buttons == BUTTON_BACK) {timer_menu();}

  if (update_text == 1)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,130);
    tft.print(F("Timer Margin : "));tft.print(TimerMargin); 

    update_text = 0;

  }
      
   DoControl();

} // End of while loop
   
}

void SelectKettleSensor()
{
      
draw_keypad1();
Select_Sensor_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);   
NumSens = 2;   

while(true)
{

  ReadButtons();
      
  if (buttons == BUTTON_1) {IntKettleSensor = 1; StrKettleSensor = "1";}

  if (buttons == BUTTON_2) {IntKettleSensor = 2; StrKettleSensor = "2";}

  if (buttons == BUTTON_3)
  {
        
  sensors_on_off = 0;
      
  if (IntKettleSensor == 1)
  {
        
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,150);
    tft.print(F("Sensor 1 : "));tft.print(ratio0);tft.print(F(" C   "));tft.print(F(" On "));
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,160);
    tft.print(F("Sensor 2 : "));tft.print(ratio1);tft.print(F(" C   "));tft.print(F(" On "));

  }

  if (IntKettleSensor == 2)
  {      
      
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,150);
    tft.print(F("Sensor 1 : "));tft.print(ratio0);tft.print(F(" C   "));tft.print(F(" On "));
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,160);
    tft.print(F("Sensor 2 : "));tft.print(ratio1);tft.print(F(" C   "));tft.print(F(" On "));

  }}

  if (buttons == BUTTON_4)
  {
        
    sensors_on_off = 1;

    if (IntKettleSensor == 1)
    {
        
      tft.setTextColor(WHITE,BLUE);
      tft.setCursor(160,150);
      tft.print(F("Sensor 1 : "));tft.print(ratio0);tft.print(F(" C : "));tft.print(F(" Off"));
      tft.setTextColor(WHITE,BLUE);
      tft.setCursor(160,160);
      tft.print(F("Sensor 2 : "));tft.print(ratio1);tft.print(F(" C : "));tft.print(F(" Off"));

    }
      
    if (IntKettleSensor == 2)
    {      
      
      tft.setTextColor(WHITE,BLUE);
      tft.setCursor(160,150);
      tft.print(F("Sensor 1 : "));tft.print(ratio0);tft.print(F(" C : "));tft.print(F(" Off"));
      tft.setTextColor(WHITE,BLUE);
      tft.setCursor(160,160);
      tft.print(F("Sensor 2 : "));tft.print(ratio1);tft.print(F(" C : "));tft.print(F(" Off"));

      }}

      if (buttons == BUTTON_BACK) {timer_menu();}


      if (NumSens == 0)
      {

        tft.setTextColor(RED, BLACK); 
        tft.setCursor(2,20);                
        tft.print(F("      Sensor Error   "));
        
      }

      if (NumSens == 1)
      {

      float temp1 = ratio0; 

      tft.setTextColor(WHITE,BLUE);
      tft.setCursor(160,150);
      tft.print(F("Sensor 1 : "));tft.print(temp1);tft.print(F(" C   "));

      IntKettleSensor = 1;
      StrKettleSensor = "1";

      }

      if (NumSens >= 2)
      {

        float temp1 = ratio0;
        float temp2 = ratio1;

        if (IntKettleSensor == 1)
        {

        tft.setTextColor(BLUE,WHITE);
        tft.setCursor(160,150);
        tft.print(F("Sensor 1 : "));tft.print(temp1);tft.print(F(" C   "));
        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,160);
        tft.print(F("Sensor 2 : "));tft.print(temp2);tft.print(F(" C   "));
        }

        if (IntKettleSensor == 2)
        {
        tft.setTextColor(WHITE,BLUE);
        tft.setCursor(160,150);
        tft.print(F("Sensor 1 : "));tft.print(temp1);tft.print(F(" C   "));
        tft.setTextColor(BLUE,WHITE);
        tft.setCursor(160,160);
        tft.print(F("Sensor 2 : "));tft.print(temp2);tft.print(F(" C   "));
      }

  }
      
  DoControl();

} // End of while loop
   
}

//==============================================//

void Hop_Alarm()
{

tft.setTextSize(1);
      
if (hop[HopPos] == 0 && HopPos == 0) {hop[HopPos] = 0;}

if (hop[HopPos] == 0) {hop[HopPos] = hop[(HopPos-1)] - 1;}

draw_keypad1();
Hop_Timer_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);

tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,180);
tft.print(F("Hop timer : "));tft.print(HopPos);tft.print(F(" : "));tft.print(hop[HopPos]);tft.print(F(" : Mins"));

update_text = 0;
   
while(true)
{

  ReadButtons();

  if (buttons == BUTTON_1) {increment = 1;}
      
  if (buttons == BUTTON_2) {increment = 5;}

  if (buttons == BUTTON_3) {increment = 10;}
      
  if (buttons == BUTTON_4) {increment = 15;}

  if ((buttons == BUTTON_UP) && (hop[HopPos] +increment < hop[(HopPos-1)] ) && HopPos > 1 ) {hop[HopPos] += increment; update_text = 1;}

  if ((buttons == BUTTON_UP) && (HopPos == 1) && hop[HopPos] + increment <= BoilTimer ) {hop[HopPos] += increment; update_text = 1;}

  if ((buttons == BUTTON_DOWN) && (hop[HopPos] - increment >= 0)) {hop[HopPos] -= increment; update_text = 1;}

  if (buttons == BUTTON_BACK) {timer_menu();}

  if (buttons == BUTTON_FORWARD) {HopPos += 1; update_text = 1; }

  if (HopPos > 5) {HopPos = 0;}
   

  if (update_text == 1)
  {

    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,180);
    tft.print(F("Hop timer : "));tft.print(HopPos);tft.print(F(" : "));tft.print(hop[HopPos]);tft.print(F(" : Mins"));

    update_text = 0;

  }

  DoControl();

} // End of while loop

}

void SetAlarm()
{

tft.setTextSize(1);

draw_keypad1();
SensorAlarm_keys();
tft.fillRect(152,  2 , 326,  316, BLUE);

tft.setTextColor(WHITE,BLUE);
tft.setCursor(160,200);
tft.print(F("SSR Alarm Setpoint : "));tft.print(AlarmSetpoint);tft.print(F(" C   "));

update_text = 0;
   
while(true)
{

  ReadButtons();
      
  if (buttons == BUTTON_1) {increment = 1;}

  if (buttons == BUTTON_2 ) {increment = 5;} 
   
  if (buttons == BUTTON_3 ) {increment = 10;} 

  if (buttons == BUTTON_UP) {AlarmSetpoint += increment; update_text = 1;}
      
  if (buttons == BUTTON_DOWN) {AlarmSetpoint -= increment;update_text = 1;}

  if (buttons == BUTTON_BACK) {timer_menu();}

  if (update_text == 1)
  {
    
    tft.setTextColor(WHITE,BLUE);
    tft.setCursor(160,200);
    tft.print(F("SSR Alarm Setpoint : "));tft.print(AlarmSetpoint);tft.print(F(" C   ")); 

    update_text = 0;

  }

  DoControl();

} // End of while loop

}

void buzz(int buzzerPin, long frequency, long length, int repeats) 
{

for (int i = 0; i < repeats; ++i) 
{

  // calculate the delay value between transitions 
  // 1 second's worth of microseconds, 
  // divided by the frequency, 
  // then split in half since there are two phases to each cycle

  long delayValue = 1000000 / frequency / 2; 
  

  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce

  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing


  for (long i = 0; i < numCycles; i++) // for the calculated length of time...
  { 
    //digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
    //delayMicroseconds(delayValue); // wait for the calculated delay value
    //digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
    //delayMicroseconds(delayValue); // wait again or the calculated delay value
  }

}

}

void CheckAlarm()
{

tft.setTextSize(1);

float tTempInput;

if(IntKettleSensor == 1) {  tTempInput = ratio1; }

if(IntKettleSensor == 2) {  tTempInput = ratio0; }

int TempInput = tTempInput;
int ttAlarmSetpoint = AlarmSetpoint;

if (TempInput >= ttAlarmSetpoint)
{
  AlarmRaised = 1;

  digitalWrite(Relaypin, LOW);  // make sure it is off
      
  draw_keypad1();
  CheckAlarm_keys();
  tft.fillRect(152,  2 , 326,  316, BLUE);

  tft.setTextColor(RED, BLACK); 
  tft.setCursor(2, 20);                
  tft.print(F("      SSR Overheating"));

  opState = OFF;
   
  while(true)
  {     
 
    ReadButtons();
        
    buzz(buzzerPin, NOTE_C7, 300,1);

    if (buttons == BUTTON_BACK)
    {

      Off();

    }

    DoControl();

  } // End of while loop

}

}

//==============================================//
// Execute the control loop                     //
//==============================================//

void DoControl()
{

// Read the input:

TakeTemp();

if(IntKettleSensor == 1) { Input = ratio0; } 

if(IntKettleSensor == 2) { Input = ratio1; }  

// run the auto-tuner

if (tuning) {if (aTune.Runtime()) {FinishAutoTune();}} // returns 'true' when done

else {myPID.Compute();} // Execute control algorithm 
  
onTime = Output; // Time Proportional relay state is updated regularly via timer interrupt.

}

//==============================================//
// Called by ISR every 15ms to drive the output //
//==============================================//

void DriveOutput()
{  
  
long now = millis();

// Set the output
// "on time" is proportional to the PID output

if(now - windowStartTime>WindowSize) { windowStartTime += WindowSize;} //time to shift the Relay Window 

if (Setpoint[SpPos] != 100) { if((onTime > 0) && (onTime > (now - windowStartTime))) {digitalWrite(Relaypin,HIGH); } else {digitalWrite(Relaypin,LOW);}}

}

//==============================================//
// Save any parameter changes to EEPROM         //
//==============================================//

void SaveParameters()
{

tft.setTextSize(1);

if (AlarmSetpoint != EEPROM_readDouble(AlarmSpAddress)) {EEPROM_writeDouble(AlarmSpAddress, AlarmSetpoint);}

if (Kp != EEPROM_readDouble(KpAddress)) {EEPROM_writeDouble(KpAddress, Kp);}

if (Ki != EEPROM_readDouble(KiAddress)) {EEPROM_writeDouble(KiAddress, Ki);}

if (Kd != EEPROM_readDouble(KdAddress)) {EEPROM_writeDouble(KdAddress, Kd);}

if (TimerMargin != EEPROM_readDouble(TMarginAddress)) {EEPROM_writeDouble(TMarginAddress, TimerMargin);}

for (int i = 0; i < StrKettleSensor.length(); ++i) {EEPROM.write(41, StrKettleSensor[i]);}

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("     Settings saved"));

delay(3000);  // Splash screen

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("                   "));

}


void LoadParameters()
{
  
// ************************************************
// Load parameters from EEPROM
// ************************************************

AlarmSetpoint = EEPROM_readDouble(AlarmSpAddress);
Kp = EEPROM_readDouble(KpAddress);
Ki = EEPROM_readDouble(KiAddress);
Kd = EEPROM_readDouble(KdAddress);
TimerMargin = EEPROM_readDouble(TMarginAddress);

for (int i = 41; i < 42; ++i) {if (EEPROM.read(i) != 0) { StrKettleSensor += char(EEPROM.read(i));  }}

if(StrKettleSensor == "") {IntKettleSensor = 1;}

else {IntKettleSensor = StrKettleSensor.toInt();}   
   

// Use defaults if EEPROM values are invalid

if (isnan(AlarmSetpoint)) {AlarmSetpoint = 60;}
      
if (isnan(Kp)) {Kp = 850;}

if (isnan(Ki)) {Ki = 0.5;}
   
if (isnan(Kd)) {Kd = 0.1;}
     
if (isnan(TimerMargin)) {TimerMargin = 0.5;}  

}

//==============================================//
// Write floating point values to EEPROM        //
//==============================================//

void EEPROM_writeDouble(int address, double value)
{

byte* p = (byte*)(void*)&value;

for (int i = 0; i < sizeof(value); i++) {EEPROM.write(address++, *p++);}

}

//==============================================//
// Read floating point values from EEPROM       //
//==============================================//

double EEPROM_readDouble(int address)
{

double value = 0.0;
byte* p = (byte*)(void*)&value;

for (int i = 0; i < sizeof(value); i++) {*p++ = EEPROM.read(address++);}

return value;

}

void Reset()
{

tft.setTextSize(1);

stage = 0;
SpPos = 0;
TmPos = 0;    
HopPos = 0;    

for (int i = 0; i < 5; ++i) {Setpoint[i] = 0;}

for (int i = 0; i < 5; ++i) {timer[i] = 0;}

for (int i = 0; i < 5; ++i) {hop[i] = 0;}
    
tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("     Memory cleared  "));

buzz(buzzerPin, NOTE_C7, 100,2);

}

void drawmainscreen()
{

//==============================================//
// Draw Screen
//==============================================//

tft.fillScreen(WHITE);
tft.fillRect(  2,  2,  148,  40,   BLACK);
tft.fillRect(152,  2 , 326,  316, BLUE);

}

void draw_keypad1()
{

//==============================================//
//  Draw Keys                                   //
//==============================================//   

tft.setTextSize(1);
    
tft.fillRect(  2, 44,   73,   53,   BLACK);
tft.fillRect( 77, 44,   73,   53,  VIOLET);
tft.fillRect(  2, 99,   73,   53,   BLACK);
tft.fillRect( 77, 99,   73,   53,  VIOLET);
tft.fillRect(  2,154,   73,   53,   BLACK);
tft.fillRect( 77,154,   73,   53,  VIOLET);
tft.fillRect(  2,209,   73,   53,   BLACK);
tft.fillRect( 77,209,   73,   53,  VIOLET);
tft.fillRect(  2,264,   73,   53,   BLACK);
tft.fillRect( 77,264,   73,   53,  VIOLET);

}

void draw_keypad2() // Use this as the template for all menu additions 
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(5, 20);                
tft.print(F("    !!! Menu Description Goes Here !!!     "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(23, 215);
tft.print(F("1"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 215);
tft.print(F("2"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(23, 173);
tft.print(F("3"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 173);
tft.print(F("4"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(23, 131);
tft.print(F("5"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 131);
tft.print(F("6"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(23, 89);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 89);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(23, 68);
tft.print(F("Back"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));   

}

void off_keys()
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(5, 20);                
tft.print(F("       PID is Off    "));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(23, 288);
tft.print(F("Reboot"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 288);
tft.print(F("Logging"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));
    
}

void pid_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("       PID Settings  "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(34, 288);
tft.print(F("SP"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("D"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(36, 234);
tft.print(F("P"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 234);
tft.print(F("I"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 178);
tft.print(F("Run"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 172);
tft.print(F("Temp"));
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 182);
tft.print(F("Monitor"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(17, 123);
tft.print(F("Virtual"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(85, 123);
tft.print(F("Favourites"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));

}

void tunesp_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("         Tune SP      "));   
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 288);
tft.print(F("0.01"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(112, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(35, 232);
tft.print(F("5"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(108, 232);
tft.print(F("10"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(32, 168);
tft.print(F("Set"));
tft.setTextColor(WHITE, BLACK);
tft.setCursor(29, 178);
tft.print(F("Mash"));
tft.setTextColor(WHITE, BLACK);
tft.setCursor(29, 188);
tft.print(F("Temp"));    

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(108, 170);
tft.print(F("SP"));
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(90, 180);
tft.print(F("Position"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 124);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(104, 124);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
    
}

void tunep_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                  
tft.print(F("         Tune P    ")); 
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(29, 288);
tft.print(F("0.01"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(32, 234);
tft.print(F("10"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(105, 234);
tft.print(F("100"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 178);
tft.print(F("500"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(102, 178);
tft.print(F("1000"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 120);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 120);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
  
}

void tunei_keys()

{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                  
tft.print(F("          Tune I    ")); 
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 288);
tft.print(F("0.01"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(32, 230);
tft.print(F("10"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 230);
tft.print(F("100"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 176);
tft.print(F("500"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(101, 176);
tft.print(F("1000"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 120);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 120);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
     
}

void tuned_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                  
tft.print(F("         Tune D    "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(29, 288);
tft.print(F("0.01"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 230);
tft.print(F("10"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 230);
tft.print(F("100"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 175);
tft.print(F("500"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(102, 175);
tft.print(F("1000"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 125);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(100, 125);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
     
}

void run_keys()
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("     Run/Auto Tune "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 288);
tft.print(F("Run"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(98, 288);
tft.print(F("A-Tune"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(23, 233);
tft.print(F("Graph"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 178);
tft.print(F("0.01"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 178);
tft.print(F("1"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 122);
tft.print(F("SP+"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 122);
tft.print(F("SP-"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));

}

void simulation_keys()
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("     Sim Set On/Off "));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(17, 288);
tft.print(F("Heating"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 288);
tft.print(F("Cooling"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(28, 232);
tft.print(F("Mins"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 232);
tft.print(F("Tune"));    
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 178);
tft.print(F("On"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(105, 178);
tft.print(F("Off"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));
 
}

void time_frame_keys()
{
  
tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);
tft.print(F("        Time Frame    "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(19, 288);
tft.print(F("Normal"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(93, 288);
tft.print(F("30 Mins"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(21, 232);
tft.print(F("1 Hour"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(91, 232);
tft.print(F("12 Hours"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(16, 178);
tft.print(F("24 Hours"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(96, 178);
tft.print(F("1 Week"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(19, 122);
tft.print(F("2 Weeks"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(93, 122);
tft.print(F("1 Month"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(28, 68);
tft.print(F("Back"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 68);
tft.print(F("Forward"));
  
}

void Favourite_keys()
{
  
tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                  
tft.print(F("        Favourites    "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 288);
tft.print(F("63C"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(107, 288);
tft.print(F("64C"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 232);
tft.print(F("65C"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(107, 232);
tft.print(F("66C"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 178);
tft.print(F("67C"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(107, 178);
tft.print(F("68C"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 122);
tft.print(F("74C"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(102, 122);
tft.print(F("100C"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(28, 68);
tft.print(F("Back"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 68);
tft.print(F("Forward"));

}

void simheating_keys()
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("         Heating    "));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 288);
tft.print(F("0.01"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(108, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(32, 232);
tft.print(F("10"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(25, 178);
tft.print(F("Begin"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 120);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(102, 120);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));

}

void simcooling_keys()
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("         Cooling      "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 288);
tft.print(F("0.01"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 232);
tft.print(F("10"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 122);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(104, 122);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));

}

void simtime_keys()
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("         Minutes      "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 288);
tft.print(F("0.01"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 232);
tft.print(F("10"));
                    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 122);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 122);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));

}

void simtune_keys()
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);
tft.print(F("       Fine Tune   "));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(30, 288);
tft.print(F("0.01"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(105, 288);
tft.print(F("0.1"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(35, 232);
tft.print(F("1"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(108, 232);
tft.print(F("10"));
                    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 122);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 122);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));

}

void init_temp_keys()
{
  
tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                  
tft.print(F("        Start Temp    "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 288);
tft.print(F("20C"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(107, 288);
tft.print(F("30C"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 232);
tft.print(F("40C"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(107, 232);
tft.print(F("50C"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 178);
tft.print(F("60C"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(107, 178);
tft.print(F("70C"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 122);
tft.print(F("80C"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(107, 122);
tft.print(F("90C"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(28, 68);
tft.print(F("Back"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(95, 68);
tft.print(F("Forward"));

}

void Log_keys()
{

tft.setTextSize(1);

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("         Logging     "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 288);
tft.print(F("SD On"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(98, 288);
tft.print(F("SD Off"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 232);
tft.print(F("BT On"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(98, 232);
tft.print(F("BT Off"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));

}

void timers_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("        Timers Etc    "));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(24, 286);
tft.print(F("Timer"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(102, 282);
tft.print(F("Time"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(96, 292);
tft.print(F("Margin"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(18, 232);
tft.print(F("Sensors"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(104, 228);
tft.print(F("Hop"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(89, 238);
tft.print(F("Schedule"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(29, 167);
tft.print(F("SSR"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(24, 177);
tft.print(F("Alarm"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(24 , 187);
tft.print(F("Level"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 67);
tft.print(F("Forward"));
     
}

void time_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("       Stage Timers  "));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(35, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("5"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 234);
tft.print(F("10"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(108, 234);
tft.print(F("15"));
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 178);
tft.print(F("30"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(108, 178);
tft.print(F("60"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 122);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 122);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(100, 68);
tft.print(F("TmPos"));    
      
}

void tunetmmenu()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);
tft.print(F("       Timer Margin  "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(28, 234);
tft.print(F("+0.1"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(100, 234);
tft.print(F("-0.1"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(28, 288);
tft.print(F("Save"));   
    
}

void Select_Sensor_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);                
tft.print(F("      Select Sensor  "));
        
tft.setTextColor(WHITE, BLACK);
tft.setCursor(14, 288);
tft.print(F("Sensor 1"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(89, 288);
tft.print(F("Sensor 2"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(32, 230);
tft.print(F("On"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(105, 230);
tft.print(F("Off"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
 
}

void SensorAlarm_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);
tft.print(F("      Sensor Alarm   "));    
    
tft.setTextColor(WHITE, BLACK);
tft.setCursor(35, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("5"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 230);
tft.print(F("10"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 122);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(100, 122);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));    
  
}

void Hop_Timer_keys()
{

tft.setTextColor(WHITE, BLACK); 
tft.setCursor(2, 20);
tft.print(F("        Hop Timer    "));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(35, 288);
tft.print(F("1"));   

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(110, 288);
tft.print(F("5"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(31, 232);
tft.print(F("10"));
                    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(105, 232);
tft.print(F("15"));   

tft.setTextColor(WHITE, BLACK);
tft.setCursor(33, 122);
tft.print(F("Up"));
    
tft.setTextColor(WHITE, VIOLET);
tft.setCursor(103, 122);
tft.print(F("Down"));

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));

tft.setTextColor(WHITE, VIOLET);
tft.setCursor(95, 68);
tft.print(F("HopPos +"));
  
}

void CheckAlarm_keys()
{

tft.setTextColor(WHITE, BLACK);
tft.setCursor(26, 68);
tft.print(F("Back"));
  
}

//==============================================//
// Set Backlight based on the state of control  //
//==============================================//

void setBacklight()
{

if (tuning)
{
  
  tft.drawRect(462,302,13,13,YELLOW);} 
  
  else if (abs(Input - Setpoint[SpPos]) > 1.0) {tft.drawRect(462,302,13,13,RED);} // High Alarm - off by more than 1 degree 
   
  else if (abs(Input - Setpoint[SpPos]) > 0.2) {tft.drawRect(462,302,13,13,BLACK);}  // Low Alarm - off by more than 0.2 degrees
   
  else{tft.drawRect(462,302,13,13,BLUE);

}
   
}
   
//==============================================//
// Read The Max31865 Boards                     //
//==============================================//

void TakeTemp()
{
 
uint16_t rtd0 = max0.readRTD();
uint16_t rtd1 = max1.readRTD();

ratio0 = rtd0;
ratio1 = rtd1;

ratio0 /=32768;
ratio1 /=32768;

ratio0 *= RREF;
ratio1 *= RREF;

ratio0 -= 100;
ratio1 -= 100;

ratio0 /= 0.3851;
ratio1 /= 0.3851;

if (opState == RUN && sensors_on_off == 1 && sim == 1 && IntKettleSensor == 1) {ratio0 = starttemp;}

if (opState == RUN && sensors_on_off == 1 && sim == 1 && IntKettleSensor == 2) {ratio1 = starttemp;}

if (ratio0 < -10) {ratio0 = starttemp;}

if (ratio1 < -10) {ratio1 = starttemp;}

if (ratio0 > 100) {ratio0 = 100;}

if (ratio1 > 100) {ratio1 = 100;}

if (opState == RUN && sim == 1 && sensors_on_off == 1)
{

	if (Output >= WindowSize && tuning != true) {Output = WindowSize;}

	if (Output <= 0 && tuning != true) { Output = 0;}


	if (IntKettleSensor == 1)
	{

    if (graph == 0) {ramp += ((heatingrate*(Output/WindowSize))/tune); 
    
    if(Output <= 10) {ramp -= coolingrate/tune;}}

		if (graph == 1) {ramp += ((heatingrate*(Output/WindowSize))/tune); 
		
		if(Output <= 10) {ramp -= coolingrate/tune;}} ratio0 += ramp;

	}

	if (IntKettleSensor == 2)
	{

    if (graph == 0) {ramp += ((heatingrate*(Output/WindowSize))/tune); 
    
    if(Output <= 10) {ramp -= coolingrate/tune;}}

		if (graph == 1)	{ramp += ((heatingrate*(Output/WindowSize))/tune); 
		
		if(Output <= 10) {ramp -= coolingrate/tune;}} ratio1 += ramp;

	}

}

if (ratio0 < mnf0) {mnf0 = ratio0;}

if (ratio1 < mnf1) {mnf1 = ratio1;}

if (ratio0 > mxf0) {mxf0 = ratio0;}

if (ratio1 > mxf1) {mxf1 = ratio1;}

ave0 = ((mnf0 + mxf0 + ratio0)/3);
ave1 = ((mnf1 + mxf1 + ratio1)/3);

/*
 
if (ElapsedMin == 85) { ramp -= 0.025510204; reduction1++; }
if (ElapsedMin == 80) { ramp -= 0.012755102; reduction2++; }
if (ElapsedMin == 75) { ramp -= 0.005154639; reduction3++; }

*/

}

void  ReadButtons()
{

buttons = BUTTON_NONE;   // default return

tp = ts.getPoint();   //tp.x, tp.y are ADC values

pinMode(XM, OUTPUT);
pinMode(YP, OUTPUT);
pinMode(XP, OUTPUT);
pinMode(YM, OUTPUT);
    
if (tp.z > MINPRESSURE && tp.z < MAXPRESSURE) 
{

  xpos = map(tp.x, TS_LEFT, TS_RT, 0, tft.width());
  ypos = map(tp.y, TS_TOP, TS_BOT, 0, tft.height());

  if (xpos >= 0 && xpos <= 76  && ypos >= 0 && ypos <= 42) {buttons = BUTTON_1;}
  
  if (xpos >= 77 && xpos <= 152  && ypos >= 0 && ypos <= 42) {buttons = BUTTON_2;}
  
  if (xpos >= 0 && xpos <= 76  && ypos >= 53 && ypos <= 108) {buttons = BUTTON_3;}
  
  if (xpos >=77 && xpos <= 152  && ypos >= 53 && ypos <= 108) {buttons = BUTTON_4;}

  if (xpos >= 0 && xpos <= 76 && ypos >= 109 && ypos <= 164) {buttons = BUTTON_5;}

  if (xpos >= 77 && xpos <= 152  && ypos >= 109 && ypos <= 164) {buttons = BUTTON_6;}

  if (xpos >= 0 && xpos <= 76 && ypos >= 165 && ypos <= 220) {buttons = BUTTON_UP;}

  if (xpos >= 77 && xpos <= 152  && ypos >= 165 && ypos <= 220) {buttons = BUTTON_DOWN;}

  if (xpos >= 0 && xpos <= 76 && ypos >= 221 && ypos <= 276) {buttons = BUTTON_BACK;}

  if (xpos >= 77 && xpos <= 152  && ypos >= 221 && ypos <= 276) {buttons = BUTTON_FORWARD;}

  if (xpos >= 405 && xpos <= 479  && ypos >= 0 && ypos <= 42) {buttons = GETMEOUTOFHERE;}

  /*  

  tft.setTextColor(WHITE, BLUE);
  tft.setCursor(350, 290);
  tft.print(F("Opstate : "));tft.print(opState);tft.print(F(" x:"));tft.print(xpos);tft.print(F("  "));

  tft.setTextColor(WHITE, BLUE);
    tft.setCursor(350, 300);
tft.print(F("Buttons : "));tft.print(buttons);tft.print(F(" y:"));tft.print(ypos);tft.print(F("  "));        

  */

  }

}


void sdcard_sub()
{

Serial.print(ratio0);
Serial.print(F(","));
Serial.print(ratio1);
Serial.print(F(","));
Serial.print(Setpoint[SpPos]);
Serial.print(F(","));
Serial.print(Input);
Serial.print(F(","));
Serial.print(Output);
Serial.print(F(","));
Serial.print((Output/WindowSize)*100);
Serial.print(F(","));
Serial.print(ElapsedMin);
Serial.print(F(","));
Serial.print(timeElapsed);
Serial.print(F(","));
Serial.print(Kp);
Serial.print(F(","));
Serial.print(Ki);
Serial.print(F(","));
Serial.print(Kd);
Serial.print(F(","));
Serial.print(looptime);
Serial.println(F(",EOL"));
looptime = 0;

/*

Serial.print(F(","));
Serial.print(reduction1);
Serial.print(F(","));
Serial.print(reduction2);
Serial.print(F(","));
Serial.print(reduction3);
Serial.println(F(",EOL"));

*/

setBacklight();

}
