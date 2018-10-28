//SCULLCOM HOBBY ELECTRONICS
//ELECTRONIC DC LOAD PROJECT
//Software Version 35 (4x5 Matrix Keypad Version - modified key layout)
//14th April 2018
//modified by bwack, 2018-10-17 no-keypad version. load key with indicator

#include <SPI.h>                              //include SPI library (Serial Peripheral Interface)
#include <Wire.h>                             //include I2C library
#include <LiquidCrystal_I2C.h>                // F Malpartida's NewLiquidCrystal library
                                              // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/NewliquidCrystal_1.3.4.zip
#include <math.h>                             //
#include <Adafruit_MCP4725.h>                 //Adafruit DAC library  https://github.com/adafruit/Adafruit_MCP4725
#include <MCP342x.h>                          //Steve Marple library avaiable from    https://github.com/stevemarple/MCP342x
#include <MCP79410_Timer.h>                   //Scullcom Hobby Electronics library  http://www.scullcom.com/MCP79410Timer-master.zip
#include <EEPROM.h>                           //include EEPROM library used for storing setup data

//#include <Keypad.h>                           //http://playground.arduino.cc/Code/Keypad

// program defines
//#define WELCOMESCREEN 1
//#define SETUPLIMITSSCREEN 1
//#define TEMPERATURE_CUTOFF 1
#define POWERLEVEL_CUTOFF 1
#define CURRENTLEVEL_CUTOFF 1
//#define ADC_DEBUG 1
//#define INPUTDEBUG 1

char decimalPoint;                            //used to test for more than one press of * key (decimal point)

Adafruit_MCP4725 dac;                         //constructor

uint8_t address = 0x68;                       //0x68 is the default address for the MCP3426 device
MCP342x adc = MCP342x(address);
MCP342x::Config configVolts(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain1);
MCP342x::Config configAmps (MCP342x::channel2, MCP342x::oneShot, MCP342x::resolution16, MCP342x::gain4);

const byte MCP79410_ADDRESS = 0x6f;           //0x6f is the default address for the MCP79410 Real Time Clock IC
MCP79410_Timer timer = MCP79410_Timer(MCP79410_ADDRESS);

//Set the pins on the I2C chip used for LCD connections
//ADDR,EN,R/W,RS,D4,D5,D6,D7
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);    //0x27 is the default address of the LCD with I2C bus module

#define ENCODER_LEFT  1
#define ENCODER_RIGHT 2
#define ENCODER_FAST  4
const byte pinA = 2;                          //digital pin (also interrupt pin) for the A pin of the Rotary Encoder (changed to digital pin 2)
const byte pinB = 4;                          //digital pin for the B pin of the Rotary Encoder
const byte EncButton = 17;

const byte CursorPos = 17;                    //analog pin A3 used as a digital pin to set cursor position (rotary encoder push button)
const byte LoadOnOff = 15;                    //analog pin A1 used as a digital pin to set Load ON/OFF
const byte LoadLED = 5;

const byte TriggerPulse = 16;                 //analog pin A2 used as a digital pin for trigger pulse input in transient mode

const byte fan = 3;                           //digital pin 3 for fan control output (changed to Digital pin 3)
const byte temperature = A6;                  //analog pin used for temperature output from LM35 (was A0 previously but changed)
int temp;                                     //

float BatteryLife = 0;                        //
float BatteryLifePrevious = 0;                //
float Seconds = 0;                            //time variable used in Battery Capacity Mode (BC)
float SecondsLog = 0;                         //variable used for data logging of the time in seconds
float BatteryCutoffVolts;                     //used to set battery discharge cut-off voltage
float MaxBatteryCurrent = 1.0;                //maximum battery current allowed for Battery Capacity Testing

int stopSeconds;                              //store for seconds when timer stopped

int CP = 8;                                   //cursor start position

#define SHORT_PRESS 1
#define LONG_PRESS 2
boolean toggle = false;                       //used for toggle of Load On/Off button

unsigned long controlVoltage = 0;             //used for DAC to control MOSFET

long adc_current = 0;                         //variable used by ADC for measuring the current
long adc_voltage = 0;                         //variable used by ADC for measuring the voltage

int setCurrent = 0;                         //variable used for the set current of the load
float setPower = 0;                           //variable used for the set power of the load
float setResistance = 0;                      //variable used for the set resistance of the load
float setCurrentCalibrationFactor = 0.980;    //calibration adjustment - set as required if needed (was 1.000)

float displayCurrentCal = 0.000;              //calibration correction for LCD current display (was 0.040)
int Load = 0;                                 //Load On/Off flag

float setControlCurrent = 0;                  //variable used to set the temporary store for control current required

int VoltsDecimalPlaces = 3;                   //number of decimal places used for Voltage display on LCD

float ActualVoltage = 0;                      //variable used for Actual Voltage reading of Load
float ActualCurrent = 0;                      //variable used for Actual Current reading of Load
float ActualPower = 0;                        //variable used for Actual Power reading of Load

float ResistorCutOff = 999;                   //maximum Resistor we want to deal with in software
float BatteryCurrent;                         //
float LoadCurrent;                            //

#define CURRENTCUTOFF_MAXLIMIT 5
#define POWERCUTOFF_MAXLIMIT 120
#define TEMPCUTOFF_MAXLIMIT 90
int CurrentCutOff = EEPROM.read(0x00);
int PowerCutOff = EEPROM.read(0x20);
int tempCutOff = EEPROM.read(0x40);

int setReading = 0;                           //

int ControlVolts = 0;                         //used to set output current
float OutputVoltage = 0;                      //

uint8_t menu;
uint8_t mode;
uint8_t option;
enum {
  CC_MODE, CV_MODE, CR_MODE, CP_MODE,
  TC_MODE, TT_MODE, TP_MODE, TL_MODE,
  BC_MODE
};
enum { // options for the main menu
  CC_OPTION, CV_OPTION, CR_OPTION, CP_OPTION,
  TRANSIENT_OPTION, BATCAP_OPTION
};
enum { // options for the transient menu
  TC_OPTION, TT_OPTION, TP_OPTION, TL_OPTION, EXIT_OPTION,
};
enum { // screen
  NO_MENU, MAIN_MENU, TRANSIENT_MENU, BATCAP_MENU
};
#define MAX_OPTIONS 6

struct menuoption {
  uint8_t menu;
  uint8_t option;
  uint8_t mode;
  char text[11];
  uint8_t x, y;
  uint8_t nextoptidx;
  uint8_t nextmenu;
};

struct menuoption menubuf; // buffer for options and transoptions

const struct menuoption PROGMEM options[] = {
  //  menu     menu option       mode     text    x  y nextoptidx(shortpress)  nextmenu(longpress)
  { MAIN_MENU, CC_OPTION,        CC_MODE, "CC",        3, 2, 1, NO_MENU },
  { MAIN_MENU, CV_OPTION,        CV_MODE, "CV",        7, 2, 2, NO_MENU },
  { MAIN_MENU, CR_OPTION,        CR_MODE, "CR",       11, 2, 3, NO_MENU },
  { MAIN_MENU, CP_OPTION,        CP_MODE, "CP",       15, 2, 4, NO_MENU },
  { MAIN_MENU, TRANSIENT_OPTION, TP_MODE, "TRANSIENT", 1, 3, 5, TRANSIENT_MENU },
  { MAIN_MENU, BATCAP_OPTION,    BC_MODE, "BATCAP",   12, 3, 0, NO_MENU },
  { TRANSIENT_MENU, TC_OPTION,   TC_MODE, "Continuous",3, 1, 7, NO_MENU },
  { TRANSIENT_MENU, TT_OPTION,   TT_MODE, "Toggle"   , 1, 2, 8, NO_MENU },
  { TRANSIENT_MENU, TP_OPTION,   TP_MODE, "Pulse"    ,11, 2, 9, NO_MENU },
  { TRANSIENT_MENU, TL_OPTION,   TL_MODE, "List"     , 1, 3,10, NO_MENU },
  { TRANSIENT_MENU, EXIT_OPTION, CC_MODE, "Exit"     ,11, 3, 6, MAIN_MENU }
};

int modeSelected = 0;                         //Mode status flag

int lastCount = 50;                           //
volatile int encoderPosition=0;
volatile int factor=1;
volatile int encoderMax = 255;   //sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)
int reading = 0;                            //variable for Rotary Encoder value divided by 1000
int setpoint=0;
int poschange=0;
boolean coarse_flag=true;

float LiPoCutOffVoltage = 3.0;                //set cutoff voltage for LiPo battery
float LiFeCutOffVoltage = 2.8;                //set cutoff voltage for LiFe battery
float NiCdCutOffVoltage = 1.0;                //set cutoff voltage for NiCd battery
float ZiZnCutOffVoltage = 1.0;                //set cutoff voltage for ZiZn battery
float PbAcCutOffVoltage = 1.75;               //set cutoff voltage for PbAc battery
String BatteryType ="    ";

byte exitMode = 0;                           //used to exit battery selection menu and return to CC Mode

char numbers[10];                            //keypad number entry - Plenty to store a representation of a float
byte index = 0;
int z = 1;                                   //was previously 0
float x = 0;
int y = 0;
int r = 0;

float LowCurrent = 0;               //the low current setting for transcient mode
float HighCurrent = 0;              //the high current setting for transcient mode
unsigned long transientPeriod;      //used to store pulse time period in transcient pulse mode
unsigned long current_time;         //used to store the current time in microseconds
unsigned long last_time;            //used to store the time of the last transient switch in micro seconds
boolean transient_mode_status;      //used to maintain the state of the trascient mode (false = low current, true = high current)

int transientList [10][2];        //array to store Transient List data
int total_instructions;             //used in Transient List Mode
int current_instruction;            //used in Transient List Mode

#define CONVSTATE_CONVERTVOLTS 0
#define CONVSTATE_READVOLTS    1
#define CONVSTATE_CONVERTAMPS  2
#define CONVSTATE_READAMPS     3
int convstate, nextconvstate;

int loopcount=0;
unsigned long currentMillis;
unsigned long lastMillis;
boolean modeselect;
boolean mainmenu_flag;
boolean transientmenu_flag;

//--------------------------------Interrupt Routine for Rotary Encoder------------------------
void isr()
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 10) {
    lastInterruptTime = interruptTime;
    delay(5);
    int a = digitalRead(pinA);
    int b = digitalRead(pinB);
    if (a==0) {
      if(b==1)
        encoderPosition++;
      else
        encoderPosition--;
    }
  }
}
//---------------------------------Initial Set up---------------------------------------
void setup() {
  Serial.begin(19200);                                      //used for testing only
 
  Wire.begin();                                            //join i2c bus (address optional for master)
  Wire.setClock(400000L);                                  //sets bit rate to 400KHz

  pinMode (pinA, INPUT);
  pinMode (pinB, INPUT);
  pinMode (CursorPos, INPUT_PULLUP);
  pinMode (LoadOnOff, INPUT_PULLUP);
  pinMode (LoadLED, OUTPUT);
  pinMode (TriggerPulse, INPUT_PULLUP);
  pinMode (fan, OUTPUT);
  TCCR2B = (TCCR2B & 0b11111000) | 1;                      //change PWM to above hearing (Kenneth Larvsen recommendation)
  pinMode (temperature, INPUT);
  analogReference(INTERNAL);                               //use Arduino internal reference for tempurature monitoring
  attachInterrupt(digitalPinToInterrupt(pinA), isr, FALLING);

  // ADC
  MCP342x::generalCallReset();                             //Reset devices
  delay(1);                                                //MC342x needs 300us to settle
  check_adc_presence();

  // DAC
  dac.begin(0x61);                                         //the DAC I2C address with MCP4725 pin A0 set high
  dac.setVoltage(0,false);                                 //reset DAC to zero for no output current set at Switch On

  lcd.begin(20, 4);                                        //set up the LCD's number of columns and rows 
  lcd.setBacklightPin(3,POSITIVE);                         // BL, BL_POL
  lcd.setBacklight(HIGH);                                  //set LCD backlight on

  WelcomeScreen();
  
  get_eeprom_setup_limits();
  SetupLimitsScreen();

  lcd.clear();

  last_time = 0;                                           //set the last_time to 0 at the start (Transicent Mode)
  transient_mode_status = false;                           //set the initial transient mode status (false = low, true = high);
  setCurrent = LowCurrent;                                 //first set the current to the low current value (Transicent Mode)

  printLoadOff();
  Current();                                               //sets initial mode to be CC (Constant Current) at Power Up

  //customKey = customKeypad.getKey();

  nextconvstate = CONVSTATE_CONVERTVOLTS; // initialize read state machine
  mode = CC_MODE;
  option = CC_OPTION;
  modeselect=false;
}

int doprint;
//------------------------------------------Main Program Loop---------------------------------
void loop()
{
  readVoltageCurrent();                                  //routine for ADC's to read actual Voltage and Current
  ActualReading();                                       //Display actual Voltage, Current readings and Actual Wattage
  dacControl();
  dacControlVoltage();                                   //sets the drive voltage to control the MOSFET
  fanControl();                                          //call heatsink fan control

  lastMillis = currentMillis;
  currentMillis = millis();

  doprint=0;
  loopcount++;
  if (loopcount>=300) {
    loopcount=0;
	  doprint=1;
  }

  if (CurrentCutOff > CURRENTCUTOFF_MAXLIMIT) {          //Test and go to user set limits if required
    userSetUp();
  } else {

    currentMillis = millis();
    uint8_t loadbutton = LoadButton();
    uint8_t encbutton = EncoderButton();
    uint8_t encodervelocity = EncoderVelocity();

    #ifdef INPUTDEBUG
    Serial.print("loadbutt ");
    Serial.print(loadbutton);
    Serial.print("encbutt ");
    Serial.print(encbutton);
    Serial.print("encrot ");
    Serial.println(encodervelocity);
    #endif

    if (menu != NO_MENU) {
      if (loadbutton==SHORT_PRESS) {
        nextOption();
        //printOptions();
      }
      else if (loadbutton==LONG_PRESS) {
        nextMenu(); // next menu by option
        setFirstOption();
        printMenu();
        printOptions();
        setMode();
      }
    }
    else { // NO_MENU

      switch(mode) {

        case CC_MODE: case CV_MODE: case CR_MODE: case CP_MODE:
          normalWork(loadbutton, encbutton, encodervelocity);
          if (doprint) printSetPoint(setpoint);
          break;

        case TT_MODE: case TL_MODE: case TC_MODE: case TP_MODE:
          transientLoadToggle(loadbutton, encbutton);
          if (doprint) printSetPoint(setCurrent);
          break;

        case BC_MODE:
          batteryCapacity(loadbutton, encbutton);
          break;

        default:
          break;
      }
      //transient();                                           //test for Transient Mode

      if (doprint) printMode();
      if (loopcount==100) printActualReading();
      if (loopcount==200) printTemp(temp);
    }


  }
  //while(millis()-lastMillis<5) {
  //  delay(2);
  //}
  //if (millis()-lastMillis > 12) { 
  //  Serial.print("Time ");
  //  Serial.println(millis() - lastMillis);
  //}
}

void normalWork(uint8_t loadbutton, uint8_t encbutton, uint8_t velo) {
  if (encbutton == LONG_PRESS) {
    userSetUp();
    return;
  } else if (encbutton == SHORT_PRESS) {
    coarse_flag = !coarse_flag;
    return;
  } 
  if (loadbutton == SHORT_PRESS) {
    LoadSwitch();
    return;
  }
  else if (loadbutton==LONG_PRESS) {
    LoadOff();
    menu=MAIN_MENU;
    setFirstOption();
    printMenu();
    printOptions();
    return;
  }
  maxConstantCurrentSetting();                           //set maxiumum Current allowed in Constant Current Mode (CC)
  powerLevelCutOff();                                    //Check if Power Limit has been exceeded
  temperatureCutOff();                                   //check if Maximum Temperature is exceeded
  //batteryCurrentLimitValue();                            //Battery Discharge Constant Current Limit Value in BC Mode
  if (velo) {
    if(velo) {
      if (coarse_flag) {
        setpoint+=poschange*100;
      } else {
        setpoint+=poschange*5;
      }

      if (setpoint>99999) setpoint=99999;
      else if (setpoint<0) setpoint=0;
      printSetPoint(setpoint);
    }
  }
 
  lastCount = encoderPosition;                           //store rotary encoder current position
  CursorPosition();                                      //check and change the cursor position if cursor button pressed
  lcd.noCursor();
}

// user input functions
// --------------------------------------------------------------------------------------
uint8_t LoadButton(void) {
  static unsigned long pressedMillis =0 ;
  static int button_sampled=HIGH;
  static int button_sampled_old=HIGH;
  static int oldbutton=HIGH;
  int button = digitalRead(LoadOnOff);
  int pressed_event=0;
  currentMillis=millis();
  
  if(pressedMillis==0) {
    if (button != oldbutton) { // change! start timer
      pressedMillis=currentMillis;
    }
  } else if (currentMillis - pressedMillis > 20) {
    button_sampled = button; // sample after timeout
    if ( button_sampled_old==LOW && button_sampled == HIGH) {
      if (currentMillis - pressedMillis < 500) {
        pressed_event=SHORT_PRESS;
        pressedMillis=0;
      } else {
        pressed_event=LONG_PRESS;
        //Serial.print("currentMillis - pressedMillis = ");
        //Serial.println(currentMillis-pressedMillis);
        pressedMillis=0;
      }
    }
    button_sampled_old = button_sampled;
  }
  oldbutton = button;
  return pressed_event;
}

uint8_t EncoderButton(void) {
  static unsigned long pressedMillis =0 ;
  static int button_sampled=HIGH;
  static int button_sampled_old=HIGH;
  static int oldbutton=HIGH;
  int button = digitalRead(EncButton);
  int pressed_event=0;
  currentMillis=millis();

  if(pressedMillis==0) {
    if (button != oldbutton) { // change! start timer
      pressedMillis=currentMillis;
    }
  } else if (currentMillis - pressedMillis > 20) { // 20ms debounce
    button_sampled = button; // sample after timeout
    if ( button_sampled_old==LOW && button_sampled == HIGH) {
      if (currentMillis - pressedMillis < 500) {
        pressed_event=SHORT_PRESS;
        pressedMillis=0;
      } else {
        pressed_event=LONG_PRESS;
        //Serial.print("currentMillis - pressedMillis = ");
        //Serial.println(currentMillis-pressedMillis);
        pressedMillis=0;
      }
    }
    button_sampled_old = button_sampled;
  }
  oldbutton = button;
  return pressed_event;
}

int EncoderVelocity(void) {
  static unsigned long startMillis=0;
  static uint8_t oldpos=0;
  uint8_t pos = encoderPosition;
  int returnval=0;
  currentMillis=millis();

  if (pos < 10 && oldpos >250) oldpos=0;
  else if (oldpos < 10 && pos >250) oldpos=255;
  
  if (startMillis==0) {
    if (pos!=oldpos) {
      startMillis = currentMillis;
      oldpos = pos;
    }
  } else if (currentMillis-startMillis > 200) {
    if (pos>oldpos) returnval = ENCODER_RIGHT;
    else if (pos<oldpos) returnval = ENCODER_LEFT;
    else returnval=0;
    if (pos-oldpos> 3 || pos-oldpos< -3) {
      returnval |= ENCODER_FAST;
      //Serial.print("FAST");
    }
    startMillis=currentMillis;
    poschange = (pos - oldpos);
    oldpos = pos;
  }
  return returnval;
}

// -------------------- Print Functions ---------------------------

void WelcomeScreen() {
  #ifdef WELCOMESCREEN
  lcd.clear();                                             //clear LCD display
  lcd.setCursor(6,0);                                      //set LCD cursor to column 0, row 4
  lcd.print("SCULLCOM");                                   //print SCULLCOM to display with 5 leading spaces (you can change to your own)
  lcd.setCursor(1,1);                                      //set LCD cursor to column 0, row 1 (start of second line)
  lcd.print("Hobby Electronics");                          //print Hobby Electronics to display (you can change to your own)
  lcd.setCursor(1,2);
  lcd.print("DC Electronic Load"); //
  lcd.setCursor(0,3);
  lcd.print("Ver. 35 (5x4 Keypad)"); //
  delay(2000);
  #endif
}

void printValue(int value, uint8_t decimals, uint8_t decimalpoint, uint8_t xpos, uint8_t ypos) {
  uint8_t offset=0;
  uint8_t maxlen;
  char buf[9];
  sprintf(buf,"%7d",value);

  // align field by number of decimal
  offset = 7-decimals;
  for(int i=0; i<decimals; i++)
    buf[i]=buf[i+offset];
  buf[decimals]=0;    

  // pad leading zeroes
  for(int i=0; i<decimals; i++) {
    if(buf[i]==' ' && i>=(decimals-decimalpoint-1)) buf[i]='0';
  }

  // add the dot
  if(decimalpoint>0) {
    for(int i=decimals; i>=decimals-decimalpoint; i--)   
      buf[i+1]=buf[i];
    buf[decimals-decimalpoint]='.';
  }
  
  //Serial.print("printValue buf:[");
  //Serial.print(buf);
  //Serial.println("]");
  //if( value<10 ) offset=2;
  //else if( value<100) offset=1;
  lcd.setCursor(xpos,ypos);
  lcd.print(buf);
}

void printERROR_NoADC(void) {
  lcd.clear();
  lcd.setCursor(6,0);
  lcd.print("ERROR");
  lcd.setCursor(1,2);
  lcd.print("ADC not present");
}

void printMode(void) {
  lcd.setCursor(18,3);                                   //sets display of Mode indicator at bottom right of LCD
  if(mode==CP_MODE)
    lcd.print("CP");                                       //display mode selected on LCD (CC, CP, CR or BC)
  else if (mode==CC_MODE)
    lcd.print("CC");
  else if (mode==CV_MODE)
    lcd.print("CV");
  else if (mode==CR_MODE)
    lcd.print("CR");
  else if (mode==TL_MODE)
    lcd.print("TL");
  else
    lcd.print("??");
}

void printMenu() {
  lcd.clear();
  if(menu==MAIN_MENU) {
    lcd.setCursor(5,0);
    lcd.print("Main Menu");
  } else if (menu==TRANSIENT_MENU) {
    lcd.setCursor(3,0);
    lcd.print("Transient Menu");
  }
}

void printOptions(void) {
  Serial.println("printOptions");
  if(menu==NO_MENU) return;
  printClearLine(2);
  printClearLine(3);
  uint8_t arraysize = sizeof(options)/sizeof(options[0]);
  Serial.print("arraysize ");
  Serial.println(arraysize);
  for(int i=0; i<arraysize; i++) {
    Serial.print("i=");
    Serial.print(i);
    memcpy_P(&menubuf,&options[i],sizeof(menuoption));
    Serial.print(" menu=");
    Serial.print(menu);
    Serial.print(" menubuf.menu=");
    Serial.println(menubuf.menu);
    if(menu == menubuf.menu) {
      lcd.setCursor(menubuf.x, menubuf.y);
      lcd.print(menubuf.text);
    }
  }
  // get current menu option
  memcpy_P(&menubuf,&options[option],sizeof(menuoption));
  // print clams
  lcd.setCursor(menubuf.x-1,menubuf.y);
  lcd.print("[");
  lcd.setCursor(menubuf.x+strlen(menubuf.text),menubuf.y);
  lcd.print("]");
  lcd.noCursor();
}

void nextOption(void) {
  Serial.println("nextOption");
  memcpy_P(&menubuf,&options[option],sizeof(menuoption));
  lcd.setCursor(menubuf.x-1,menubuf.y);
  lcd.print(" "); // clear clams
  lcd.setCursor(menubuf.x+strlen(menubuf.text),menubuf.y);
  lcd.print(" ");
  option = menubuf.nextoptidx;
  memcpy_P(&menubuf,&options[option],sizeof(menuoption));
  lcd.setCursor(menubuf.x-1,menubuf.y);
  lcd.print("[");
  lcd.setCursor(menubuf.x+strlen(menubuf.text),menubuf.y);
  lcd.print("]");
  lcd.noCursor();
}

void nextMenu(void) {
  Serial.println("nextMenu");
  memcpy_P(&menubuf,&options[option],sizeof(menuoption));
  menu = menubuf.nextmenu;
}

void setFirstOption(void) {
  if (menu == NO_MENU) return;
  // find first option
  uint8_t arraysize = sizeof(options)/sizeof(options[0]);
  for(int i=0; i<arraysize; i++) {
    memcpy_P(&menubuf,&options[i],sizeof(menuoption));
    if(menu==menubuf.menu) {
      option=i;
      break;
    }
  }
}

// This function is used for finding out which menu to open based on mode
//uint8_t find_menuoption_idx_by_mode(uint8_t mode) {
//  uint8_t arraysize = sizeof(options)/sizeof(options[0]);
//  for(int i=0; i<arraysize; i++) {
//    memcpy_P(&menubuf,&options[i],sizeof(menuoption));
//    if(mode==menubuf.mode)
//      return i;
//  }
//  return 0;
//}

void setMode(void) {
  if(menu!=NO_MENU) return;
  Serial.println("setMode");
  memcpy_P(&menubuf,&options[option],sizeof(menuoption));
  mode = menubuf.mode;
  if(mode==CP_MODE)
    Power();
  else if (mode==CC_MODE)
    Current();
  else if (mode==CV_MODE)
    Voltage();
  else if (mode==CR_MODE)
    Resistance();
  else if (mode==TL_MODE) {
    //transientListSetup();
  } else if (mode==BC_MODE) {
    BatteryCapacity();
  }
  //else
  //  lcd.print("??");
}

void printClearLine(uint8_t line) {
  lcd.setCursor(0,line);
  lcd.print("                    ");
}

void SetupLimitsScreen(void) {
  #ifdef SETUPLIMITSSCREEN
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Maximum Limits Set");
  lcd.setCursor(0,1);
  lcd.print("Current Limit=");
  lcd.setCursor(17,1);
  lcd.print("A");
  lcd.setCursor(15,1);
  lcd.print(CurrentCutOff);

  lcd.setCursor(0,2);
  lcd.print("Power Limit  =");
  lcd.setCursor(17,2);
  lcd.print("W");
  lcd.setCursor(15,2);
  lcd.print(PowerCutOff);

  lcd.setCursor(0,3);
  lcd.print("Temp. Limit  =");
  lcd.setCursor(17,3);
  lcd.print((char)0xDF);
  lcd.print("C");
  lcd.setCursor(15,3);
  lcd.print(tempCutOff);
  delay(2000);
  #endif
}

void printLoadOn(void) {
  lcd.setCursor(0,0);
  lcd.print("DC LOAD ON ");
  digitalWrite(LoadLED,1);
}

void printLoadOff(void) {
  lcd.setCursor(0,0);
  lcd.print("DC LOAD OFF");
  digitalWrite(LoadLED,0);
}

void printTemp(int _temp) {
  lcd.setCursor(16,0);
  if(_temp<10) lcd.print(" ");
  lcd.print(_temp);                                        //display temperature of heatsink on LCD
  lcd.print((char)0xDF);
  lcd.print("C");
}

void printActualReading(void) {
 lcd.setCursor(0,1);
    
  if ( ActualCurrent < 10.0 ) {
    lcd.print(ActualCurrent,3);
  } else {
    lcd.print(ActualCurrent,2);
  }
  
  lcd.print("A");
  lcd.print(" ");
  
  if (ActualVoltage < 10.0) {
    lcd.print(ActualVoltage, 3);
  } else {
    lcd.print(ActualVoltage, 2);
  }    
  
  lcd.print("V");
  lcd.print(" ");
   
  if (ActualPower < 100 ) {
    lcd.print(ActualPower,2);
  } else {
    lcd.print(ActualPower,1);
  }
  lcd.print("W");
  lcd.print(" ");
}

void printSetPoint (int _setpoint) {
  const uint8_t decimals=5, decimalpoint=3, xpos=8, ypos=2;
  printValue(_setpoint, decimals, decimalpoint, xpos, ypos);
  lcd.setCursor (CP, 2);
  lcd.cursor();
}

//------------------------------------------------Read Keypad Input-----------------------------------------------------
/*
void readKeypadInput (void) {
  //customKey = customKeypad.getKey();
  
  //  if (customKey != NO_KEY){                             //only used for testing keypad (uncomment if required for testing keypad)
  // Serial.print("customKey = ");                          //only used for testing keypad (uncomment if required for testing keypad)
  // Serial.println(customKey);                             //only used for testing keypad (uncomment if required for testing keypad)
  //  }                                                     //only used for testing keypad (uncomment if required for testing keypad)
  
  //if(customKey == 'E'){                                     //check for Load on/off
  //  LoadSwitch();
  //}
  
  //  if(customKey == 'D') {                                   //check if Set-Up Mode Selected 
  //  delay(200);
  //  toggle = false;                                         //switch Load OFF
  //  userSetUp();
  //  encoderPosition = 0;                                    //reset encoder reading to zero
  //  index = 0;
  //  z = 1;                                                  //sets column position for LCD displayed character
  //  decimalPoint = (' ');                                   //clear decimal point text character reset
  //}
  //
  //  if(customKey == 'C'){                                   //check if Transient Mode Selected
  //  toggle = false;                                         //switch Load OFF
  //  transientType();
  // 
  //  encoderPosition = 0;                                    //reset encoder reading to zero
  //  index = 0;
  //  z = 1;                                                  //sets column position for LCD displayed character
  //  decimalPoint = (' ');                                   //clear decimal point text character reset
  //  }
  //       
  //  if(customKey == 'A'){                                   //check if Constant Current button pressed
  //  toggle = false;                                         //switch Load OFF
  //  printLoadOff();
  //  Current();                                              //if selected go to Constant Current Selected routine
  //  encoderPosition = 0;                                    //reset encoder reading to zero
  //  index = 0;
  //  z = 1;                                                  //sets column position for LCD displayed character
  //  decimalPoint = (' ');                                   //clear decimal point test character reset
  //  }
  //         
  //  if(customKey == 'B'){                                   //check if Constant Power button pressed
  //  toggle = false;                                         //switch Load OFF
  //  printLoadOff();
  //  Power();                                                //if selected go to Constant Power Selected routine
  //  encoderPosition = 0;                                    //reset encoder reading to zero
  //  index = 0;
  //  z = 1;                                                  //sets column position for LCD displayed character
  //  decimalPoint = (' ');                                   //clear decimal point test character reset
  //  }
  //          
  //  if(customKey == '#'){                                   //check if Constant Resistance button pressed  
  //  toggle = false;                                         //switch Load OFF
  //  printLoadOff();
  //  Resistance();                                           //if selected go to Constant Resistance Selected routine
  //  encoderPosition = 0;                                    //reset encoder reading to zero
  //  index = 0;
  //  z = 1;                                                  //sets column position for LCD displayed character
  //  decimalPoint = (' ');                                   //clear decimal point test character reset
  //  }
  //
  //  if(customKey == '*'){                                   //check if Battery Capacity button pressed
  //  dac.setVoltage(0,false);                                //Ensures Load is OFF - sets DAC output voltage to 0
  //  toggle = false;                                         //switch Load OFF
  //  batteryType();                                          //select battery type
  //  index = 0;
  //  z = 1;                                                  //sets column position for LCD displayed character
  //  decimalPoint = (' ');                                   //clear decimal point test character reset
  //
  //    if (exitMode == 1){                                   //if NO battery type selected revert to CC Mode
  //    printLoadOff();
  //    Current();                                            //if selected go to Constant Current Selected routine
  //    encoderPosition = 0;                                  //reset encoder reading to zero
  //    customKey = 'A';
  //    }
  //    else
  //    {
  //    lcd.setCursor(16,2);
  //    lcd.print(BatteryType);                               //print battery type on LCD 
  //    printLoadOff();
  //    timer.reset();                                        //reset timer
  //    BatteryLifePrevious = 0;
  //    CP = 9;                                               //set cursor position
  //    BatteryCapacity();                                    //go to Battery Capacity Routine
  //    }
  //  }
  //
  //if (Mode != "BC"){
  //
  //  if(customKey >= '0' && customKey <= '9'){               //check for keypad number input
  //       numbers[index++] = customKey;
  //       numbers[index] = '\0';
  //       lcd.setCursor(z,3);                                //                   
  //       lcd.print(customKey);                              //show number input on LCD
  //       z = z+1;
  //     }
  //  
  //  if(customKey == '>'){                                   //check if decimal button key pressed
  //      if (decimalPoint != ('>')){                         //test if decimal point entered twice - if so skip 
  //      numbers[index++] = '.';
  //      numbers[index] = '\0';
  //      lcd.setCursor(z,3);
  //      lcd.print(".");
  //      z = z+1;
  //      decimalPoint = ('>');                             //used to indicate decimal point has been input
  //        }
  //      }
  //
  //  if(customKey == '<'){                                 //clear input entry
  //     index = 0;
  //     z = 1;                                             //sets column position for LCD displayed character
  //     lcd.setCursor(z,3);
  //     lcd.print("     ");
  //     numbers[index] = '\0';                             //
  //     decimalPoint = (' ');                              //clear decimal point test character reset
  //    }
  //
  //  if(customKey == 'F') {                                //use entered number
  //    x = atof(numbers);     
  //         reading = x;
  //         encoderPosition = reading*1000;
  //         index = 0;
  //         numbers[index] = '\0';
  //         z = 1;                                         //sets column position for LCD displayed character
  //         lcd.setCursor(0,3);
  //         lcd.print("        ");
  //         decimalPoint = (' ');                          //clear decimal point test character reset
  //          }
  //    }
}
*/

//----------------------Limit Maximum Current Setting-----------------------------------------
void maxConstantCurrentSetting (void) {

  if (mode == CC_MODE && reading > CurrentCutOff){           //Limit maximum Current Setting
  reading = CurrentCutOff;
  //encoderPosition = (CurrentCutOff * 1000);               //keep encoder position value at maximum Current Limit
  //lcd.setCursor(0,3);
  //lcd.print("                    ");                      //20 spaces to clear last line of LCD 
  }

  if (mode == CP_MODE && reading > PowerCutOff) {             //Limit maximum Current Setting
        reading = PowerCutOff;
        //encoderPosition = (PowerCutOff * 1000);            //keep encoder position value at maximum Current Limit
        //lcd.setCursor(0,3);
        //lcd.print("                    ");                   //20 spaces to clear last line of LCD 
    }

   if (mode == CR_MODE && reading > ResistorCutOff ) {             //Limit maximum Current Setting
        reading = ResistorCutOff;
        //encoderPosition = (ResistorCutOff * 1000);            //keep encoder position value at maximum Current Limit
        //lcd.setCursor(0,3);
        //lcd.print("                    ");                   //20 spaces to clear last line of LCD 
    }
}

//----------------------Power Level Cutoff Routine-------------------------------------------
void powerLevelCutOff (void) {
  #ifdef POWERLEVEL_CUTOFF
  if (ActualPower > PowerCutOff) {                        //Check if Power Limit has been exceed
    reading = 0;
    printLoadOff();
    toggle = false;                                         //switch Load Off
    delay(200);
  }
  #endif
}

//----------------------Battery Constant Current Limit Value------------------------------------------
void batteryCurrentLimitValue (void) {
  if (reading > MaxBatteryCurrent){
  reading = MaxBatteryCurrent;
  encoderPosition = (MaxBatteryCurrent*1000);            //keep encoder position value at 1000mA
  }
}

//----------------------Display Rotary Encoder Input Reading on LCD---------------------------
void displayEncoderReading (void) {
    lcd.setCursor(8,2);                                      //start position of setting entry

    if ( ( mode == CP_MODE || mode == CR_MODE ) && reading < 100 ) {
        lcd.print("0");
    }
    
    if (reading < 10) {                                      //add a leading zero to display if reading less than 10
        lcd.print("0"); 
    }

    if ( mode == CP_MODE || mode == CR_MODE ) {
        lcd.print (reading, 2);                              //show input reading from Rotary Encoder on LCD
    } else {
        lcd.print ((int)reading, 3);
    }
    lcd.setCursor (CP, 2);                                   //sets cursor position
    lcd.cursor();                                            //show cursor on LCD
}

//--------------------------Cursor Position-------------------------------------------------------
//Change cursor the position routine
void CursorPosition(void) {
    // Defaults for two digits before decimal and 3 after decimal point
    int unitPosition = 9;

    //Power and Resistance modes can be 3 digit before decimal but only 2 decimals
    if ( mode == CP_MODE || mode == CR_MODE ) {
        unitPosition = 10;        
    }

    if (digitalRead(CursorPos) == LOW) {
    
        //delay(200);                                          //simple key bounce delay  
        CP = CP + 1;
        if (CP == unitPosition + 1 ) {
            CP = CP + 1;
        }
    }
    
    if (CP > 13)  { CP = unitPosition; }                     //No point in turning tens and hundreds
    if (CP == unitPosition +4 ) { factor = 1; }
    if (CP == unitPosition +3 ) { factor = 10; }
    if (CP == unitPosition +2 ) { factor = 100; }
    if (CP == unitPosition )    { factor = 1000; }
}

//---------------------------------------------Read Voltage and Current--------------------------------------------------------------
void readVoltageCurrent (void) {
  long value;
  MCP342x::Config status;

  uint8_t err;

  // conversion state machine
  convstate = nextconvstate;

  switch(convstate) { // nextstate logic

    case CONVSTATE_CONVERTVOLTS:
      //Serial.println("Convert volts");
      err = adc.convert(configVolts);
      if (err) {
        Serial.print("Convert ERROR: ");
        Serial.println(err);
      }
	    nextconvstate = CONVSTATE_READVOLTS;
      break;
	  
    case CONVSTATE_READVOLTS:
      err = adc.read(value, status);
      if (!err && status.isReady()) {
        adc_voltage = value;
        // For debugging purposes print the return value.
        #ifdef ADC_DEBUG
        Serial.print("Volts Value: ");
        Serial.print(value);
        Serial.print("  Config: 0x");
        Serial.println((int)configVolts, HEX);
        #endif
        nextconvstate = CONVSTATE_CONVERTAMPS;
      } else {
        #ifdef ADC_DEBUG
        Serial.print(".");
        #endif
      }
      break;

    case CONVSTATE_CONVERTAMPS:
      //Serial.println("Convert amps");
      err = adc.convert(configAmps);
      if (err) {
        Serial.print("Convert ERROR: ");
        Serial.println(err);
      }
	    nextconvstate = CONVSTATE_READAMPS;
      break;

    case CONVSTATE_READAMPS:
      err = adc.read(value, status);
      if (!err && status.isReady()) {
        adc_current = value;
        // For debugging purposes print the return value.
        #ifdef ADC_DEBUG
        Serial.print("Amps  Value: ");
        Serial.print(value);
        Serial.print("  Config: 0x");
        Serial.println((int)configAmps, HEX);
        #endif
        nextconvstate = CONVSTATE_CONVERTVOLTS;
      } else {
        #ifdef ADC_DEBUG
        Serial.print(".");
        #endif
      }
      break;
  }
}

//-----------------------------------Calculate Actual Voltage and Current and display on LCD-----------------------------------------
void check_adc_presence() {
  Wire.requestFrom(address, (uint8_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    printERROR_NoADC();
    while (1)
      ;
  } else {
    Serial.println("device found!");
  }
}

void ActualReading(void) {
  ActualCurrent = (((adc_current*2.048)/32767) * 2.5);    //calculate load current
  currentDisplayCal();                                    //LCD display current calibration correction
  ActualVoltage = (((adc_voltage*2.048)/32767) * 50.4);   //calculate load voltage upto 100v (was 50)
  ActualPower = ActualVoltage*ActualCurrent;

  if (ActualPower <=0){
    ActualPower = 0;
  }

  if (ActualVoltage <=0.0){                              //added to prevent negative readings on LCD due to error
    ActualVoltage = 0.0;
  }
  if (ActualCurrent <= 0.0){                             //added to prevent negative readings on LCD due to error
    ActualCurrent = 0.0;
  }
}

//-----------------------DAC Control Voltage for Mosfet---------------------------------------
void dacControlVoltage (void) {
  if (mode == CC_MODE){
    setCurrent = reading*1000;                                //set current is equal to input value in Amps
    setReading = setCurrent;                                  //show the set current reading being used
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
    controlVoltage = setControlCurrent; 
  }

  if (mode == CP_MODE){
    setPower = reading*1000;                                  //in Watts
    setReading = setPower;
    setCurrent = setPower/ActualVoltage;
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
    controlVoltage = setControlCurrent;                       //
  }

  if (mode == CR_MODE){
    setResistance = reading;                                  //in ohms
    setReading = setResistance;
    setCurrent = (ActualVoltage)/setResistance*1000;
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
    controlVoltage = setControlCurrent; 
  }

  if (mode == TT_MODE || mode == TL_MODE || mode == TC_MODE || mode == TP_MODE) { //Transient Modes
    setControlCurrent = (setCurrent * 1000) * setCurrentCalibrationFactor;
    controlVoltage = setControlCurrent; 
  }
}

//-------------------------------------Battery Capacity Discharge Routine----------------------------------------------------
void batteryCapacity (uint8_t loadbutton, uint8_t encbutton) {
  if (mode != BC_MODE) return;
  if (encbutton == LONG_PRESS) {
    userSetUp();
    return;
  }

  if (loadbutton == SHORT_PRESS) {
    LoadSwitch();
    return;
  }
  else if (loadbutton==LONG_PRESS) {
    LoadOff();
    menu=MAIN_MENU;
    setFirstOption();
    printMenu();
    printOptions();
    setMode();
    return;
  }
  setCurrent = reading*1000;                             //set current is equal to input value in Amps
  setReading = setCurrent;                               //show the set current reading being used
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent;

  lcd.setCursor(0,3);
  lcd.print (timer.getTime());                           //start clock and print clock time

  Seconds = timer.getTotalSeconds();                     //get totals seconds
  
  LoadCurrent = ActualCurrent;                           //if timer still running use present Actual Current reading
  if (timer.status() == 2){                              //if timer is halted then use last Actual Current reading before timer stopped
    LoadCurrent = BatteryCurrent;
  }
 
  BatteryLife = (LoadCurrent*1000)*(Seconds/3600);       //calculate battery capacity in mAh
  lcd.setCursor(9,3);
  BatteryLife = round(BatteryLife);

  if (BatteryLife >= BatteryLifePrevious){                //only update LCD (mAh) if BatteryLife has increased
  
    if (BatteryLife < 10) {                              //add a 3 leading zero to display if reading less than 10
      lcd.print("000");
    }

    if (BatteryLife >= 10 && BatteryLife <100){          //add a 2 leading zero to display
      lcd.print("00");  
    }

    if (BatteryLife >= 100 && BatteryLife <1000){        //add a 1 leading zero to display
      lcd.print("0"); 
    }
    lcd.print(BatteryLife,0);
    lcd.setCursor(13,3);
    lcd.print("mAh");
    BatteryLifePrevious = BatteryLife;                      //update displayed battery capacity on LCD
  } 

  if (ActualVoltage <= BatteryCutoffVolts) { //stops clock if battery reached cutoff level and switch load off
    BatteryCurrent = ActualCurrent;
    dac.setVoltage(0,false);                                  //reset DAC to zero for no output current set at switch on                                             
    toggle = false;                                           //Load is toggled OFF
    printLoadOff();
    timer.stop();
  }

  if (Load == 1) {                       //Routine used for data logging in Battery Capacity Mode
    if (Seconds != SecondsLog){                       //only send serial data if time has changed
      SecondsLog = Seconds;
      Serial.print (SecondsLog);                    //sends serial data of time in seconds
      Serial.print (",");                             //sends a comma as delimiter for logged data
      Serial.println (ActualVoltage);                   //sends serial data of Voltage reading         
    }
  }
}

//--------------------------------------------------Fan Control----------------------------------------------------------
void fanControl (void) {
  temp = analogRead(temperature);
  temp = temp * 0.107421875;                                // convert to Celsius
  
  if (temp >= 40){                                        //if temperature 40 degree C or above turn fan on.
    digitalWrite(fan, HIGH); 
  }  else {
    digitalWrite(fan, LOW);                               //otherwise turn fan turned off
  }
}

//-----------------------Toggle Current Load ON or OFF------------------------------
void LoadSwitch(void) {
  
  //delay(200);                                              //simple key bounce delay 
 
    if(toggle)
    {
      LoadOff();
    }
    else
    {
      LoadOn();
    }
}

void LoadOff(void) {
  printLoadOff();
  current_instruction = 0;                            //reset current instruction for Transient List Mode to zero
  last_time = 0;                                      //reset last time to zero
  transientPeriod = 0;                                //reset transient period time to zero
  setCurrent = 0;                                     //reset setCurrent to zero
  toggle = false;
  Load = 0;        
}

void LoadOn(void) {
	printLoadOn();
  toggle = true;
  Load = 1;
}

//-----------------------Select Constant Current LCD set up--------------------------------
void Current(void) {
  mode = CC_MODE;
  lcd.setCursor(0,0);
  lcd.print("DC LOAD             ");
  lcd.setCursor(0,2);
  lcd.print("Set I =             ");
  lcd.setCursor(15,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  lcd.setCursor(18,3);
  lcd.print("CC");
  CP = 9;
  LoadOff();
}

//----------------------Select Constant Power LCD set up------------------------------------
void Power(void) {
  mode = CP_MODE;
  lcd.setCursor(0,0);
  lcd.print("DC LOAD             ");
  lcd.setCursor(0,2);
  lcd.print("Set W =             ");
  lcd.setCursor(15,2);
  lcd.print("W");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  lcd.setCursor(18,3);
  lcd.print("CP");
  CP = 10;                                               //sets cursor starting position to units.
  LoadOff();
}

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Resistance(void) {
  mode = CR_MODE;
  lcd.setCursor(0,0);
  lcd.print("DC LOAD             ");
  lcd.setCursor(0,2);
  lcd.print("Set R =             ");
  lcd.setCursor(15,2);
  lcd.print((char)0xF4);
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  lcd.setCursor(18,3);
  lcd.print("CR");
  CP = 10;                                               //sets cursor starting position to units.
  LoadOff();
}

void Voltage(void) {
  mode = CV_MODE;
  lcd.setCursor(0,0);
  lcd.print("DC LOAD             ");
  lcd.setCursor(0,2);
  lcd.print("Set V =             ");
  lcd.setCursor(15,2);
  lcd.print("V");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
  lcd.setCursor(18,3);
  lcd.print("CV");
  CP = 10;                                               //sets cursor starting position to units.
  LoadOff();
}
//----------------------- Select Battery Capacity Testing LCD set up---------------------------------------
void BatteryCapacity(void) {
  mode = BC_MODE;
  lcd.setCursor(0,0);
  lcd.print("BATTERY");
  lcd.setCursor(0,2);
  lcd.print("                ");
  lcd.setCursor(0,2);
  lcd.print("Set I = ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("                    ");                    //20 spaces so as to allow for Load ON/OFF to still show
}

//----------------------Battery Type Selection Routine------------------------------------------------
void batteryType (void) {
  exitMode = 0;                                         //reset EXIT mode
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Select Battery Type");  
  lcd.setCursor(0,1);
  lcd.print("1=LiPo/Li-Ion 2=LiFe");
  lcd.setCursor(0,2);
  lcd.print("3=NiCd/NiMH   4=ZiZn");
  lcd.setCursor(0,3);                                   //clear last line of time info
  lcd.print("5=Set Voltage 6=Exit");                    //20 spaces so as to allow for Load ON/OFF to still show

  //customKey = customKeypad.waitForKey();                //stop everything till the user press a key.
  //
  //if (customKey == '1'){
  //BatteryCutoffVolts = LiPoCutOffVoltage;
  //BatteryType = ("LiPo");
  //  }
  //
  //if (customKey == '2'){
  //BatteryCutoffVolts = LiFeCutOffVoltage;
  //BatteryType = ("LiFe");
  //  }
  //
  //if (customKey == '3'){
  //BatteryCutoffVolts = NiCdCutOffVoltage;
  //BatteryType = ("NiCd");  
  //  }
  //
  //if (customKey == '4'){
  //BatteryCutoffVolts = ZiZnCutOffVoltage;
  //BatteryType = ("ZiZn"); 
  //  }
  //
  //if (customKey == '5'){ 
  //BatteryType = ("SetV");
  //  }
  //
  //if (customKey == '6'){                                  //Exit selection screen
  //exitMode = 1;
  //  }
  //
  //if (customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == 'A' || customKey == 'B' || customKey == 'C' || customKey == 'D' || customKey == '*' || customKey == '#' || customKey == 'E' || customKey == 'F' || customKey == '<' || customKey == '>' ){
  //batteryType();                                                      //ignore other keys
  //  }
  //
  //if(BatteryType == "SetV" && exitMode != 1){
  //setBatteryCutOff();
  //  }

  batteryTypeSelected();                                    //briefly display battery type selected and discharge cut off voltage

  lcd.clear();

}

//--------------------------Set DAC Voltage--------------------------------------------
void dacControl (void) {
  if (!toggle){
    dac.setVoltage(0,false);                                 //set DAC output voltage to 0 if Load Off selected
    if(mode == BC_MODE && ActualVoltage >= BatteryCutoffVolts && timer.status() == 1) {
      timer.stop();
    }
  
  } else {
    //Serial.println("Control Voltage");                    //used for testing only
    //Serial.println(controlVoltage);                       //used for testing only
    dac.setVoltage(controlVoltage,false);                   //set DAC output voltage for Range selected
    if(mode == BC_MODE && ActualVoltage >= BatteryCutoffVolts && timer.status() != 1) {
      timer.start();
    }
  }
}

//--------------------------Battery Selected Information--------------------------------------------
void batteryTypeSelected (void) {
  if (exitMode != 1) {                                      //if battery selection was EXIT then skip this routine
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("Battery Selected");
    lcd.setCursor(8,1);
    lcd.print(BatteryType);                                 //display battery type selected
    lcd.setCursor(2,2);
    lcd.print("Discharge Cutoff");
    lcd.setCursor(6,3);
    lcd.print(BatteryCutoffVolts);                          //display battery discharge cut off voltage
    lcd.print(" volts");                                    
    delay(3000);
  }
}

//--------------------------Set Battery Cut-Off Voltage--------------------------------------------
void setBatteryCutOff (void) {

  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("Enter Battery");
  lcd.setCursor(3,1);
  lcd.print("Cut-Off Voltage");
  y = 8;
  z = 8;
  r = 2;

  //inputValue();
  BatteryCutoffVolts = x;

  lcd.clear();
}

void inputValue (int* value, int maxvalue, int minvalue, uint8_t decimals, uint8_t decimalpoint, uint8_t xpos, uint8_t ypos) {
  Serial.println("inputValue");
  uint8_t encbutton = 0;
  uint8_t encvelo = 0;
  int LOOP=0;
  int oldvalue=-1; // make it print the first time
  //encoderPosition = *value;
  //uint8_t currentencpos = encoderPosition;
  if (*value>maxvalue) *value=maxvalue;
  else if (*value<minvalue) *value=minvalue;
  while(encbutton==0) {
    uint8_t velo = EncoderVelocity();
    if (velo) {
      if(velo) {
        if (decimals>3) {
          *value+=poschange*100;
        }
        else {
          *value+=poschange;
        }
        if (*value>maxvalue) *value=maxvalue;
        else if (*value<minvalue) *value=minvalue;
        printValue(*value, decimals, decimalpoint, xpos, ypos);
      }
    }
    encbutton = EncoderButton();

    LOOP++;
    if(LOOP>100) {
      printValue(*value,decimals,decimalpoint,xpos,ypos);
      LOOP=0;
    }
    delay(1);
  }
  printValue(*value,decimals,decimalpoint,xpos,ypos);
  Serial.print("inputValue returns ");
  Serial.println(*value);
}

//----------------------------------------Transient Mode--------------------------------------------
void transientMode (void) {

  if (mode != TL_MODE) {
    y = 11;
    z = 11;

    lcd.noCursor();                                       //switch Cursor OFF for this menu               
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("Transient Mode");  
    lcd.setCursor(0,1);
    lcd.print("Set Low  I=");
    lcd.setCursor(19,1);
    lcd.print("A");
    r = 1;
    //inputValue();

    if (x >= CurrentCutOff) {
      LowCurrent = CurrentCutOff;
    } else {
      LowCurrent = x;
    }
    lcd.setCursor(11,r);
    lcd.print(LowCurrent,3);

    //customKey = '0';

    z = 11;

    lcd.setCursor(0,2);
    lcd.print("Set High I=");
    lcd.setCursor(19,2);
    lcd.print("A");
    r = 2;
    //inputValue();
    if (x >= CurrentCutOff) {
      HighCurrent = CurrentCutOff;
    } else {
      HighCurrent = x;
    }
    lcd.setCursor(11,r);
    lcd.print(HighCurrent,3);

    //customKey = '0';

    if (mode == TC_MODE || mode == TP_MODE) {
      z = 11;

      lcd.setCursor(0,3);
      lcd.print("Set Time  = ");
      lcd.setCursor(16,3);
      lcd.print("mSec");
      r = 3;
      //inputValue();
      transientPeriod = x;
      lcd.setCursor(11,r);
      lcd.print(transientPeriod);
    } else {
      lcd.setCursor(0,3);
      lcd.print("                    ");
    }
    lcd.clear();
    toggle = false;                                           //switch Load OFF
    printLoadOff();

  } else {

    transientListSetup();
    lcd.clear();
    toggle = false;                                           //switch Load OFF
    printLoadOff();
  }
}

//----------------------------------------Transient--------------------------------------------
void transient (void) {
  
  if (mode == TC_MODE || mode == TP_MODE || mode == TT_MODE || mode == TL_MODE) {
    lcd.noCursor();
    lcd.setCursor(0,0);
    lcd.print("DC LOAD");
  
    if(mode != TL_MODE){
      lcd.setCursor(0,2);
      lcd.print("Lo=");
      lcd.setCursor(3,2);
      lcd.print(LowCurrent,3);
      lcd.setCursor(8,2);
      lcd.print("A");
      lcd.setCursor(11,2);
      lcd.print("Hi=");
      lcd.setCursor(14,2);
      lcd.print(HighCurrent,3);
      lcd.setCursor(19,2);
      lcd.print("A");
    } else {
      delay(1);
    }

    if (mode == TC_MODE || mode == TP_MODE || mode == TL_MODE) {
      printValue(current_instruction, 2, 0, 0, 3);
      lcd.print(": ");
      //void printValue(int value, uint8_t decimals, uint8_t decimalpoint, uint8_t xpos, uint8_t ypos);
      printValue(transientPeriod, 4, 0, 4, 3);
      //lcd.print(transientPeriod);
      lcd.print("ms");
    } else {
     lcd.setCursor(0,3);
     lcd.print("  ");
    }

  }
  delay(1);
}

//-------------------------------------Transcient List Setup-------------------------------------------
void transientListSetup() {
  lcd.noCursor();                                                 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setup Transient List");
  lcd.setCursor(0,1);
  lcd.print("Enter Number in List");
  lcd.setCursor(0,2);
  lcd.print("(between 2 to 10 max"); 
  inputValue(&total_instructions,10,2,2,0,0,3);
  for(uint8_t i=0; i<(total_instructions); i++) {
    if(i%4 == 0) lcd.clear();
    lcd.setCursor(0, i & 0x03);
    lcd.print("I");
    lcd.print(i+1);
    lcd.print("=");
    lcd.setCursor(6,i & 0x03);
    lcd.print("A");
    inputValue(&transientList[i][0],50,0,2,1,3,i&0x03); // Amps
    transientList[i][0] = transientList[i][0]*100;
    Serial.print("transientList[][0] = ");
    Serial.println(transientList[i][0]);
    lcd.setCursor(8,i & 0x03);
    lcd.print("t");
    lcd.print(i+1);
    lcd.print("=");
    lcd.setCursor(17,i&0x03);
    lcd.print("ms");
    inputValue(&transientList[i][1],32000,0,5,0,12,i&0x03);
    Serial.print("transientList[][1] = ");
    Serial.println(transientList[i][1]);
  }
  current_instruction = 0;      //start at first instrution
  delay(200);
  LoadOff();
}

//---------------------------Transcient Load Toggel-----------------------------------
void transientLoadToggle(uint8_t loadbutton, uint8_t encbutton) {
  if (encbutton == LONG_PRESS) {
    userSetUp();
    return;
  } else if (encbutton == SHORT_PRESS) {
    transientListSetup();
    return;
  }
  if (loadbutton == SHORT_PRESS) {
    LoadSwitch();
    return;
  }
  else if (loadbutton==LONG_PRESS) {
    LoadOff();
    menu=TRANSIENT_MENU;
    setFirstOption();
    printMenu();
    printOptions();
    return;
  }
  if (mode == TC_MODE) {
    current_time = micros();                              //get the current time in micro seconds()
    if (last_time == 0) {
      last_time = current_time;
    } else {
      switch (transient_mode_status) {

        case (false):
          // we are in the low current setting
          if ((current_time - last_time) >= (transientPeriod * 1000.0))
            transientSwitch(LowCurrent, true);
          break;

        case (true):
          // we are in the high current setting 
          if ((current_time - last_time) >= (transientPeriod * 1000.0))
            transientSwitch(HighCurrent, true);
          break;
      } 
    }
  }

  else if (mode == TP_MODE) {
    current_time = micros();
    if (last_time == 0) {
      last_time = current_time;
      transientSwitch(LowCurrent, true);
    }
    if (digitalRead(TriggerPulse) == LOW) {
      // a trigger pluse is received
      // set to the high current
      transientSwitch(HighCurrent, true);
    } else {
      if ((current_time - last_time) >= (transientPeriod * 1000.0)) {
          transientSwitch(LowCurrent, true);
      }
    }
  }

 else if (mode == TT_MODE) {          // this function will toggle between high and low current when the trigger pin is taken low
  if (digitalRead(TriggerPulse) == LOW) {
    switch (transient_mode_status){

      case (false):
        transientSwitch(LowCurrent, true);
        delay(200);         //added to prevent key bounce when transient button used (date of addition 31-03-2018)
        break;

      case (true):
        transientSwitch(HighCurrent, true);
        delay(200);         //added to prevent key bounce when transient button used (date of addition 31-03-2018)   
        break;

      }
    }
  }

  else if (mode == TL_MODE) {
    if (Load == 1) {                                                 //Only perform Transient List if Load is ON
      //current_time = micros();                                     //get the current time in micro seconds()
      current_time = millis();
      if (last_time == 0) {
        last_time = current_time;
        transientPeriod = transientList[current_instruction][1];    //Time data for LCD display
        transientSwitch(transientList[current_instruction][0], false);
        transient();
      }
      if ((current_time - last_time) >= transientPeriod) {     //move to next list instruction
        transientPeriod = transientList[current_instruction][1];   //Time data for LCD display
        transientSwitch(transientList[current_instruction][0], false);
        transient();
        current_instruction++;
        if (current_instruction >= total_instructions)
          current_instruction = 0;
      }
    }
  }
}

//-------------------------------------Transcient Switch-------------------------------------------
void transientSwitch(int current_setting, boolean toggle_status){
  if (toggle_status) {
    transient_mode_status = !transient_mode_status;
  }
  setCurrent = current_setting;
  printSetPoint(setCurrent);
  //Serial.print("set current = ");                     //used for testing only
  //Serial.println(setCurrent);                         //used for testing only
  last_time = current_time;
}

//-------------------------------------User set up for limits-------------------------------------------------
void userSetUp (void) {
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("User Set-Up");
  lcd.setCursor(0,1);
  lcd.print("Current Limit=");
  lcd.setCursor(19,1);
  lcd.print("A");
  const uint8_t xpos=15, ypos=1, decimals=3, decimalpoint=0;
  inputValue(&CurrentCutOff,CURRENTCUTOFF_MAXLIMIT,1,decimals,decimalpoint,xpos,ypos);
  EEPROM.write(0x00, CurrentCutOff);

  lcd.setCursor(0,2);
  lcd.print("Power Limit  =");
  lcd.setCursor(19,2);
  lcd.print("W");
  inputValue(&PowerCutOff,254,1,decimals,decimalpoint,15,2);
  EEPROM.write(0x20, PowerCutOff);              //

  lcd.setCursor(0,3);
  lcd.print("Temp. Limit  =");
  lcd.setCursor(18,3);
  lcd.print((char)0xDF);
  lcd.print("C");
  inputValue(&tempCutOff,90,1,decimals,decimalpoint,15,3);
  EEPROM.write(0x40, tempCutOff);

  lcd.clear();
  printLoadOff();
  encoderPosition = 0;                                  //reset encoder reading to zero
  setMode();
}

//------------------------------------------High Temperature Cut-Off--------------------------------------------------------------
void temperatureCutOff (void) {
  #ifdef TEMPERATURE_CUTOFF
  if (temp >= tempCutOff) {
    reading = 0;
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("Over Temperature");
    printLoadOff();
    toggle = false;                                         //switch Load Off
  }
  #endif
}

//-----------------------------------------Current Read Calibration for LCD Display --------------------------------------------
void currentDisplayCal (void) {

  if (ActualCurrent <= 0) {
    ActualCurrent = 0;
  } else if (Load == 0) {
    ActualCurrent = 0;
  } else {
    ActualCurrent = ActualCurrent + displayCurrentCal;
  }

  /*  if (ActualCurrent <= 0.5)
    {
    ActualCurrent = (ActualCurrent +(displayCurrentCal * 3));
    }
    else if (ActualCurrent >= 0.5 && ActualCurrent <1.0)
    {
    ActualCurrent = (ActualCurrent + (displayCurrentCal * 2));
    }
    else if (ActualCurrent >= 1.0 && ActualCurrent <= 1.4)
    {
    ActualCurrent = (ActualCurrent + (displayCurrentCal));
    }
    else 
    {
    ActualCurrent = ActualCurrent;
    }
  */
}
  
//-----------------------------Show limits Stored Data for Current, Power and Temp-----------------------------
void get_eeprom_setup_limits (void) {
  CurrentCutOff = EEPROM.read(0x00);
  PowerCutOff = EEPROM.read(0x20);
  tempCutOff = EEPROM.read(0x40);
}
