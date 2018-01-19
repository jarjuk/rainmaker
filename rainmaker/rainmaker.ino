// State machine library
#include <SM.h>
#include <State.h>

// This library allows an Arduino board to control LiquidCrystal
// displays (LCDs) based on the Hitachi HD44780 (or a compatible)
// chipset, which is found on most text-based LCDs.
#include <LiquidCrystal.h>


//Needed to access the eeprom read write functions
#include <EEPROM.h>

#if 0
// Provides an Arduino library for reading and interpreting Bosch
// BME280 data over I2C, SPI or Sw SPI.
#include <BME280I2C.h>
#endif
#include <AM2320.h>


/*
Partco 
    PUN = VDD  5V
    MUS = GND  GND
    KEL = SDA  A4 
    VAL = SCL  A5
*/

// This library allows you to communicate with I2C / TWI devices
#include <Wire.h>


// ------------------------------------------------------------------
// Configs

const int selectButtonPin = 7;     // pin for 'select' pushbutton
const int setButtonPin = 8;        // pin for 'set' pushbutton
const int ledPin =  13;            // the number of the LED pin

#if 0
// LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
// Manual 
const int lcd_pin1=12;
const int lcd_pin2=11;
const int lcd_pin3=5;
const int lcd_pin4=4;
const int lcd_pin5=3;
const int lcd_pin6=2;
#endif
// Lcd-shield pins
const int lcd_pin1=8;
const int lcd_pin2=9;
const int lcd_pin3=4;
const int lcd_pin4=5;
const int lcd_pin5=6;
const int lcd_pin6=7;

// define some values used by the panel and buttons
const int lcd_button_port=0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5


#define BUTTON_DEBOUNCE_INTERVAL 50            // 50 ms
#define MENU_UPDATE_INTERVAL 100               // 1/2 sec
#define MEASUREMENT_UPDATE_INTERVAL 5000       // 5 secs
#define IDLE_TIME_OUT 1000*idle_timeout        // change to idle mode
#define RAIN_UPDATE_INTERVAL 1000              // check for rain
#define RAIN_DELAY_MAX  8                      // 8 * 15 min = 2h                    
#define RAIN_DELAY_TICK 60*15                  // 60 sec * 15 = 15 min
#define RAIN_DELAY  rain_delay*RAIN_DELAY_TICK // count 15 min tick between showers
#define RAIN_DURATION rain_duration            // secs to rain


#define CMD_NONE 0  // emtpy cmd queueu
#define CMD_SEL  1  // first button, select action
#define CMD_SET  2  // second button, set action


// ------------------------------------------------------------------
// State

short currentCommand = CMD_NONE;   // set in button detect

// LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
LiquidCrystal lcd(lcd_pin1, lcd_pin2, lcd_pin3, lcd_pin4, lcd_pin5, lcd_pin6);

// Temp and humidy
// Create an instance of sensor
AM2320 sensor;
#if 0
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
#endif


const char* relase= "$Release:0.1.4-SNAPSHOT$";

// Measurement
struct Ambient {
       float temp; // (NAN);
       float hum; // (NAN);
       float pres; // (NAN);
};
struct Ambient ambient;

char operatingState[20];        // Line 2 tr:++++++++++

long  rainTimer;       // Counter in WaitToRain && Raining, use long to allow debug

boolean idling=false;  // true-->DisplayIdle, false-->DisplayMain

// ------------------------------------------------------------------
// Configuration

#define CONFIG_SLOTS 6
short configSlot;                  // points CONFIG_SLOTS

struct ConfigSlot {
  char* name;
  long* val;
  long  increment;
  long  max;
  long  min;
  int   eepromAddress;  // 0 no write, !=0 write to EEPROM[eepromAddress-1]
};

long temp_too_treshold;
long relative_humidity_treshold;
long idle_timeout;      //  secs to change to idle
long rain_delay;        //  rain_delay *RAIN_DELAY (15min) between showers
long rain_duration;     //  secs to rain

ConfigSlot slots[CONFIG_SLOTS] = {
  {
    "TH"                // warm temperature threshold
    ,&temp_too_treshold
    ,5                  // increment by 5 degrees
    ,50                 // max 50 deg
    ,20                 // min 20 deg
    ,1                  // save to eeprom 0 address
  },
  {
    "RH"                // dry/humid threshold
    ,&relative_humidity_treshold
    ,5                  // increment
    ,100                // max
    ,30                 // min
    ,2                  // save to EEPROM address slot
  },
  {
    "IDLE"              // Timeout before blanking LCD
    ,&idle_timeout      // idle timeout (seconds)
    ,30                 // increment
    ,600                // max 10 min
    ,30                 // min
    ,3                  // save to EEPROM address slot
  },
  {
    "DLY"               // Rain delay, number of ticks between showers
    ,&rain_delay        // delay * 15 mins between rain showever
    ,1                  // increment by slot count
    ,RAIN_DELAY_MAX     // max 8*15 mins = 2 hours
    ,1                  // min 1 slot = 15 mins
    ,4                  // save to EEPROM address slot
  },
  {
    "DUR"               // Rain duration = length of shower
    ,&rain_duration     // delay * 15 mins between rain showever
    ,10                 // increment 10 sec slots
    ,120                // max 120 secs = 2 minutes
    ,10                 // min 10 secs
    ,5                  // save to EEPROM address slot
  },
  {
    "TMR"               // Current counter value, allow change but NOT persist
    ,&rainTimer         // delay * 15 mins between rain showever
    ,RAIN_DELAY_TICK    // increment counter by tick size
    ,RAIN_DELAY_TICK * RAIN_DELAY_MAX 
                        // max to rain delay = number of ticks between showers
    ,0                  // min = start counter from 0
    ,0                  // NOT saved to EEPROM
  }  
};


// ------------------------------------------------------------------
// State machine

// Define state machine 'Main'

State MainStarting();         // Init
State MainOperating();        // Main operating mode
SM    SMMain(MainStarting);

State SelectButtonDetect();   // Debounce 'select' button
SM    SMSelectButton(SelectButtonDetect);

State SetButtonDetect();      // Debounce 'set' button
SM    SMSetButton(SetButtonDetect);

State IdleMeasurement();      // Just read 
State UpdateMeasurement();    // Read and display
SM    SMMeasurement(UpdateMeasurement);

State NoRain();               // Default, not too hot
State Raining();              // Rain for rain duration
State WaitToRain();           // Wait until rain
SM    SMRain(NoRain);

State IdleChecker();          // Idling changes DisplayMain -> DisplayIdle
SM    SMIdle(IdleChecker);

State DisplayMain();          // Display in 'operating mode'
State DisplayIdle();          // Display in 'idle operating mode'
State DisplayConfiguring();   // Display in 'configuration' mode
State DisplayConfigureSet();  // Display in 'configuration set' mode
State DisplayPersist();     // Display make permanent
SM    SMDisplay(DisplayMain);

// ------------------------------------------------------------------
// 


// Read measurement to 'ambient' state variable, update display
State UpdateMeasurement() {
    if(SMMeasurement.Timeout(MEASUREMENT_UPDATE_INTERVAL)) {
      Serial.println("UpdateMeasurement timeout" );
      readSensor( &ambient );
      displayBME( &ambient );
      SMMeasurement.Set(UpdateMeasurement);
    }
}

// Read measurement to 'ambient' state variable, no display
State IdleMeasurement() {

  if(SMMeasurement.Timeout(MEASUREMENT_UPDATE_INTERVAL)) {
      Serial.println("IdleMeasurement timeout" );
      readSensor( &ambient );
      // No update on display
      // displayBME( &ambient );
      // displayState();
      SMMeasurement.Set(IdleMeasurement);
  }
}


// ------------------------------------------------------------------
// Rain

State NoRain() {

  if( SMRain.Timeout(RAIN_UPDATE_INTERVAL) ) {

    if ( shouldRain('+', rainTimer, RAIN_DELAY) ) {

      
      // Configuration change does no reset counnter
      // rainTimer = 0;
      SMRain.Set(WaitToRain);
    }
    else {
      rainTimer = 0;
      SMRain.Set(NoRain);
    }
    displayState(!idling);
  }
  

}

State WaitToRain() {

  if( SMRain.Timeout(RAIN_UPDATE_INTERVAL) ) {
    if ( shouldRain('+', rainTimer, RAIN_DELAY) ) {
      // Serial.println("WaitToRain rainTime=" + String(rainTimer) );
      if ( rainTimer  >= RAIN_DELAY )  {
	// start counting for rain duration
	Serial.println("WaitToRain->Raining rainTime=" + String(rainTimer) );
	rainTimer = 0;
	SMRain.Set(Raining);
      } else {
	SMRain.Set(WaitToRain);
      }
      rainTimer =  rainTimer + 1;
      displayState(!idling);
    }
    else {
      Serial.println("WaitToRain->NoRain rainTime=" + String(rainTimer) );
      SMRain.Set(NoRain);
    }
  }
}

State Raining() {

  if( SMRain.Timeout(RAIN_UPDATE_INTERVAL) ) {

    if ( shouldRain('=', rainTimer, RAIN_DURATION) ) {
      // Serial.println("Raining rainTimer=" + String(rainTimer) );
      if ( rainTimer  >= RAIN_DURATION )  {
	// start counting for rain delay
	rainTimer = 0;
	Serial.println("Raining->WaitToRain rainTimer=" + String(rainTimer) );	
	SMRain.Set(WaitToRain);
      } else {
	SMRain.Set(Raining);
      }
      rainTimer =  rainTimer + 1;
      displayState(!idling);
    }
    else {
      Serial.println("Raining->NoRain rainTimer=" + String(rainTimer) );	
      SMRain.Set(NoRain);
    }
  }
}




// ------------------------------------------------------------------
// Implement state machine 'Display'

// Active display mode - idle timeout changes display mode 'DisplayIdle'
State DisplayMain() {
  
  //  if(SMDisplay.Timeout(MENU_UPDATE_INTERVAL)) {

    // Idle inactivity changes DisplayMain -> DisplayIdle
    EXEC( SMIdle );

    // Operting normally
    EXEC( SMMeasurement );
    EXEC( SMRain );
    
    // Waiting for menu actions
    switch ( currentCommand ) {
    case  CMD_SEL:
      currentCommand = CMD_NONE;
      configSlot = 0;
      Serial.println("change DisplayConfiguring" );
      SMDisplay.Set(DisplayConfiguring);
      break;
    case  CMD_SET:
      // Reset idle
      Serial.println("DisplayMain - restart SMIdle" );      
      currentCommand = CMD_NONE;
    default:
      if ( idling) {
	// idling change - change mode
	activateDisplay(!idling);
	SMMeasurement.Set(IdleMeasurement);      
	SMDisplay.Set(DisplayIdle);
      } else {
	// SMMeasurement.Set(UpdateMeasurement);      
	SMDisplay.Set(DisplayMain);      
      }
    }
    //   }
} // DisplayMain

// Any key restores back to 'DisplayMain'
State DisplayIdle() {
  
  //   if(SMDisplay.Timeout(MENU_UPDATE_INTERVAL)) {

    // Operating normally
    EXEC( SMMeasurement );
    EXEC( SMRain );

    // Waiting for menu actions
    switch ( currentCommand ) {
    case  CMD_SEL:
    case  CMD_SET:
      // Change measurement mode
      currentCommand = CMD_NONE;
      Serial.println("DisplayIdle - activateDisplay(true)" );
      idling = false;
      activateDisplay(!idling);
      // restart idle timeout
      SMIdle.Set(IdleChecker);      
      SMMeasurement.Set(UpdateMeasurement);      
      SMDisplay.Set(DisplayMain);
      // Change to active display
      break;
    default:
      SMDisplay.Set(DisplayIdle);      
    }
    //   }
} // DisplayIdle



State DisplayConfiguring() {
  
  if(SMDisplay.Timeout(MENU_UPDATE_INTERVAL)) {

    displayConfig( configSlot );
    switch ( currentCommand ) {
    case  CMD_SEL:
      Serial.println("configSlot=" + String(configSlot) );
      currentCommand = CMD_NONE;
      configSlot = configSlot + 1;
      if ( configSlot>= CONFIG_SLOTS ) {
	// All slots configured -> persist?
	SMDisplay.Set(DisplayPersist);
      }
      else {
	SMDisplay.Set(DisplayConfiguring);      
      }
      break;
    case  CMD_SET:
      currentCommand = CMD_NONE;
      // Change current slot
      SMDisplay.Set(DisplayConfigureSet);
      break;
    default:
      SMDisplay.Set(DisplayConfiguring);      
    }
  }
}

// Change current slot
State DisplayConfigureSet() {
  
  if(SMDisplay.Timeout(MENU_UPDATE_INTERVAL)) {

    // Show menu to change
    displayConfigSet( configSlot );
    switch ( currentCommand ) {
    case  CMD_SEL:
      Serial.println("DisplayConfigureSet slot=" + String(configSlot) );
      // advance
      *(slots[configSlot].val) = *(slots[configSlot].val) + slots[configSlot].increment;
      if ( *slots[configSlot].val > slots[configSlot].max ) {
	// min value
	*slots[configSlot].val = slots[configSlot].min;
      }
      currentCommand = CMD_NONE;
      SMDisplay.Set(DisplayConfigureSet);      
      break;
    case  CMD_SET:
      currentCommand = CMD_NONE;
      SMDisplay.Set(DisplayConfiguring);      
      break;
    default:
      SMDisplay.Set(DisplayConfigureSet);      
    }
  }
}

State DisplayPersist() {
  if(SMDisplay.Timeout(MENU_UPDATE_INTERVAL)) {
    displayMakePermanent();
    switch ( currentCommand ) {
    case  CMD_SET:
      // save to eeprom - config slot with eepromAddress
      for ( int i=0; i < CONFIG_SLOTS; i++ ) {
	if ( slots[i].eepromAddress != 0  ) {
	  EEPROMWritelong( (slots[i].eepromAddress-1), *slots[i].val );
	}
      }
    case  CMD_SEL:
      currentCommand = CMD_NONE;
      // restart idle timeout
      SMIdle.Set(IdleChecker);
      // Back to main active main display
      SMMeasurement.Set(UpdateMeasurement);      
      SMDisplay.Set(DisplayMain); 
      break;
    default:
      SMDisplay.Set(DisplayPersist);      
    }
  }
}

// ------------------------------------------------------------------

// Idling changes DisplayMain -> DisplayIdle
State IdleChecker() {

    if(SMIdle.Timeout(IDLE_TIME_OUT)) {
      // SMIdle EXEC only within 'SMDisplay.DisplayMain'
      Serial.println("IdleChecker: timeout" );
      idling = true;
      SMIdle.Set(IdleChecker);      
    }
}

// ------------------------------------------------------------------
// Implement state machine 'Button'


State SelectButtonDetect() {
  
  static int detected = false;
  if(SMSelectButton.Timeout(BUTTON_DEBOUNCE_INTERVAL)) {

    //     boolean input = digitalRead(selectButtonPin);
    boolean input = read_LCD_button(btnSELECT);
    
    // if(detected && digitalRead(selectButtonPin) &&  currentCommand == CMD_NONE ) {
    
    if( RE(input, detected ) && (currentCommand == CMD_NONE) ){
      currentCommand = CMD_SEL;
      Serial.println("   currentCommand=" + String(currentCommand));
    }
    // detected = digitalRead(selectButtonPin);
    SMSelectButton.Set(SelectButtonDetect);
  }
}
  
State SetButtonDetect() {
  
  static int detected = false;

  if(SMSetButton.Timeout(BUTTON_DEBOUNCE_INTERVAL)) {
    // boolean input = digitalRead(setButtonPin);
    boolean input = read_LCD_button(btnLEFT);
    if( RE(input, detected ) && (currentCommand == CMD_NONE) ){
      currentCommand = CMD_SET;
      Serial.println("   currentCommand=" + String(currentCommand));
    }
    SMSetButton.Set(SetButtonDetect);
  }
}

// @return [boolean] true if button pressed
boolean read_LCD_button( int button ) {

    int button_read = read_LCD_buttons();
    return( button == button_read );
}

// read the buttons
int read_LCD_buttons()
{
  int adc_key_in;
 adc_key_in = analogRead(lcd_button_port);      // read the value from the sensor 
 // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
 // For V1.1 us this threshold
 /*
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 250)  return btnUP; 
 if (adc_key_in < 450)  return btnDOWN; 
 if (adc_key_in < 650)  return btnLEFT; 
 if (adc_key_in < 850)  return btnSELECT;  
 */

 // For V1.0 comment the other threshold and use the one below:

 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 195)  return btnUP; 
 if (adc_key_in < 380)  return btnDOWN; 
 if (adc_key_in < 555)  return btnLEFT; 
 if (adc_key_in < 790)  return btnSELECT;   


 return btnNONE;  // when all others fail, return this...
}


// ------------------------------------------------------------------
// Implement state machine 'Main'

State MainStarting() {

  // restore from EEPROM
  for ( int i=0; i < CONFIG_SLOTS; i++ ) {
    if ( slots[i].eepromAddress != 0  ) {
      *slots[i].val = EEPROMReadlong( slots[i].eepromAddress-1  );
      // Restore guarantees valid value ranges
      if ( *slots[i].val < slots[i].min  ) {
	*slots[i].val = slots[i].min;
      }
      if ( *slots[i].val > slots[i].max  ) {
	*slots[i].val = slots[i].max;
      }
      Serial.println( String(slots[i].name) + String("=")+ String(*slots[i].val) );
    }
  }
  
  
  Wire.begin();
  // while(!bme.begin())  {
  //   // Serial.println("Could not find BME280 sensor!");
  //   displayText("Could not find BME280 sensor!");
  //   delay(1000);
  // }

  // reset rain
  rainTimer = 0;
  SMRain.Set(NoRain);
  
  // Reset idle
  idling = false;
  SMIdle.Set(IdleChecker);
  // Main display 
  SMMeasurement.Set(UpdateMeasurement);      
  SMDisplay.Set(DisplayMain);      
  // Main loop
  SMMain.Set(MainOperating);
  
}

// Active elements are key controllers and display
State MainOperating() {
  EXEC( SMDisplay );
  EXEC( SMSetButton );    
  EXEC( SMSelectButton );
}

// ------------------------------------------------------------------
// Helpers

// Change display mode
void activateDisplay( boolean active ) {
  if ( active ) {
    // show same info as in 'UpdateMeasurement'
    displayReset();
    displayBME( &ambient );
    displayState(true);
  } else {
    // clean display
    displayReset();
  }
}

void displayReset() {
  lcd.setCursor(0, 0);
  lcd.clear();

}
void displayText( char* line1, char* line2=NULL ) {
  displayReset();
  lcd.print(line1 );
  if ( line2 ) {
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
}

// Show configuration value allow change
void displayConfig( short slot ) {
  displayReset();
  // lcd.print("Config" + String(slot) );
  char buf[80];
  sprintf( buf, "%s:%ld<%ld<%ld", slots[slot].name, slots[slot].min, *slots[slot].val, slots[slot].max );
  lcd.print( buf );
  lcd.setCursor(0, 1);
  lcd.print( "Next    Change");
}

// Display when changing value
void displayConfigSet( short slot ) {
  displayReset();
  char buf[80];
  sprintf( buf, "%s=%ld", slots[slot].name, *slots[slot].val  );
  // Serial.println(  buf );
  lcd.print( buf );
  lcd.setCursor(0, 1);
  lcd.print( "Next    Ret");
}

void displayMakePermanent() {
  displayReset();
  printConfig();
  lcd.setCursor(0, 1);
  lcd.print( "No     Yes");
}

// @param ch [char] to use indidicator
// @return true if should rain, set global 'operatingState'
boolean shouldRain(char ch, long timer, int timerMax ) {

  // default not raining
  boolean ret=false;
  
  if ( ambient.temp > temp_too_treshold && ambient.hum > relative_humidity_treshold ) {
    sprintf( operatingState, "TH - not raining" );
  }
  else if ( ambient.temp > temp_too_treshold  ) {
    char ticks[11];
    // Create indicitator
    for ( int i=0;i<sizeof(ticks);i++) {
      ticks[i] = ch;
    }
    int tickCount=((sizeof(ticks)-1)*timer)/timerMax;
    tickCount = (tickCount >= sizeof(ticks) ) ? sizeof(ticks)-1 : tickCount;
    ticks[ tickCount ] = '\0';
    ret = true;
    // Serial.println( "shouldRain: tickCount=" + String(tickCount) + "=" + String(((sizeof(ticks)-1)*timer)/timerMax) + ",timerMax=" + String(timerMax) );
    sprintf( operatingState, "ThRain%-10.10s|", ticks );
  }  else {
    sprintf( operatingState, "t- - not raining" );
  }
  // Serial.println( "shouldRain: operatingState=" + String(operatingState) );
  return( ret );
  
  
}

// One line for config
void printConfig() {
  char buf[80];
  sprintf( buf, "%ld,%ld,%ld,%ld,%ld", *slots[0].val, *slots[1].val, *slots[2].val,  *slots[3].val, *slots[4].val );
  lcd.print( buf );
}

void displayBME( struct Ambient *ambient ) {
     lcd.setCursor(0, 0);
     lcd.print( String( ambient->temp ) + "C "+ String( ambient->hum ) + "% RH");
     // displayState( ambient );
}

// Present 'operatingState' on line2 if 'show'
void displayState( boolean show ) {
  if ( show) {
    lcd.setCursor(0, 1);
    lcd.print( operatingState );
  }
}

// ------------------------------------------------------------------
// FUnction to access sensor
void readSensor( struct Ambient* ambient) {
  readAM2320( ambient );
}  

void readAM2320( struct Ambient* ambient) {

  if (sensor.measure()) {
    ambient -> temp = sensor.getTemperature();
    ambient -> hum= sensor.getHumidity();
    ambient -> pres = NAN;

    Serial.print("Temperature: ");
    Serial.println(sensor.getTemperature());
    Serial.print("Humidity: ");
    Serial.println(sensor.getHumidity());
  }
  else {
    int errorCode = sensor.getErrorCode();
    switch (errorCode) {
      case 1: Serial.println("ERR: Sensor is offline"); break;
      case 2: Serial.println("ERR: CRC validation failed."); break;
    }        
    ambient -> temp = NAN;
    ambient -> hum= NAN;
    ambient -> pres = NAN;
  }

  
}
#if 0
void readBME( struct Ambient* ambient) {
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celcius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);
   
   bme.read(pres, temp, hum, tempUnit, presUnit);
   ambient -> temp = temp;
   ambient -> hum= hum;
   ambient -> pres = pres;
}
#endif

// -------------------------------------------------------------------
// EEprop

//This function will write a 4 byte (32bit) long to the eeprom at
//the specified address to address + 3.
void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address*4, four);
      EEPROM.write(address*4 + 1, three);
      EEPROM.write(address*4 + 2, two);
      EEPROM.write(address*4 + 3, one);
      }

long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address*4);
      long three = EEPROM.read(address*4 + 1);
      long two = EEPROM.read(address*4 + 2);
      long one = EEPROM.read(address*4 + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }

// ------------------------------------------------------------------
// Arduino
void setup() {
  Serial.begin(115200);
  Serial.println("Starting " + String(relase) );
  
  // call sensor.begin() to initialize the library
  sensor.begin();

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  displayText( "rainmaker", relase );
  delay( 2000);


  // // initialize the LED pin as an output:
  // pinMode(ledPin, OUTPUT);

  // // initialize the pushbutton pin as an input:
  // pinMode(selectButtonPin, INPUT);
  // pinMode(setButtonPin, INPUT);

  // Main loop
  SMMain.Set(MainStarting);

}

void loop() {
  EXEC(SMMain);
}
