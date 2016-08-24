//----------------------------------------------------------------------------------------------------
// Sound Wave solar controller

#include "TimerOne.h"                // using Timer1 library from http://www.arduino.cc/playground/Code/Timer1
#include "LiquidCrystal_I2C.h"
#include "Alarm.h"
#include <Wire.h>  
#include <SoftwareSerial.h>         // using the Software Serial library Ref : http://www.arduino.cc/en/Reference/SoftwareSerialConstructor
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <pins_arduino.h>
 
//////// Arduino pins Connections//////////////////////////////////////////////////////////////////////////////////

// A0 - Voltage divider (solar)
// A1 - ACS 712 Out
// A2 - Voltage divider (battery)
// A4 - SDA I2C
// A5 - SCL I2C
// D6 - Solar Relay Out
// D7 - Alarm output

///////// Definitions ////////////////////////////////////////////////////////////////////////////////////////////////

#define SOLAR_VOLTS_CHAN 0               // defining the adc channel to read solar volts
#define SOLAR_AMPS_CHAN 1                // Defining the adc channel to read solar amps
#define BATTERY_VOLTS_CHAN 2             // defining the adc channel to read battery volts
#define RELAY_OUTPUT_PIN 6
#define ALARM_PIN 7
#define ADC_SAMPLES 6                        // number of iterations of the adc routine to average the adc readings

// ACS 712 Current Sensor is used. Current Measured = (5/(1024 *0.185))*ADC - (2.5/0.185) 

#define SOLAR_VOLTAGE_DIVIDER_R1 1000    // voltage divider R1 for solar voltage in ohms
#define SOLAR_VOLTAGE_DIVIDER_R2 270     // voltage divider R2 for solar voltage in ohms
#define SOLAR_MAXIMUM_VOLTAGE 21.6       // open circuit voltage, just to check your math

#define BATTERY_VOLTAGE_DIVIDER_R1 992  // voltage divider R1 for battery voltage in ohms
#define BATTERY_VOLTAGE_DIVIDER_R2 271   // voltage divider R2 for battery voltage in ohms

#define VOLTAGE_DIVIDER(Vcc, R1, R2) ((Vcc)/1024.0*((R1)+(R2))/(R2))
#define SOLAR_VOLTAGE_SCALE(Vcc) VOLTAGE_DIVIDER(Vcc, SOLAR_VOLTAGE_DIVIDER_R1, SOLAR_VOLTAGE_DIVIDER_R2)
#define BATTERY_VOLTAGE_SCALE(Vcc) VOLTAGE_DIVIDER(Vcc, BATTERY_VOLTAGE_DIVIDER_R1, BATTERY_VOLTAGE_DIVIDER_R2)

#define SENSOR_SCALE_AMPS 5
#define SOL_MA_SCALE  5000.0/512.0

#define TRUE 1
#define FALSE 0
#define ON TRUE
#define OFF FALSE

#define MAX_BAT_VOLTS 14.70         // maximum battery voltage
#define BATT_FLOAT 14.10            // charger turns OFF above this voltage
#define BATT_ON 12.6                // charger turns ON below this voltage
  
//-----------------------------------------------------------------------------------------------------
// Defining lcd back light pin
#define BACK_LIGHT_PIN 5       // pin-5 is used to control the lcd back light
//---------------------------------------------------------------------------------------------------------

int wdt = 0;

byte battery_icons[6][8]=
{{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11011,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
},
{
  0b01110,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
}};
#define SOLAR_ICON 6
byte solar_icon[8] = //icon for termometer
{
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b10101,
  0b11111,
  0b00000
};

//-------------------------------------------------------------------------------------------------------

// global variables
float sol_amps;                       // solar amps 
double Vcc = 5;
float sol_volts;                      // solar volts 
float bat_volts;                      // battery volts 
float sol_watts;                      // solar watts
uint32_t backlight_time = 0;     // variable to store time the back light control button was pressed in millis
int back_light_pin_State = 0;         // variable for storing the state of the backlight button
boolean load_status = false;                  // variable for storing the load output state (for writing to LCD)
int sol_amps_raw;
bool solarOn = false;
  
enum charger_mode {off, on, bulk, bat_float} charger_state;    // enumerated variable that holds state for charger state machine
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Set the LCD I2C address
Alarm alarm(ALARM_PIN);

//------------------------------------------------------------------------------------------------------
// This routine is automatically called at powerup/reset
//------------------------------------------------------------------------------------------------------
void KillWDT() {
    MCUSR &= ~_BV(WDRF);                 // Clear the WDT reset flag
    WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
    WDTCSR = 0x00;                      // Disable the WDT
}
void setup()                           // run once, when the sketch starts
{
  cli();
  if(MCUSR & _BV(WDRF)) {            // If a reset was caused by the Watchdog Timer...
     KillWDT();
  }
  sei();
  lcd.begin();                         // initialize the LCD

  // create the LCD special characters. Characters 0-5 are the various battery fullness icons
  // icon 7 is for the solar icon
  lcd.backlight();                     // turn on the backlight
  
  // create battery fullness icons
  for (int batchar = 0; batchar < 6; ++batchar) {
    lcd.createChar(batchar, battery_icons[batchar]);
  }
  lcd.createChar(SOLAR_ICON,solar_icon);
  Serial.begin(9600);                 // open the serial port at 9600 bps:
  digitalWrite(BACK_LIGHT_PIN,LOW);   // default LCd back light is OFF
  digitalWrite(RELAY_OUTPUT_PIN, LOW);       // relay is off to start

  // display the constant stuff on the LCD
  lcd.setCursor(0, 0);
  lcd.print("SOL");
  lcd.setCursor(4, 0);
  //lcd.write(SOLAR_ICON);
  lcd.setCursor(0, 2);
  lcd.print("BAT");
  // configure the watchdog with:
  // WDIE: Watchdog Interrupt Enable
  // WDCE: Watchdog Change Enable (because we are changing prescaler)
  // WDE:  Watchdog System Reset Enable
  // WDP2|WDP0: 0.5s
  cli();
  //WDTCSR |= (_BV(WDCE) | _BV(WDE)); // Enable the WD Change Bit
  WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
  WDTCSR =   _BV(WDIE)  |   // Enable WDT Interrupt
           _BV(WDP2) | _BV(WDP0);     // Set Timeout to 0.5 seconds
  sei();
}

//------------------------------------------------------------------------------------------------------
// Main loop
//------------------------------------------------------------------------------------------------------
void loop()
{
  read_data();
  enable_solar();
  print_data();
  lcd_display();
  alarm.SendVoltage(wdt, bat_volts);

  delay(50);                            // let everything settle

  if (alarm.IsChirping()) {
    wdt_reset();                         // no watchdog timeout
    delay(50);
  } else {                               // don't sleep if we are chirping
    if (wdt == 0) ++wdt;
    //set_sleep_mode(SLEEP_MODE_PWR_SAVE); // go into powersave mode
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // go into powersave mode

    cli();                               // no interrupts for this part
    sleep_enable();                      // wait for the watchdog
    sleep_bod_disable();                 // turn off brownout detector while sleeping
    sei();                               // turn on interrupts for watchdog
    sleep_cpu();                         // we will return after watchdog
    WDTCSR |= _BV(WDIE);                 // do another interrupt on next watchdog
    sleep_disable();                     // done sleeping
  }
}

void enable_solar() {

  if (charger_state == on) {
    if (bat_volts > BATT_FLOAT) {
      digitalWrite(RELAY_OUTPUT_PIN, LOW);
      charger_state = off;
    }
  } else {
    if (bat_volts < BATT_ON) {
      charger_state = on;
      digitalWrite(RELAY_OUTPUT_PIN, HIGH);
    }
  }
  
}

//------------------------------------------------------------------------------------------------------
// This routine reads and averages the analog inputs for this system, solar volts, solar amps and 
// battery volts. 
//------------------------------------------------------------------------------------------------------

long readVcc() {
  // Read 1.1V reference against AVcc
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(10); // Wait for Vref to settle
  //ADCSRA |= _BV(ADSC); // Convert
  //while (bit_is_set(ADCSRA,ADSC));
  //long result = ADCL;
  //result |= ADCH<<8;
  //result = 1058700L / result; // Back-calculate AVcc in mV
  //return result;

  int adc = interruptDrivenReadADC();
  return 1102353L / adc;
}
double readVcc2() { 
  long v = readVcc(); 
  return (Vcc + v/1000.0)/2;
}

int read_adc(int channel){
  
  int sum = 0;
  int temp;
  int i;

  // discard the first read
  temp = analogRead(channel);
  if (ADC_SAMPLES == 1)
      return temp;
  for (i=1; i<ADC_SAMPLES; i++) {          // loop through reading raw adc values ADC_SAMPLES number of times  
    delayMicroseconds(50);             // pauses for 50 microseconds
    temp = analogNoiseReducedRead(channel);        // read the input pin
    sum += temp;                       // store sum for averaging
  }
  return(sum / (ADC_SAMPLES-1));               // divide sum by ADC_SAMPLES to get average and return it
}

//------------------------------------------------------------------------------------------------------
// This routine reads all the analog input values for the system. Then it multiplies them by the scale
// factor to get actual value in volts or amps. 
//------------------------------------------------------------------------------------------------------
void read_data(void) {

  sol_amps_raw = read_adc(SOLAR_AMPS_CHAN);
  sol_amps =  (sol_amps_raw-511) * SOL_MA_SCALE;    //input of solar amps
  Vcc = readVcc2();
  sol_volts = read_adc(SOLAR_VOLTS_CHAN) * SOLAR_VOLTAGE_SCALE(Vcc);          //input of solar volts 
  bat_volts = read_adc(BATTERY_VOLTS_CHAN) * BATTERY_VOLTAGE_SCALE(Vcc);          //input of battery volts 
  sol_watts = sol_amps * sol_volts ;                               //calculations of solar watts                  
}

double bat_volts_sum;
long bat_volts_count;
//------------------------------------------------------------------------------------------------------
// This routine prints all the data out to the serial port.
//------------------------------------------------------------------------------------------------------
void print_data(void) {
  if (charger_state == on) {
    Serial.print(1);
  } else {
    Serial.print(0);
  }
  Serial.write(' ');
  //Serial.print("vcc=");
  Serial.print(Vcc, 3);
  Serial.write(' ');
  //Serial.print("sol_amps=");
  //Serial.print(sol_amps, 2);
  //Serial.write(' ');
  //Serial.print("raw=");
  Serial.print(sol_amps_raw-511, DEC);
  Serial.write(' ');
  //Serial.print("sol_volts=");
  Serial.print(sol_volts, 2);
  Serial.write(' ');
  //Serial.print(" bat_volts=");
  Serial.print(bat_volts, 2);
  Serial.write(' ');
  Serial.println(wdt);
}

//------------------------------------------------------------------------------------------------------
//-------------------------- LCD DISPLAY --------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------
void lcd_display()
{
  static bool current_backlight_state = -1;
  back_light_pin_State = digitalRead(BACK_LIGHT_PIN);
  if (current_backlight_state != back_light_pin_State) {
    current_backlight_state = back_light_pin_State;
    if (back_light_pin_State == HIGH)
      lcd.backlight();// finish with backlight on
    else
      lcd.noBacklight();
  }

  if (back_light_pin_State == HIGH)
  {
    backlight_time = millis();                        // If any of the buttons are pressed, save the time in millis to "time"
  }
 
 lcd.setCursor(0, 1);
 lcd.print(sol_volts);
 lcd.print("V ");
 //lcd.setCursor(0, 2);
 //lcd.print(sol_amps);
 //lcd.print("A ");  
 //lcd.setCursor(0, 3);
 //lcd.print(sol_watts);
 //lcd.print("W "); 
 //lcd.setCursor(8, 1);
 //lcd.print(bat_volts);
 //lcd.setCursor(8,2);

 /*if (charger_state == on) 
 lcd.print("on   ");
 else if (charger_state == off)
 lcd.print("off  ");
 else if (charger_state == bulk)
 lcd.print("bulk ");
 else if (charger_state == bat_float)
 {
 lcd.print("     ");
 lcd.setCursor(8,2);
 lcd.print("float");
 } */
 
 //-----------------------------------------------------------
 //--------------------Battery State Of Charge ---------------
 //-----------------------------------------------------------
 int pct = 100.0*(bat_volts - 11.3)/(12.7 - 11.3);
 if (pct < 0)
     pct = 0;
 else if (pct > 100)
     pct = 100;

/*
 lcd.setCursor(12,0);
 lcd.print((char)(pct*5/100));
*/

 lcd.setCursor(8,3);
 pct = pct - (pct%10);
 lcd.print(pct);
 lcd.print("%  ");
 
 //----------------------------------------------------------------------
 //------------------------Load Status-----------------------------------
 //----------------------------------------------------------------------
 lcd.setCursor(15,2);
 lcd.print("Solr");
 lcd.setCursor(15,3);
 if (load_status)
 {
    lcd.print("On  ");
 }
 else
 {
   lcd.print("Off ");
 } 
 spinner();
 backLight_timer();                      // call the backlight timer function in every loop 
}

void backLight_timer(){
  if((millis() - backlight_time) <= 15000)         // if it's been less than the 15 secs, turn the backlight on
      lcd.backlight();                   // finish with backlight on  
  else 
      lcd.noBacklight();                 // if it's been more than 15 secs, turn the backlight off
}
void spinner(void) {
  static int cspinner;
  static char spinner_chars[] = { '*','*', '*', ' ', ' '};
  cspinner++;
  lcd.print(spinner_chars[cspinner%sizeof(spinner_chars)]);
}

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int analogNoiseReducedRead(int pin)
{
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    if (pin >= 54) pin -= 54; // allow for channel or pin numbers
  #elif defined(__AVR_ATmega32U4__)
    if (pin >= 18) pin -= 18; // allow for channel or pin numbers
  #elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
    if (pin >= 24) pin -= 24; // allow for channel or pin numbers
  #elif defined(analogPinToChannel) && (defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__))
    pin = analogPinToChannel(pin);
  #else
    if (pin >= 14) pin -= 14; // allow for channel or pin numbers
  #endif
  
  #if defined(__AVR_ATmega32U4__)
    pin = analogPinToChannel(pin);
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
  #elif defined(ADCSRB) && defined(MUX5)
    // the MUX5 bit of ADCSRB selects whether we're reading from channels
    // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
  #endif
  
  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
  ADMUX = (DEFAULT << 6) | (pin & 0x07);
  delay(2);
  return interruptDrivenReadADC(); 
}
// NOTE: ADMUX must be set before you call this, and ADLAR should be 0
int interruptDrivenReadADC() {
  sbi(ADCSRA, ADIE);                 // interrupt at end of ADC
  set_sleep_mode(SLEEP_MODE_ADC);    // sleep during ADC w/interrupt
  sbi(ADCSRA, ADSC);                 // start a conversion

  do {
    sei();                           // enable interrupts
    sleep_cpu();                     // zzzzzz
    cli();                           // disable interrupts
  } while (bit_is_set(ADCSRA, ADSC));// wake for right reason?
  sleep_disable();                   // disable sleep
  _SFR_BYTE(ADCSRA) &= ~ _BV(ADIE);  // no interrupt at end of ADC
  sei();                             // enable interrupts

  // read the result
  uint8_t low = ADCL;                // low bit first
  uint8_t high = ADCH;
  return (high << 8) | low;
}

// If no ISR is provided, the default one resets the sketch
// We don't want to do anything in the ISR for an ADC
ISR(ADC_vect) 
{
}

ISR(WDT_vect)
{
  if (wdt == 0) {
    KillWDT();
  }
  wdt++;
  sleep_disable();
  sleep_enable();
}

