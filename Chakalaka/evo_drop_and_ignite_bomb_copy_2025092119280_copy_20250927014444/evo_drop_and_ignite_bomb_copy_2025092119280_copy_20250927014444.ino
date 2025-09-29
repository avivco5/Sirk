#define DEBUG 1

#include <FastLED.h>
#include <Servo.h>

/* ========= הגדרות לוגיות ========= */
const bool ARM_ARMED_WHEN_CLOSED    = true;   // true=דרוך כשסגור (LOW), false=כשפתוח (HIGH)
const bool TRIG_FIRE_ON_CLOSED_EDGE = true;   // מצית על קצה סגירה (LOW) אחרי דיבאונס

/* ========= פינים ========= */
const uint8_t LED_PIN         = 6;
const uint8_t FET_PIN         = 3;
const uint8_t SERVO_PIN       = 5;
const uint8_t ARM_SWITCH_PIN  = 7;   // microswitch, INPUT_PULLUP
const uint8_t TRIG_SWITCH_PIN = 9;   // microswitch, INPUT_PULLUP
const uint8_t LIGHT_PIN       = A0;

/* ========= רפרנס ========= */
const float VREF_VOLTS = 3.3; // אם משתמשים ב-AREF חיצוני 3.3V
// analogReference(EXTERNAL);

/* ========= NeoPixel ========= */
#define NUM_LEDS 3
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

/* ========= זמנים/ספים ========= */
int ADC_DARK  = 0;   // יוגדר בזמן כיול
int ADC_LIGHT = 0;   // יוגדר בזמן כיול
const uint8_t  LIGHT_SAMPLES = 8;
const uint32_t DEBOUNCE_MS   = 30;
const uint32_t REPORT_EVERY_MS = 600;
const uint32_t SERVO_SETTLE_MS   = 350;
const uint32_t MOSFET_ACTIVE_MS  = 3000;
const uint32_t BLINK_TIME_MS     = 500;
const uint32_t LOOP_INTERVAL_MS  = 30;

/* ========= זוויות סרוו ========= */
const int SERVO_LIGHT_ANGLE = 90;   // אור
const int SERVO_DARK_ANGLE  = 180;  // חושך

/* ========= גלובליים ========= */
Servo servoMotor;
unsigned long lastLoopMs=0, tStateStart=0, lastBlinkMs=0, lastReportMs=0;
bool blinkOn=false, fetOn=false;

int  lightRaw=0; bool lightOn=false, prevLightOn=false;
bool lightRising=false;         
bool seenDarkBeforeLight=false; 

bool armClosed=false, armClosedRaw=false, lastArmClosedRaw=false; unsigned long armDebT=0;
bool trigClosed=false, trigClosedRaw=false, lastTrigClosedRaw=false; unsigned long trigDebT=0;
bool edgeTrigClosed=false, edgeTrigOpen=false;  

int servoAngle = SERVO_DARK_ANGLE;

/* ---- Latch ---- */
bool latchedOpen = false;   
bool wasArmed    = false;   
uint32_t armCycles = 0;     

/* ========= מצבים ========= */
enum class State { IDLE, ARMED, SERVO_OPENING, READY, STROBE_ON, COMPLETE };
State state=State::IDLE;

/* ========= עוזרים ========= */
void setAll(const CRGB& c){ leds[0]=c; leds[1]=c; leds[2]=c; FastLED.show(); }
void strobeOn(){  fetOn=true;  digitalWrite(FET_PIN, HIGH); }
void strobeOff(){ fetOn=false; digitalWrite(FET_PIN, LOW);  }
void toState(State s){
  state=s; tStateStart=millis();
#if DEBUG
  const char* n[]={"IDLE","ARMED","SERVO_OPENING","READY","STROBE_ON","COMPLETE"};
  Serial.print(F("[STATE] ")); Serial.println(n[(int)s]);
#endif
}
int readLightAvg(){ long s=0; for(uint8_t i=0;i<LIGHT_SAMPLES;i++) s+=analogRead(LIGHT_PIN); return (int)(s/LIGHT_SAMPLES); }
bool lightHyst(bool prev, int a){ return prev ? (a > ADC_DARK) : (a >= ADC_LIGHT); }

/* ========= דיבאונס ========= */
void updateSwitches(){
  unsigned long now=millis();

  bool rawArmClosed = (digitalRead(ARM_SWITCH_PIN)==LOW); 
  if(rawArmClosed!=lastArmClosedRaw){ lastArmClosedRaw=rawArmClosed; armDebT=now; }
  if(now-armDebT>=DEBOUNCE_MS) armClosed = rawArmClosed;
  armClosedRaw = rawArmClosed;

  bool rawTrigClosed = (digitalRead(TRIG_SWITCH_PIN)==LOW); 
  if(rawTrigClosed!=lastTrigClosedRaw){ lastTrigClosedRaw=rawTrigClosed; trigDebT=now; }
  if(now-trigDebT>=DEBOUNCE_MS){
    if (!trigClosed && rawTrigClosed)  edgeTrigClosed = true; 
    if ( trigClosed && !rawTrigClosed) edgeTrigOpen   = true; 
    trigClosed = rawTrigClosed;
  }
  trigClosedRaw = rawTrigClosed;
}

/* ========= סטטוס ========= */
void printStatus(const __FlashStringHelper* why){
#if DEBUG
  float volts = (lightRaw * VREF_VOLTS) / 1023.0f;
  Serial.println(F("------------------------------------------------"));
  Serial.print (F("[WHY] "));      Serial.println(why);
  Serial.print (F("[ARM ] closed=")); Serial.print(armClosed?1:0);
  Serial.print (F("  armed="));
  Serial.println( ARM_ARMED_WHEN_CLOSED ? (armClosed?1:0) : (!armClosed?1:0) );
  Serial.print (F("[TRIG] closed=")); Serial.print(trigClosed?1:0);
  Serial.print (F("  edge(closed/open)=")); Serial.print(edgeTrigClosed?1:0); Serial.print('/'); Serial.println(edgeTrigOpen?1:0);
  Serial.print (F("[LIGHT] ADC=")); Serial.print(lightRaw);
  Serial.print (F("  V="));         Serial.print(volts,3);
  Serial.print (F("  lightOn="));   Serial.print(lightOn?1:0);
  Serial.print (F("  rising="));    Serial.println(lightRising?1:0);
  Serial.print (F("[SERVO] angle=")); Serial.print(servoAngle); Serial.println(F(" deg"));
  Serial.print (F("[FET ] "));      Serial.println(fetOn?F("ON"):F("OFF"));
  Serial.print (F("[LATCH] "));     Serial.print(latchedOpen?1:0);
  Serial.print (F("  cycles="));    Serial.println(armCycles);
  Serial.println(F("------------------------------------------------"));
#endif
}

/* ========= Setup ========= */
void setup(){
#if DEBUG
  Serial.begin(115200);
  delay(30);
  Serial.println(F("\n=== Boot ==="));
  Serial.print(F("Vref=")); Serial.println(VREF_VOLTS,3);
  Serial.print(F("ARM armed when ")); Serial.println(ARM_ARMED_WHEN_CLOSED?F("CLOSED(LOW)"):F("OPEN(HIGH)"));
  Serial.print(F("TRIG fires on ")); Serial.println(TRIG_FIRE_ON_CLOSED_EDGE?F("CLOSED edge(LOW)"):F("OPEN edge(HIGH)"));
#endif

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  setAll(CRGB::Blue); delay(200); setAll(CRGB::Black);

  pinMode(FET_PIN, OUTPUT); strobeOff();
  pinMode(ARM_SWITCH_PIN,  INPUT_PULLUP);
  pinMode(TRIG_SWITCH_PIN, INPUT_PULLUP);

  servoMotor.attach(SERVO_PIN);
  servoAngle = SERVO_DARK_ANGLE;    
  servoMotor.write(servoAngle);

  // === כיול חיישן אור ===
  int baseline = readLightAvg();
  ADC_DARK  = baseline + 200;      
  ADC_LIGHT = ADC_DARK + 550;      
#if DEBUG
  Serial.print(F("Light calibration: baseline=")); Serial.print(baseline);
  Serial.print(F("  ADC_DARK=")); Serial.print(ADC_DARK);
  Serial.print(F("  ADC_LIGHT=")); Serial.println(ADC_LIGHT);
#endif

  lightRaw   = readLightAvg();
  lightOn    = lightHyst(false, lightRaw);
  seenDarkBeforeLight = !lightOn;     
  prevLightOn = lightOn;

  toState(State::IDLE);
  printStatus(F("Boot"));
}

/* ========= Loop ========= */
void loop(){
  unsigned long now=millis();
  if(now-lastLoopMs<LOOP_INTERVAL_MS) return;
  lastLoopMs=now;

  bool prevFET=fetOn; int prevAngle=servoAngle; State prevState=state;
  updateSwitches();

  bool isArmed = ARM_ARMED_WHEN_CLOSED ? armClosed : !armClosed;

  lightRaw   = readLightAvg();
  prevLightOn = lightOn;
  lightOn    = lightHyst(lightOn, lightRaw);
  lightRising = (!prevLightOn && lightOn);

  leds[0] = lightOn ? CRGB::Yellow : CRGB::Black;

  if (isArmed && !wasArmed) {
    latchedOpen = false;
    seenDarkBeforeLight = !lightOn;   
  }
  if (!isArmed && wasArmed) {
    latchedOpen = false;
    armCycles++;
    seenDarkBeforeLight = false;
  }
  wasArmed = isArmed;

  if(!isArmed){
    strobeOff(); fetOn=false;
    servoAngle  = SERVO_DARK_ANGLE;  
    servoMotor.write(servoAngle);

    if(now-lastBlinkMs>=BLINK_TIME_MS){ lastBlinkMs=now; blinkOn=!blinkOn; }
    leds[1]=leds[2]= blinkOn?CRGB::Red:CRGB::Black;
    FastLED.show();

    if(state!=State::IDLE) toState(State::IDLE);
    edgeTrigClosed=false; edgeTrigOpen=false;

    if (prevFET!=fetOn || prevAngle!=servoAngle || prevState!=state) printStatus(F("Not armed"));
    if (now-lastReportMs>=REPORT_EVERY_MS){ lastReportMs=now; printStatus(F("Periodic (not armed)")); }
    return;
  }

  leds[1]=leds[2]=CRGB::Black;

  switch(state){
    case State::IDLE:
      toState(State::ARMED);
      break;

    case State::ARMED:
      if (!seenDarkBeforeLight && !lightOn) {
        seenDarkBeforeLight = true;
      }
      servoAngle = lightOn ? SERVO_LIGHT_ANGLE : SERVO_DARK_ANGLE;
      servoMotor.write(servoAngle);

      if (seenDarkBeforeLight && lightRising) {
        latchedOpen = true;
        servoAngle = SERVO_LIGHT_ANGLE;
        servoMotor.write(servoAngle);
        leds[1]=leds[2]=CRGB::Yellow; FastLED.show();
        edgeTrigClosed=false; edgeTrigOpen=false;
        toState(State::SERVO_OPENING);
      }
      break;

    case State::SERVO_OPENING:
      if(now-tStateStart >= SERVO_SETTLE_MS){
        edgeTrigClosed=false; edgeTrigOpen=false;
        leds[1]=leds[2]=CRGB::Blue; FastLED.show();
        toState(State::READY);
      }
      servoAngle = SERVO_LIGHT_ANGLE; servoMotor.write(servoAngle);
      break;

    case State::READY:
      servoAngle = SERVO_LIGHT_ANGLE; servoMotor.write(servoAngle);
      if ( (TRIG_FIRE_ON_CLOSED_EDGE && edgeTrigClosed) ||
           (!TRIG_FIRE_ON_CLOSED_EDGE && edgeTrigOpen) ) {
        edgeTrigClosed=false; edgeTrigOpen=false;
        strobeOn();
        leds[1]=leds[2]=CRGB::White; FastLED.show();
        toState(State::STROBE_ON);
      }
      break;

    case State::STROBE_ON:
      if(now-tStateStart >= MOSFET_ACTIVE_MS){
        strobeOff();
        leds[1]=leds[2]=CRGB::Green; FastLED.show();
        toState(State::COMPLETE);
      }
      servoAngle = SERVO_LIGHT_ANGLE; servoMotor.write(servoAngle);
      break;

    case State::COMPLETE:
      servoAngle = SERVO_LIGHT_ANGLE; servoMotor.write(servoAngle);
      break;
  }

  FastLED.show();

  if (prevFET!=fetOn || prevAngle!=servoAngle || prevState!=state) printStatus(F("Edge change"));
  if (now-lastReportMs>=REPORT_EVERY_MS){ lastReportMs=now; printStatus(F("Periodic")); }

  edgeTrigClosed=false; edgeTrigOpen=false;
}
