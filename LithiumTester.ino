
// for use with charger module with 4056 chip
// mods needed:
//   cut enable pin - 8
//   solder lead to both pin 8 as the control line
// the module also has a DW01A protection chip and MOSFETS to do additional over/under volts and
// overdischarge current. internal wiring for clarity:
// +in -> chargecontrol -> +out/+batt   (+out and +batt tied)
// -in/-out -> protectionMOSFETs -> -batt  (-in and -out tied)
// for discharge +batt -> R -> MOSFET -> -out
// so the volts measure on the battery will include a tiny effect from the protection MOSFETs
// however this prevents accidental over discharge..
//
// the 4056 controls the charge up to 4.2 volts
// arduino directly measures the cell voltage
// discharge is via an output to gate IRF520N mosfet and a series resistor
// discharge current is implied from voltage and resistor

// arduino connections
// digital pins run from 2 in pairs charge enable, discharge:
// analogue pin from 0

// all volts stored in mv as integer - timers in seconds
//
// as a state engine see state enumeration
// for commmands see processserial()
//
// code changes for dev2:
//  reduce fails to number codes - memory
//  extend error testing
//  merge some settings - pause and interval
//  add energy as AH
//  add quick test
//  limit state change to 1/sec - avoid charge supply spikes

// use a watchdog. used to perform reset
#include "Watchdog.h"
Watchdog watchdog;

/// strings.. limited ram so ned to manage
const String stateNotOff = "!!state<>off";
const char c = ',';

// configuration..
const byte numCells = 6;                            // cells in rig
const byte anaSamples = 5;                          // N slot ring buffer for adding raw analog readings each 1 sec loop
const byte samplesPerSec = 5;                       // samples read per second added to each ring buffer slot.
const float anaDivider = anaSamples * samplesPerSec; // for dividing sum of ring buffer by for average raw value
const float voltConvert =  4.11 / 981.0 * 1000.0 / anaDivider;  // from sample reading = volts / raw analogue * 1000 to give mv
const int dischargeVoltAdjust = 92;                 // V diff measured between cell and analog input points at 3.7v pause 
const float resistor[6] = {4.004, 4.147, 4.147, 4.113, 4.114, 4.124}; // resistors = one 8+8 rest 8+8.2 - from volt/current measurement at 3.7V

// test
//const int intervalTime = 15;            // max interval including pause time
//const int pauseTime = anaSamples + 2;   // pause time in charge/discharge to check volt drop

const int intervalTime = 180;                 // max interval including pause time
const int pauseTime = anaSamples + 5;         // pause time in charge/discharge to check volt drop
const int maxChargeDelta = 300;               // max allowable charge delta value - indicates suspect charge/cell/connection
const int dischargeMidVolts = 3700;           // paused delta volts captured at mid discharge
const int dischargeEndVolts = 3000 - dischargeVoltAdjust*3000.0/dischargeMidVolts;   // discharge stop volts - adjusted
const int storeVoltsDefault = 3800;           // storage charge level
const int buildVoltsDefault = 3500;           // alternative store level bring to charge state for building packs
const int storeVoltsOvershoot = 20;           // overshoot target voltage to allow for volts settle
const int storeExtraPauseTime = 180;          // stretch pause at end store to allow settle
const int minStartVolts = 3000;               // min volts to start charge or discharge
const int statePauseTime = 180;               // delay time between switching states

// state enumeration
const byte stDisable = 0;                 // can disable to supress logging
const byte stOff = 1;                     // off
const byte stCharge = 2;                  // charging phase with charge end tests
const byte stDischarge = 3;               // discharge
const byte stStoreCharge = 4;             // recover to store volts charging
const byte stStoreDischarge = 5;          // recover to store volts discharging
const byte stTest = 6;                    // quick volt drop test
const byte stFail = 7;                    // failed for some reason

// error codes - too many to offer strings - use codes
const byte errLowVoltsCharge = 1;           // low volt start charge
const byte errLowVoltsDischarge = 2;        // low volt start discharge
const byte errLowVoltsStoreCharge = 3;      // low volt start store charge
const byte errChargeStall = 4;              // no pause volts change AND no delta volts change
const byte errChargeDeltaVolts = 5;         // pause delta volts > limit
const byte errDischargeDeltaVolts = 6;      // pause delta volts > limit
const byte errDischargeVoltsCollapse = 7;   // volts drop > expeted - connection?
const byte errDischargeVoltsIncrease = 8;   // volts increased pause to pause - connection?

// data per cell..
struct Cell
{
  public:
    byte ix;                              // index + analog pin
    int chargePin()
    {
      return 2 + ix + ix;                 // charge io pin
    };
    int dischargePin()
    {
      return 3 + ix + ix;                 // discharge out pin
    };
    unsigned int anaReadings[anaSamples]; // one per second - each samples/sec summed each second
    unsigned int lastAnalogRaw;           // last value read
    int volts;                            // averaged volts
    float resistor;                       // discharge resistor value
    byte state;                           // current state
    bool charge;                          // is charging
    bool discharge;                       // is discharging
    bool paused;                          // flags whan paused - used by logging
    float ah;                             // discharge AH
    float wh;                             // discharge watt hours
    unsigned int waitTill;                // wait timer - seconds.
    int storeLongPause;                   // on final store pause extend to 60 sec
    int pauseVolts;                       // volts at start of pause
    int prevPauseVolts;                   // previous volts at start of pause
    int chargeDeltaVolts;                 // volt drop after pausing charge
    int dischargeDeltaVolts;              // ... and difference after pause time
    int dischargeMidDeltaVolts;           // end discharge - volt recovery
    int storeChargeFinishVolts;           // target to charge to to hit storeVolts
    unsigned int elapsed;                 // elapsed seconds for current operation
    bool doCharge;                        // charge pending
    bool doDischarge;                     // discharge pending
    bool doStoreCharge;                   // store charge pending
    bool doTest;                          // quick discharge delta test
    void off()                            // switch to off
    {
      state = stOff;
      doCharge = false;
      doDischarge = false;
      doStoreCharge = false;
      digitalWrite(dischargePin(), false);
      digitalWrite(chargePin(), false);
    }
    byte errCode;
    void fail(byte inErrCode)
    {
      errCode = inErrCode;
      off();
      state = stFail;
    }
    String failMsg()
    {
      if (errCode == 0)
      {
        return "ok";
      }
      return ("!!" + String(errCode));
    }
    int elapsedMin()  // elapsed integer minutes
    {
      return elapsed/60;
    }
};

// variables..
Cell cells[numCells];                     // the cells
// currently selected cell
unsigned long nextSecond;                 // timing - millis() at next second
unsigned long nextVoltsRead;              // timing - millis() at next analogue read
unsigned int ts;                          // time counter in seconds - for all wait timing
int anaArrIx = 0;                         // analog ring buffer voltArr index
unsigned int secCounter;                  // second counter for counting minutes
int storeVolts =  storeVoltsDefault;      // set by command - can be set to buildVoltsDefault
byte stateChanges = 0;                    // to limit state change per sec - avoid current spikes
bool forceLog = false;                    // to force a log on commmand

// state string values for log
String stateS(byte iS)
{
  switch (iS)
  {
    case stOff:      
      return "off";
    case stCharge:   
      return "chg";
    case stDischarge: 
      return "dis";
    case stStoreCharge: 
      return "sto";
    case stStoreDischarge: 
      return "std";
    case stTest: 
      return "tst";
    case stFail: 
     return "fai";
  }
  return "xxx";
}

// write to log
void log(String s)
{
  Serial.println(s);
}

// log prefix with cell id s= status change
void logCell(Cell *cell, String s)
{
  log("s,c:" + String(cell->ix + 1) + c + s);
}
// log prefix with cell id l= log
void logCell2(Cell *cell, String s)
{
  log("l,c:" + String(cell->ix + 1) + c + s);
}

// convert to x.xxx
String stringK(int val)
{
  return String(val / 1000.0, 3);
}
// v=x.xxx   for reporting volts
String stringKV(int val)
{
  return " v=" + stringK(val);
}
// d=xxx     for reporting millivolt delta
String stringDmv(int val)
{
  return " d=" + String(val);
}

void setup()
{
  Serial.begin(9600);           //  setup serial
  analogReference(EXTERNAL);
  const char compile_date[] = "lithiumtester "__DATE__ " " __TIME__;
  log(compile_date);
  log("start setup..");
  for (int ic = 0; ic < numCells; ic++)
  {
    Cell *cell;
    cell = &cells[ic];
    cell->ix = ic;
    
    // configure output pins
    pinMode(cell->dischargePin(), OUTPUT);
    pinMode(cell->chargePin(), OUTPUT);

    cell->off();
    // and resistors
    cell->resistor = resistor[ic];
    for (int ix = 0; ix < anaSamples; ix++)
    {
      cell->anaReadings[ix] = 0;
    }
  }
  // flash leds on chargers for visual
  for (int ic = 0; ic < numCells; ic++)
  {
    Cell *cell;
    cell = &cells[ic];
    digitalWrite(cell->chargePin(), true);
    delay(200);
  }
  for (int ic = 0; ic < numCells; ic++)
  {
    Cell *cell;
    cell = &cells[ic];
    digitalWrite(cell->chargePin(), false);
    delay(200);
  }
  // start timers
  unsigned long tMillis = millis();
  nextVoltsRead = tMillis + 1000 / samplesPerSec / 2;
  nextSecond = tMillis + 1000;
  secCounter = 0;
  ts = 0;

  //watchdog
  watchdog.enable(Watchdog::TIMEOUT_2S);

  log("setup done");
}

// writes last analog raw alues to log - for calibration
void logAnalogRaw()
{
  String s;
  for (int ic = 0; ic < numCells; ic++)
  {
    Cell *cell;
    cell = &cells[ic];
    s += String(cell->lastAnalogRaw) + " ";
  }
  log(s);
}

// process each cell on 1 sec timer
void doCell(Cell *cell)
{
  // total up raw readings
  long voltSum = 0;
  for (int ix = 0; ix < anaSamples; ix++)
  {
    voltSum += cell->anaReadings[ix];
  }
  // calculate volts
  cell->volts = int(float(voltSum) * voltConvert);

  cell->charge = false;
  cell->discharge = false;
  cell->paused = false;

  switch (cell->state)
  {
    // in stOff check pending states to change to
    case stOff:
      {
        if (ts < cell->waitTill)
        {
          // enforced wait for state change
        }
        else if (stateChanges <= 0)
        {
          // only one allowed per main loop
        }
        else if (cell->doCharge)
        {
          stateChanges--;
          cell->doCharge = false;
          if (cell->volts < minStartVolts)
          {
            cell->fail(errLowVoltsCharge);
            logCell(cell, cell->failMsg());
          }
          else
          {
            cell->state = stCharge;
            cell->storeLongPause = 0;         // borrow as a counter
            cell->chargeDeltaVolts = 0;
            cell->pauseVolts = cell->volts;
            cell->elapsed = 0;
            cell->waitTill = ts + intervalTime;
            cell->errCode = 0;
            logCell(cell, stateS(cell->state) + stringKV(cell->volts));
          }
        }
        else if (cell->doDischarge)
        {
          stateChanges--;
          cell->doDischarge = false;
          if (cell->volts < minStartVolts)
          {
            cell->fail(errLowVoltsDischarge);
            logCell(cell, cell->failMsg());
          }
          else
          {
            cell->state = stDischarge;
            cell->elapsed = 0;
            cell->ah = 0;
            cell->wh = 0;
            cell->pauseVolts = cell->volts;
            cell->dischargeMidDeltaVolts = 0;
            cell->waitTill = ts + intervalTime;
            cell->errCode = 0;
            logCell(cell, stateS(cell->state) + stringKV(cell->volts));
          }
        }
        else if (cell->doStoreCharge)
        {
          stateChanges--;
          cell->doStoreCharge = false;
          if (cell->volts < minStartVolts)
          {
            cell->fail(errLowVoltsStoreCharge);
            logCell(cell, cell->failMsg());
          }
          else
          {
            cell->elapsed = 0;
            cell->storeChargeFinishVolts = storeVolts;
            cell->storeLongPause = storeExtraPauseTime;         // extra seconds on final store pause
            cell->waitTill = 0;
            cell->errCode = 0;
            if (cell->volts > storeVolts)
            {
              cell->state = stStoreDischarge;
            }
            else
            {
              cell->state = stStoreCharge;
            }
            logCell(cell, stateS(cell->state) + stringKV(cell->volts) + " to" + stringKV(cell->storeChargeFinishVolts));
          }
        }
        else if (cell->doTest)
        {
          stateChanges--;
          cell->doTest = false;
          cell->waitTill = ts + 10;
          cell->state = stTest;
          cell->discharge = true;
          cell->pauseVolts = cell->volts;          // is volts BEFORE discharge starts
        }
      }
      break;

    case stTest:
      {
        // after 10 sec discharge measure diff and report
        if (ts < cell->waitTill)
        {
          cell->discharge = true;
        }
        else
        {
          int adjust = int(float(dischargeVoltAdjust) * float(cell->volts) / float(dischargeMidVolts));  
          int delta = cell->pauseVolts - cell->volts - adjust;
          logCell(cell, stateS(cell->state) + " d=" + String(delta));
          cell->state = stOff;
        }
        break;
      }

    case stCharge:
      {
        if (ts < cell->waitTill - pauseTime)
        {
          // charging
          cell->elapsed++;
          cell->charge = true;
          if (cell->pauseVolts != 0)
          {
            // just starting charge - still in pause
            cell->prevPauseVolts = cell->pauseVolts;
            cell-> pauseVolts = 0;
          }
        }
        else if (ts >= cell->waitTill)
        {
          // end of charge pause
          if (cell->volts >= cell->pauseVolts - 4)
          {
            // volts has dropped <=4mv in pause so assume charge not engaging - assume charged
            logCell(cell, stateS(cell->state) +"-end: " + stringKV(cell->volts) + " d=" + String(cell->chargeDeltaVolts)+ " t=" + String(cell->elapsedMin()));
            cell->state = stOff;
            cell->waitTill = ts + statePauseTime;
          }
          else
          {
            // volts dropped by > 1mv - still charging
            int deltaVolts = cell->pauseVolts - cell->volts; // capture last delta volts for log
            // quality check - should see either pause volts increase of delta volts decrease if ot count consecutive instances to 2
            int voltChange = cell->pauseVolts - cell->prevPauseVolts;
            int deltaChange = deltaVolts - cell->chargeDeltaVolts;
            cell->chargeDeltaVolts = deltaVolts;
            if (voltChange <=0 && deltaChange >=0)    
            {
              cell->storeLongPause++;
            }
            else
            {
              cell->storeLongPause = 0;
            }
            if (cell->storeLongPause >= 2)
            {
              cell->fail(errChargeStall);
              logCell(cell,cell->failMsg());
            }
            else if (cell->chargeDeltaVolts > maxChargeDelta)
            {
              cell->fail(errChargeDeltaVolts);
              logCell(cell,cell->failMsg());
            }
            else
            {
              // charge again - 
              cell->waitTill = ts + intervalTime;
              cell->charge = true;
            }
          }
        }
        else
        {
          // paused
          if (cell-> pauseVolts == 0)
          {
            cell->pauseVolts = cell->volts;
          }
          cell->paused = true;
        }
      }
      break;

    case stDischarge:
      {
        // discharge to end volts pausing at intervals to capture delta volts
        if (ts < cell->waitTill - pauseTime)
        {
          // discharging
          if (cell->pauseVolts != 0)
          {
            cell->prevPauseVolts = cell->pauseVolts;
            cell-> pauseVolts = 0;
          }
          int adjust = int(float(dischargeVoltAdjust) * float(cell->volts) / float(dischargeMidVolts));
          float volts = float(cell->volts)/1000.0;
          float adjVolts = float(cell->volts + adjust)/1000.0;
          float amps = volts / cell->resistor;
          cell->ah += amps / 3600.0;
          cell->wh += amps * adjVolts / 3600.0;
          if (cell->volts <= dischargeEndVolts-50)
          {
            // must have dropped too much - connection
            cell->fail(errDischargeVoltsCollapse);
            logCell(cell,cell->failMsg());
          }
          else if (cell->volts <= dischargeEndVolts)
          {
            // is discharged
            logCell(cell, stateS(cell->state) +"-end: " + stringKV(cell->volts) + " t=" + String(cell->elapsedMin()) + " ah=" + String(cell->ah,3) + " wh=" + String(cell->wh,2));
            cell->state = stOff;
            cell->waitTill = ts + statePauseTime;
          }
          else
          {
            // leave discharging
            cell->discharge = true; 
            cell->elapsed++;
          }
        }
        else if (ts >= cell->waitTill)
        {
          // end pause - difference between on discharge at start pause and off discharge at end pause
          //logCell(cell,String(cell->volts));
          cell->dischargeDeltaVolts = cell->volts - cell->pauseVolts;
         
          if (cell->dischargeDeltaVolts > maxChargeDelta)
          {
            cell->fail(errDischargeDeltaVolts);
            logCell(cell,cell->failMsg());
          }
          
          cell->waitTill = ts + intervalTime;
          if (cell->dischargeMidDeltaVolts == 0 && cell->volts <= dischargeMidVolts)
          {
            cell->dischargeMidDeltaVolts = cell->dischargeDeltaVolts;  // capture mid discharge value
          }
          cell->discharge = true;
        }
        else
        {
          // paused
          cell->paused = true;
          if (cell->pauseVolts == 0)
          {
            int adjust = int(float(dischargeVoltAdjust) * float(cell->volts) / float(dischargeMidVolts));
            cell->pauseVolts = cell->volts + adjust;
            int voltDrop = cell->prevPauseVolts - cell->pauseVolts;   // volts should decrease - if goes up something wrong. allow 10mv lift
            if (voltDrop < -10)
            {
              //logCell(cell,String(cell->prevPauseVolts) + " " + String(cell->pauseVolts));
              cell->fail(errDischargeVoltsIncrease);
              logCell(cell,cell->failMsg() + String(voltDrop));
            }
          }
        }
      }
      break;

    case stStoreCharge:
      {
        // charge to target voltage. at pause interval check volt drop and adjust target voltage
        if (ts < cell->waitTill - pauseTime)
        {
          // charging time
          if (cell->pauseVolts != 0)
          {
            cell->prevPauseVolts = cell->pauseVolts;
            cell-> pauseVolts = 0;
          }
          cell->charge = true;
          cell->elapsed++;
          if (cell->volts >= cell->storeChargeFinishVolts)
          {
            // hit finished volts - bring forward to pause time for next pass
            cell->waitTill = ts + pauseTime;
          }
        }
        else if (ts >= cell->waitTill)
        {
          //end pause - hit store volts?
          if (cell->volts > storeVolts) 
          {
            if (cell->storeLongPause > 0)
            {
              cell->waitTill++;                         // use up extra time in pause - 
              cell->storeLongPause--;
              cell->paused = true;
            }
            else
            {
              logCell(cell, stateS(cell->state) +"-end: " + stringKV(cell->volts)+ " t=" + String(cell->elapsedMin()));
              cell->state = stOff;
              cell->waitTill = ts + statePauseTime;
            }
          }
          else
          {
            cell->storeChargeFinishVolts = storeVolts + cell->pauseVolts - cell->volts + storeVoltsOvershoot;
            cell->waitTill = ts + intervalTime;
            cell->charge = true;
          }
        }
        else
        {
          // in pause
          if (cell->pauseVolts == 0)
          {
            cell->pauseVolts = cell->volts;     // capture start pause volts
          }
          cell->paused = true;
        }
      }
      break;

     case stStoreDischarge:
      {
        // opposite of store charge - but brings volts down to store level
        // discharge to target voltage. at pause interval check volt drop and adjust target voltage
        if (ts < cell->waitTill - pauseTime)
        {
          // discharging time
          if (cell->pauseVolts != 0)
          {
            cell->prevPauseVolts = cell->pauseVolts;
            cell-> pauseVolts = 0;
          }
          cell->discharge = true;
          cell->elapsed++;
          if (cell->volts <= cell->storeChargeFinishVolts)
          {
            // hit finished volts - bring forward to pause time for next pass
            cell->waitTill = ts + pauseTime;
          }
        }
        else if (ts >= cell->waitTill)
        {
          //end pause - hit store volts?
          if (cell->volts < storeVolts) 
          {
            if (cell->storeLongPause > 0)
            {
              cell->waitTill++;                         // use up extra time in pause - 
              cell->storeLongPause--;
              cell->paused = true;
            }
            else
            {
              logCell(cell, stateS(cell->state) +"-end: " + stringKV(cell->volts)+ " t=" + String(cell->elapsedMin()));
              cell->state = stOff;
              cell->waitTill = ts + statePauseTime;
            }
          }
          else
          {
            cell->storeChargeFinishVolts = storeVolts + cell->pauseVolts - cell->volts - storeVoltsOvershoot;
            cell->waitTill = ts + intervalTime;
            cell->discharge = true;
          }
        }
        else
        {
          // in pause
          if (cell->pauseVolts == 0)
          {
            cell->pauseVolts = cell->volts;     // capture start pause volts
          }
          cell->paused = true;
        }
      }
      break;
  }

  // set charge and discharge outputs
  digitalWrite(cell->dischargePin(), cell->discharge);
  digitalWrite(cell->chargePin(), cell->charge);
}

void processSerial()
{
  // commands are single character processed in order - see below under ? command
  // for example b1f2f3f4f5f6f will set to build volts and start full cycle on all cells
  int cellIx = -1;

  while (Serial.available() > 0)
  {
    bool done = false;
    char k = Serial.read();
    if (k >= char('0') && k <= char('9'))
    {
      cellIx = k - char('0');
      if (cellIx < 1 || cellIx > numCells)
      {
        log("!!invalid cell=" + String(k));
        return;
      }
      done = true;
    }
    if (!done)
    {
      switch (k)
      {
        case '\r':
          done = true;
          break;
        case '\n':
          done = true;
          break;

        case '?':
          log("<n> sel 0-" + String(numCells));    // N select cell to operate on 1-6
          log("log");                              // l forces a log output now
          log("full cycle");                       // Nf schedules charge, discharge, store charge
          log("chg");                              // Nc - charge only
          log("disch");                            // Nd - discharge only
          log("store");                            // Ns - does a charge to store volts or build volts
          log("xit");                              // Nx - exit - sets cell back to off
          log("z-disabl");                         // Nz - disables that cell so does not log
          log("ana raw");                          // a - shows analog raw values
          log("*-reset");                          // * - causes restet after 2 seconds
          log("build");                            // b switches build volts between 3500 and 3800
          log("test");                             // fast discharge delta test 
          log("rv=" + String(storeVolts));
          done = true;
          break;

        case '*':
          // cause watchdog timeout..
          log("reset..");
          delay(3000);
          done = true;
          break;

        case 'l':
          forceLog = true;
          done = true;
          break;

        case 'a':
          logAnalogRaw();
          done = true;
          break;

        case 'b':
          if (storeVolts == buildVoltsDefault)
          {
            storeVolts = storeVoltsDefault;
          }
          else
          {
            storeVolts = buildVoltsDefault;
          }
          log("rv=" + String(storeVolts));
          done = true;
          break;
      }
    }
    // following need a selected cell...
    if (!done)
    {
      if (cellIx == -1)
      {
        log("!!no cell selected");
        return;
      }
      Cell *cell;
      cell = &cells[cellIx - 1];
      switch (k)
      {
        case 'z':
          if (cell->state == stDisable)
          {
            cell->state = stOff;
          }
          else if (cell->state == stOff)
          {
            cell->state = stDisable;
          }
          break;

        case 'f':
          if (cell->state == stOff)
          {
            cell->waitTill = ts;
            cell->doCharge = true;
            cell->doDischarge = true;
            cell->doStoreCharge = true;
          }
          else
          {
            logCell(cell, stateNotOff);
          }
          break;

        case 'c':
          if (cell->state == stOff)
          {
            cell->waitTill = ts;
            cell->doCharge = true;
          }
          else
          {
            logCell(cell, stateNotOff);
          }
          break;

        case 'd':
          if (cell->state == stOff)
          {
            cell->waitTill = ts;
            cell->doDischarge = true;
          }
          else
          {
            logCell(cell, stateNotOff);
          }
          break;

        case 's':
          if (cell->state == stOff)
          {
            cell->waitTill = ts;
            cell->doStoreCharge = true;
          }
          else
          {
            logCell(cell, stateNotOff);
          }
          break;

        case 't':
          if (cell->state == stOff)
          {
            cell->waitTill = ts;
            cell->doTest = true;
          }
          else
          {
            logCell(cell, stateNotOff);
          }
          break;

        case 'x':
          cell->off();
          logCell(cell, "off");
          break;

        default:
          log("!! '" + String(k) + "' ??");
          return;
      }
    }
  }
}

void loop()
{
  // process serial.
  if (Serial.available() > 0)
  {
    delay(100);        // wait all characters in
    processSerial();
    while (Serial.available() > 0)    // spill any remaining in case error
    {
      Serial.read();
    }
  }

  // timers
  unsigned long tmillis = millis();

  // read analogues at samplesPerSec
  while (tmillis >= nextVoltsRead && tmillis < nextSecond)
  {
    for (int ic = 0; ic < numCells; ic++)
    {
      Cell *cell = &cells[ic];
      cell->lastAnalogRaw = analogRead(cell->ix);
      cell->anaReadings[anaArrIx] += cell->lastAnalogRaw;
    }
    nextVoltsRead += 1000 / samplesPerSec;
  }

  if (tmillis >= nextSecond)
  {
    bool loggit = false;
    watchdog.reset();
    ts = ts + 1;
    nextVoltsRead = nextSecond + 10;
    nextSecond += 1000;
    secCounter++;
    if (secCounter >= 60)
    {
      secCounter = 0;
      loggit = true;
    }
    
    // process each cell
    stateChanges = 1;    // only 1 per main loop
    for (int ic = 0; ic < numCells; ic++)
    {
      doCell(&cells[ic]);
    }

    // step index on ring buffer
    anaArrIx++;
    if (anaArrIx >= anaSamples)
    {
      anaArrIx = 0;
    }
    for (int ic = 0; ic < numCells; ic++)
    {
      cells[ic].anaReadings[anaArrIx] = 0;
    }

    for (int ic = 0; ic < numCells; ic++)
    {
      Cell *cell = &cells[ic];
      if ((cell->state >= stOff && loggit) || (cell->state >= stOff && forceLog) ) 
      {
        int showVolts = cell->volts;
        if (cell->paused)
        {
          showVolts = cell->pauseVolts;
        }
        String state = stateS(cell->state);
        if (cell->state == stOff && (cell->doCharge || cell->doDischarge || cell->doStoreCharge))
        {
          state = "pnd";  //have pending operations 
        }
        
        logCell2(cell, String(ts / 60)
                 + c + String(cell->elapsedMin()) 
                 + c + state
                 + c + stringK(showVolts)
                 + c + String(cell->ah,3)
                 + c + String(cell->wh,2)
                 + c + String(cell->chargeDeltaVolts)
                 + c + String(cell->dischargeDeltaVolts)
                 + c + String(cell->dischargeMidDeltaVolts)
                 + c + String(cell->failMsg())
                   );
      }
    }
    forceLog = false;
  }
}
