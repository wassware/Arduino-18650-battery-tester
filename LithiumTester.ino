
// for use with charger module with 4056 chip
// mods needed:
//   cut enable pin - 8
//   solder lead to both pins 6 (inverse of charge complete) and 8 as the control line
// the control line read is
//   high = charging  - this keps enable high
//   low = charged - this takes enable low
// seting arduino pin to output briefly and setting new state low or high has
// enough umph to flip flop the charge state
// the module also has a DW01A protection chip and MOSFETS to do additional over/under volts and
// overdischarge current. internal wiring for clarity:
// +in -> chargecontrol -> +out/+batt   (+out and +batt tied)
// -in/-out -> protectionMOSFETs -> -batt  (-in and -out tied)
// for discharge +batt -> R -> MOSFET -> -out
// so the volts measure on the battery will include a tiny effect from the protectionMOSFETs
// however this prevents accidental over discharge..
//
// the 4056 controls the charge up to 4.2 volts
// arduino directly measures the cell voltage
// discharge is via an output to gate IRF520N mosfet and a series resistor
// discharge current is implied from voltage

// arduino connections
// digital pins run from 2 in pairs control state, discharge:
// analogue pin from 0

// all volts im mv

const String stateNotOff = "!! state <> off";
//const String voltsUnderLimit = "!!volts under limit";

const byte numCells = 6;            // cells in rig
const byte anaSamples = 10;         // 10 slot cycle buffer - 10 samples per slot
const byte samplesPerSec = 10;        // samples read per second added to each buffer slot.
const byte anaDivider = anaSamples * samplesPerSec; 
const double resistorDefault = 4.0;
const double defaultVoltConvert =  4.11 / 1006 * 1000.0;
const int dischargeRawAdjust = 60 / defaultVoltConvert;

int chargeTestVolts = 5000;       // test mode - lower test charge
int chargeRestTime = 60;          // after charging - seconds before discharge
int dischargeTestVolts = 3700;  // pause on discharge to test how volts recover.
int dischargeTestTime = 60;       // .. for how long - same at end discharge
int dischargeEndVolts = 3000;         // discharge stop volts
int storeVolts = 3800;          // storage charge default
int chargeTestTime = 60;        // during store charge - rest time before volt check in recharge-to loop
bool showAnaRaw = false; 

const char c = ',';

// state
const byte stDisable = 0;        // initially disabled until check controls work
const byte stOff = 1;            // off
const byte stCharge = 2;         // charging= start charge - checks voltage
const byte stChargePause = 3;    // holds after charging to check volts drop
const byte stDischarge1 = 4;     // discharge stage 1
const byte stDischPause1 = 5;  // discharge pause = check volt rise
const byte stDischarge2 = 6;     // discharge stage 1
const byte stDischPause2 = 7;   // pause before recharge - check volt lift
const byte stStoreCharge = 8; // recover to standby volts charging
const byte stStoreChargePause = 9;   // recover to standby volts resting

// data per cell..
struct Cell
{
  public:
   
    byte ix;                      // index + analog pin
    int chargePin()
    {
      return 2 + ix + ix;               // charge io pin
    }
    int dischargePin()
    {
      return 3 + ix + ix;            // discharge out pin
    }
    double voltConvert =  defaultVoltConvert;
    unsigned int voltArr[anaSamples];     // one per second - each samples/sec sum
    int volts;                // averaged volts
    double resistor = resistorDefault;             // 
    byte state;
    byte prevState;
    double mah;
    unsigned int waitTill;                // wait timer - seconds.
    bool logCell;                 // log flag for cell
    int chargeFinishVolts;     // volts at charge end signal
    int chargeFinishDeltaVolts;     // volt drop after resting after charge
    int dischargePauseVolts;   // volts samples at test pause1 start
    int discharge1DeltaVolts;   // ... and difference after pause time
    int discharge2DeltaVolts;   // end discharge - volt recovery
    int storeChargeFinishVolts;    // target to charge to to hit storeVolts 
    int lastMinuteVolts;
    int millivoltsPerMin;
    unsigned int elapsed;                  // elapsed seconds for current operation
    bool doCharge;
    bool doDischarge;
    bool doStoreCharge;
    void off()
    {
      state = stOff;
      doCharge = false;
      doDischarge = false;
      doStoreCharge = false;
    };
};

Cell cells[numCells];

Cell *selCell;

int logInterval = 60;
unsigned int nextLogAt = 0;
int voltArrIx = 0;               // volt cycle buffer pointer
unsigned long nextSecond = 1000;
unsigned long nextVoltsRead = 0;
unsigned int secCounter = 0;               // for counting minutes




String stateS(byte iS)
{
  switch (iS)
  {
    case stOff:      return "off";
    case stCharge:   return "chg ";
    case stChargePause: return "chgp";
    case stDischarge1: return "disch1";
    case stDischPause1: return "dischp1";
    case stDischarge2: return "disch2";
    case stDischPause2: return "dischp2";
    case stStoreCharge: return "store";
    case stStoreChargePause: return "storep";
  }
  return "disabl";
}

unsigned int ts = 0;           // time seconds counter

void log(String s)
{
  Serial.println(s);
}
void logCell(Cell *cell, String s)
{
  if (cell->ix == selCell->ix)
  {
    log("c*" + String(cell->ix) + c + s);
  }
  else
  {
    log("c:" + String(cell->ix) + c + s);
  }
}

// true = charge, false = no charge
// involves flipping from input to output
void setChargeState(Cell *cell, bool newState)
{
  if (cell->state < 0)
  {
    return;
  }
  pinMode(cell->chargePin(), INPUT_PULLUP);   // set as input - should e already..
  bool nowState = digitalRead(cell->chargePin());
  if (nowState == newState)
  {
    return;
  }
  pinMode(cell->chargePin(), OUTPUT);
  digitalWrite(cell->chargePin(), newState);
  pinMode(cell->chargePin(), INPUT_PULLUP);   // set back as input
  nowState = digitalRead(cell->chargePin());
  if (nowState == newState)
  {
    return;
  }
  logCell(cell, "!! set state pin: " + String(cell->chargePin()));
  cell->state = stDisable;     // disable this cell.
}


void setup()
{
  Serial.begin(9600);           //  setup serial
  analogReference(EXTERNAL);
  log("start setup..");
  log("check cell control..");
  for (int ic = 0; ic < numCells; ic++)
  {
    Cell *cell;
    cell = &cells[ic];
    cell->ix = ic;
    // check control charge state - might disable
    cell->state = stOff;
    setChargeState(cell, false);
    delay(200);
    setChargeState(cell, true);
    delay(200);
    setChargeState(cell, false);
    delay(100);
    // configure discharge pin
    pinMode(cell->dischargePin(), OUTPUT);
    digitalWrite(cell->dischargePin(), false);
    cell->resistor = resistorDefault;
    for (int ix = 0; ix < anaSamples; ix++)
    {
      cell->voltArr[ix] = 0;
    }
    unsigned long tMillis = millis();
    nextVoltsRead = tMillis + 1000/samplesPerSec/2;
    nextSecond = tMillis + 1000;
    secCounter = 0;
    ts = 0;
  }
  selCell = &cells[0];
 
  log("OK");
}


void doCell(Cell *cell)
{
  long voltSum = 0;
  for (int ix = 0; ix < anaSamples; ix++)
  {
    voltSum += cell->voltArr[ix];
    //log(String(ix) + "=" + String(cell->voltArr[ix]));
    
  }

  if (showAnaRaw)
  {
    cell->volts = voltSum / anaDivider;
  }
  else
  {
    cell->volts = voltSum / anaDivider * cell->voltConvert;
  }
  
  bool charged = !digitalRead(cell->chargePin()) || cell->volts > chargeTestVolts;
  
  switch (cell->state)
  {
    case stOff:
      {
        if (cell->doCharge)
        {
          cell->doCharge = false;
          if (cell->volts < 3000)
          {
            logCell(cell,"!!volts");
            cell->off();
          }
          else
          {
            cell->state = stCharge;
            cell->elapsed = 0;
            logCell(cell,"charge v=" + String(cell->volts));
          }
        }
        else if (cell->doDischarge)
        {
          cell->doDischarge = false;
          if (cell->volts < 3000)
          {
            logCell(cell,"!!volts");
            cell->off();
          }
          else
          {
            cell->elapsed = 0;
            cell->mah = 0;
            if (cell->volts > dischargeTestVolts)
            {
              cell->state = stDischarge1;
              logCell(cell,"disch1 v=" + String(cell->volts));
            }
            else
            {
              cell->state = stDischarge2;
              logCell(cell,"disch2 v=" + String(cell->volts));
            }
          }
        }
        else if (cell->doStoreCharge)
        {
          if (cell->volts < 3000)
          {
            cell->doStoreCharge = false;
            logCell(cell,"!!volts");
            cell->off();
          }
          else
          {
            cell->elapsed = 0;
            cell->storeChargeFinishVolts = storeVolts;      // start with +50mv as safe bet
            cell->waitTill = 0;
            cell->state = stStoreCharge;
            logCell(cell,"store v=" + String(cell->volts) + ", t= " + String(cell->storeChargeFinishVolts));
          }
        }
      }
      break;
      
    case stCharge:
      {
        // track time and current volts till charge flagged
        if (!charged)
        {
          cell->chargeFinishVolts = cell->volts;    // must capture while still charging
          cell->elapsed++;
        }
        else
        {
          cell->state = stChargePause;
          cell->waitTill = ts + chargeRestTime;  // wait before discharge for volt check
          logCell(cell,"charge pause v=" + String(cell->volts));
        }
      }
      break;
    
    case stChargePause:
      {
        // wait for rest time - test volt drop - then discharge 
        if (ts >= cell->waitTill)
        {
          cell->state = stOff;
          cell->chargeFinishDeltaVolts = cell->chargeFinishVolts - cell->volts;
          logCell(cell,"charge end v=" + String(cell->volts));
        }
      }
      break;
      
    case stDischarge1:
      {
        cell->mah += cell->volts / cell->resistor / 3600;
        cell->elapsed++;
        
        if (cell->volts <= dischargeTestVolts)
        {
          cell->dischargePauseVolts = cell->volts;
          cell->state = stDischPause1;
          cell->waitTill = ts + dischargeTestTime;  // wait then volt check
          logCell(cell,"disch1 pause v=" + String(cell->volts));
        }
      }
      break;
    
    case stDischPause1:
      {
        if (ts >= cell->waitTill)
        {
          cell->state = stDischarge2;
          cell->discharge1DeltaVolts = cell->volts - cell->dischargePauseVolts;
          logCell(cell,"disch2 v=" + String(cell->volts));
        }
      }
      break;
    
    case stDischarge2:
      {
        cell->mah += cell->volts / cell->resistor / 3600;
        cell->elapsed++;
        if (cell->volts <= dischargeEndVolts)
        {
          cell->dischargePauseVolts = cell->volts;
          cell->state = stDischPause2;
          cell->waitTill = ts + dischargeTestTime;  // wait after discharge for volt check
          logCell(cell,"disch2 pause v=" + String(cell->volts));
        }
      }
      break;
    
    case stDischPause2:
      {
        if (ts >= cell->waitTill)
        {
          cell->state = stOff;
          cell->elapsed = 0;
          cell->discharge2DeltaVolts = cell->volts - cell->dischargePauseVolts;
          logCell(cell,"disch2 end v=" + String(cell->volts));
        }
      }
      break;
    
    case stStoreCharge:
      {
        cell->elapsed++;
        if (cell->volts >= cell->storeChargeFinishVolts && ts >= cell->waitTill || cell->volts >= 4000) 
        {
          cell->storeChargeFinishVolts = cell->volts;
          cell->state = stStoreChargePause;
          cell->waitTill = ts + chargeTestTime;
          logCell(cell,"store pause v=" + String(cell->volts));
        }
      }
      break;
    
    case stStoreChargePause:
      {
        if (ts >= cell->waitTill)
        {
          if(cell->volts > storeVolts)
          {
            cell->waitTill = ts + chargeTestTime;
          }
          else
          {
            cell->state = stStoreCharge;
            cell->waitTill = ts + 30;   // give time for volts to stabilize
            cell->storeChargeFinishVolts = min(4000, storeVolts + (cell->storeChargeFinishVolts - cell->volts)+10);
            logCell(cell,"storecont v=" + String(cell->volts) + ", t=" + String(cell->storeChargeFinishVolts));
          }
        }
      }
      break;
  }

  bool enable = false;
  bool discharge = false;

  switch (cell->state)
  {
    case stCharge:
    case stStoreCharge:
      enable = true;
      break;
    case stDischarge1:
    case stDischarge2:
      discharge = true;
      break;
  }
  setChargeState(cell, enable);
  digitalWrite(cell->dischargePin(), discharge);
}
bool forceLog = false;


void processSerial()
{
  Cell *cell = selCell;
  char k = Serial.read();
  if (k >= char('0') && k <= char('9'))
  {
    int newIx = k - char('0');
    if (newIx >= 0 && newIx < numCells)
    {
      selCell = &cells[newIx];
      cell = selCell;
      logCell(cell, "sel");
    }
    else
    {
      log("invalid cell");
    }
  }
  else switch (k)
    {
      case '?':
        //log("each character is a command..");
        log("<n> sel 0-" + String(numCells));
        log("l log");
        log("f full cycle");
        log("c chg");
        log("d disch");
        log("s store");
        log("x stop");
        log("t test");
        log("r raw");
        log("z disabl");

      
      case 't':
         chargeTestVolts = 3900;       // test mode - lower test charge
         chargeRestTime = 30;          // after charging - seconds before discharge
         dischargeTestVolts = 3700;  // pause on discharge to test how volts recover.
         dischargeTestTime = 30;       // .. for how long - same at end discharge
         dischargeEndVolts = 3500;         // discharge stop volts
         storeVolts = 3700;          // storage charge default
         chargeTestTime = 30;        // during store charge - rest time before volt check in recharge-to loop
         log("!!test");
         break;

       case 'r':
        showAnaRaw = !showAnaRaw;
        break;

       case 'z':
         if (selCell->state == stDisable)
         {
          selCell->state = stOff;
         }
         else if (selCell->state == stOff)
         {
            selCell->state = stDisable;
         }
         break;
     
      case 'l':
        forceLog = true;
        break;

      case 'f':
        if (cell->state == stOff)
        {
          cell->doCharge = true;
          cell->doDischarge = true;
          cell->doStoreCharge = true;
        }
        else
        {
          logCell(cell, stateNotOff); 
        }
      case 'c':
        if (cell->state == stOff)
        {
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
          cell->doDischarge = true;
        }
        else
        {
          logCell(cell,stateNotOff); 
        }
        break;
      case 's':
        if (cell->state == stOff)
        {
          cell->doStoreCharge = true;
        }
        else
        {
          logCell(cell,stateNotOff); 
        }
        break;
      case 'x':
        cell->off();
        logCell(cell, "off");
        break;
    }
}

void loop()
{
   // process serial.
  while (Serial.available() > 0)
  {
    processSerial(); 
  }
  
  unsigned long tmillis = millis();
 
  // read analogues at samplesPerSec
  while (tmillis >= nextVoltsRead)
  {
    for (int ic = 0; ic < numCells; ic++)
    {
      Cell *cell = &cells[ic];
      cell->voltArr[voltArrIx] += analogRead(cell->ix);
      if (cell->state == stDischarge1 || cell->state == stDischarge1)
      {
         cell->voltArr[voltArrIx] += dischargeRawAdjust;
      }
    }
    nextVoltsRead += 1000/samplesPerSec;
  }

  if (tmillis >= nextSecond)
  {
    ts = ts + 1;
    nextVoltsRead = nextSecond + 1000/samplesPerSec/2;
    nextSecond+= 1000;
    secCounter++;
   
    if (secCounter >= 60)
    {
      secCounter = 0;
    }
    
    for (int ic = 0; ic < numCells; ic++)
    {
      Cell *cell = &cells[ic];
      if (cell->state >= stOff)
      {
        doCell(cell);
      }
      if (secCounter == 0)
      {
        if (cell->lastMinuteVolts == 0)
        {
          cell->lastMinuteVolts = cell->volts;
        }
        cell->millivoltsPerMin = cell->volts - cell->lastMinuteVolts;
        cell->lastMinuteVolts = cell->volts;
      }
    }
    
    voltArrIx++;
    if (voltArrIx >= anaSamples)
    {
      voltArrIx = 0;
    }
    for (int ic = 0; ic < numCells; ic++)
    {
      cells[ic].voltArr[voltArrIx] = 0;
    }
  
    bool loggit = false;
    if (ts >= nextLogAt)
    {
      loggit = true;
      nextLogAt += logInterval;
    }
    //forceLog = true;
    for (int ic = 0; ic < numCells; ic++)
    {
      Cell *cell = &cells[ic];
      if ((cell->state > stOff && loggit) || (cell->state >= stOff && forceLog) || (cell->state != cell->prevState) || cell->logCell)
      {
        logCell(cell, String(cell->elapsed/60.0, 1) + c + stateS(cell->state) 
                + c + String(cell->volts) 
                + c + String(cell->millivoltsPerMin)
                + c + String(cell->mah, 0) 
                + c + String(cell->chargeFinishVolts) 
                + c + String(cell->chargeFinishDeltaVolts) 
                + c + String(cell->discharge1DeltaVolts) 
                + c + String(cell->discharge2DeltaVolts));
      }
      cell->prevState = cell->state;
      cell->logCell = false;
      
    }
    forceLog = false;
  }
}
