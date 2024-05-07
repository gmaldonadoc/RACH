/*

 */

using namespace std;

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <cmath>

#include "drx-manager.h"
#include "../device/UserEquipment.h"
#include "../protocolStack/mac/ue-mac-entity.h"
#include "../protocolStack/protocol-stack.h"
#include "../core/eventScheduler/simulator.h"
#include "../componentManagers/FrameManager.h"
#include "../energyConsumption/ue-consumption-model.h"

void
DRXManager::DefaultValuesLong() {
  enable = OFF;
  cycleType = LONG;
}

void
DRXManager::DefaultValuesShort() {
  enable = ON;
  cycleType = SHORT;

  expectedOff = 1;
  remainingOnDuration = 0; //Start with the ON state active
  expectedOn = 0;
  
  remainingShortCycles = drxShortCycleTimer;
}

DRXManager::DRXManager() {
  init();
}

void DRXManager::init() {
  offset = 0;
  currentTime = 0;
  currentStatus = ON_DURATION; //DRX_LIGHT

  remainingOnDuration = 0; //time left to end the onDurationTimer
  remainingInactivity = 0; //time left to end the InactivityTimer
  remainingShortCycles = 0;

  expectedOff = 0;
  expectedOn = 0;

  tActiveToSleep = 0;
  tSleepToActive = 0;

  drxOnDurationTimer = 2; // in ms
  drxInactivityTimer = 3; // in ms
  longDrxCycle = 40; // in ms
  shortDrxCycle = 20; // in ms
  drxStartOffset = 0; // in ms
  drxShortCycleTimer = 3;

  drxWakeupLightTimer = 6; // in ms
  drxPowerdownLightTimer = 5; // in ms

  drxWakeupDeepTimer = 12; // in ms
  drxPowerdownDeepTimer = 9; // in ms

  drxThresholdDeepSleep = 40; // in ms

  drxSyncTimer = 0; // in ms
  drxMeasTimer = 5; // in ms

  //Initial parameter value for a SHORT cycle in the active mode
  //enable = ON; //Short DRX cycle activated
  //cycleType = SHORT; //The initial cycle is Short
  //remainingOnDuration = drxOnDurationTimer; //Start with the ON state active
  //remainingShortCycles = drxShortCycleTimer; //Start with the Short Cycle type Active
  //expectedOn = drxOnDurationTimer;

  m_device = NULL;
  //DefaultValuesLong();
  DefaultValuesShort();

  m_sleepType = 0;
}

DRXManager::DRXManager(unsigned int t[6], EnableShortCycle en) {
  init();

  drxOnDurationTimer = t[0];
  drxInactivityTimer = t[1];
  drxStartOffset = t[2];
  longDrxCycle = t[3];
  shortDrxCycle = t[4];
  drxShortCycleTimer = t[5];

  enable = en;
  if (en == ON) {
    cycleType = SHORT;
    remainingShortCycles = drxShortCycleTimer; //drxShortCycleTimer;
    //TODO ver se precisa colocar o expectedOn en algum valor

  } else {
    cycleType = LONG;
    remainingShortCycles = 0;
    //TODO ver se precisa colocar o expectedOn en algum valor
  }
}

void DRXManager::update() {
  double td = Simulator::Init()->Now();

#ifdef DRX_DEBUG
  cerr << td << " [DRX] expectedOff " << expectedOff << " expectedOn " << expectedOn << " remainingOnDuration " << remainingOnDuration << " remainingInactivity " << remainingInactivity << endl;
#endif

  int t = rint(td * 1000);
  int tempoDecorrido = t - currentTime;

  if (tempoDecorrido < 0) {
    cout << "Error: Tempo invalido" << endl;
    exit(EXIT_FAILURE);
  } else if (tempoDecorrido == 0 && t != 0) {
    cout << "Error: Multiple call of update DRX" << endl;
    //exit(EXIT_FAILURE);
    return;
  } else if (tempoDecorrido == 0 && t == 0) {
    tempoDecorrido = 1; //ms
  }

  currentTime = t;

  bool inRandomAccess = ((UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity())->executingRA();

#ifdef DRX_DEBUG
  cerr << Simulator::Init()->Now() << " [DRX] current time set to " << currentTime << " ms" << endl;
#endif
  //***Update on duration
  if (remainingOnDuration > 0) {
    remainingOnDuration -= tempoDecorrido;
    if (remainingOnDuration < 0) {
      remainingOnDuration = 0;
    }
#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] decreasing remainingOnDuration by " << tempoDecorrido << " ms" << " newValue " << remainingOnDuration << " ms" << endl;
#endif
    // if OnDurationTimer expires in this TTI && shortCycle in use: decrease the number of remaining short cycles
    if (cycleType == SHORT && remainingOnDuration <= 0) {
      remainingShortCycles--;
      // if ShortCycleTimer expires in this TTI, use the LongCycle

#ifdef DRX_DEBUG
      cerr << Simulator::Init()->Now() << " [DRX] decreasing remainingShortCycles by 1 newValue " << remainingShortCycles << endl;
#endif

      if (remainingShortCycles <= 0)
        cycleType = LONG; //TODO Verify if this is correct. It can be wrong because the off time of last cycle still needs to be short and not long
      /*if drxShortCycleTimer expires in this subframe:
        -    use the Long DRX cycle
       */
    }
  }

  //***	update I-Timer
#ifdef DRX_DEBUG
  cerr << Simulator::Init()->Now() << " [DRX] remainingInactivity " << remainingInactivity << endl;
#endif

  if (remainingInactivity > 0) {
    remainingInactivity -= tempoDecorrido;

    if (remainingInactivity <= 0) {
      remainingInactivity = 0;
    }
#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] decreasing remainingInactivity by " << tempoDecorrido << " ms" << " newValue " << remainingInactivity << endl;
#endif
    // if I-timer expires in this TTI, use ShortCycle and start shortCycleTimer
    if (enable == ON && remainingInactivity <= 0) {
#ifdef DRX_DEBUG
      cerr << Simulator::Init()->Now() << " [DRX] reset remainingShortCycles to " << drxShortCycleTimer << endl;
#endif
      remainingShortCycles = drxShortCycleTimer;
      cycleType = SHORT;
    } else {
      cycleType = LONG;
    }
  }

  //***	HARQ TODO to Implement

  //***	Cycle Control and expected ON/OFF duration evaluation

  //int offset = 0; // offset according to the current type of drx cycle
  int timeMod = 0;
  // Compute the number of TTI, according to SHORT/LONG DRX cycles, until the next ON phase
  int auxTime = 0;
  //int RNTI = (FrameManager::Init()->GetNbFrames() * 10) + FrameManager::Init()->GetNbSubframes();
  int RNTI = currentTime;
  /*
  -   If the Short DRX Cycle is used and [(SFN * 10) + subframe number] modulo (shortDRX-Cycle) =
  (drxStartOffset) modulo (shortDRX-Cycle); or
  -    if the Long DRX Cycle is used and [(SFN * 10) + subframe number] modulo (longDRX-Cycle) = drxStartOffset
- start onDurationTimer
   * */
  if (cycleType == SHORT) {
    // adjust offset position in case of startOffset != 0
    //timeMod = (currentTime % shortDrxCycle);
    timeMod = (RNTI % shortDrxCycle);
    if (timeMod < (drxStartOffset % shortDrxCycle)) {
      timeMod += shortDrxCycle;
    }
    // offset of the current TTI, inside the current type of drx cycle
    offset = ((drxStartOffset % shortDrxCycle) - timeMod);
    offset = abs(offset);
    auxTime = (offset - shortDrxCycle); // remaining TTIs until the next OnDuration restart
    expectedOff = abs(auxTime);
#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] shortCycle offset " << offset << ", timeMod " << timeMod << " and expectedOff " << expectedOff << " ms" << endl;
#endif
  } else { //cycleType == LONG
    // adjust offset position in case of startOffset != 0
    //timeMod = (currentTime % longDrxCycle);
    timeMod = (RNTI % longDrxCycle);
    if (timeMod < drxStartOffset) {
      timeMod += longDrxCycle;
    }
    // offset of the current TTI, inside the current type of drx cycle
    offset = (drxStartOffset - timeMod);
    offset = abs(offset);
    auxTime = (offset - longDrxCycle); // remaining TTIs until the next OnDuration restart
    expectedOff = abs(auxTime);

#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] LongCycle offset " << offset << ", timeMod " << timeMod << " and expectedOff " << expectedOff << " ms" << endl;
#endif

  }
  SetExpectedOffTime(expectedOff);

  // if the offset inside the current type of drx cycle is 0, start the OnDuration Timer
  if (offset == 0) {
    remainingOnDuration = drxOnDurationTimer;
#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] drxOnDurationTimer reseted" << endl;
#endif
  }

  unsigned int newExpectedOn;

  if (remainingInactivity > remainingOnDuration)
    newExpectedOn = remainingInactivity;
  else
    newExpectedOn = remainingOnDuration;

  if (GetDevice() == NULL) {//Verify if the device was attached to the DRXManager.
    cerr << Simulator::Init()->Now() << " [Error] Device not Specified for DRX Manager" << endl;
    exit(EXIT_FAILURE);
  }

  if (tSleepToActive == 0) {

    //if (getStatus() == LIGHT || getStatus() == WAKEUP_LIGHT || getStatus() == DEEP) {
    if (getStatus() == LIGHT || getStatus() == WAKEUP_LIGHT) {
      if (expectedOff <= drxWakeupLightTimer + drxSyncTimer) {
        setStatus(WAKEUP_LIGHT);
        if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
          ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(WAKEUP_LIGHT, PDCCH_NO_DATA);
        }

        tSleepToActive++;
#ifdef DRX_DEBUG
        cerr << Simulator::Init()->Now() << " [DRX] LIGHT Sleep to Active" << endl;
#endif
      }
    } else if (getStatus() == DEEP || getStatus() == WAKEUP_DEEP) {
      if (expectedOff <= drxWakeupDeepTimer + drxSyncTimer) {
        setStatus(WAKEUP_DEEP);
        if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
          ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(WAKEUP_DEEP, PDCCH_NO_DATA);
        }

        tSleepToActive++;
#ifdef DRX_DEBUG
        cerr << Simulator::Init()->Now() << " [DRX] DEEP Sleep to Active" << endl;
#endif
      }
    }

    if (inRandomAccess) {
#ifdef DRX_DEBUG
      cerr << Simulator::Init()->Now() << " [DRX] Executing RandomAccess " << endl;
#endif
      if (newExpectedOn <= 0)
        newExpectedOn = 1;
    }

    expectedOn = newExpectedOn;

    /* ---------------- */
    if (remainingInactivity > 0 || remainingOnDuration > 0 || inRandomAccess) {
      //setStatus(ACTIVE);
      setStatus(ON_DURATION);
      if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
        ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(ACTIVE_NO_DATA, PDCCH_NO_DATA);
      }

      tActiveToSleep = 0;
      tSleepToActive = 0;

      m_sleepType = 0;

    } else if (remainingOnDuration == 0 && remainingInactivity == 0 && expectedOff > 1 && (expectedOff > drxWakeupLightTimer || expectedOff > drxWakeupDeepTimer)) {

      bool a1 = false;

      if (m_sleepType == 0) {
        if (expectedOff < drxThresholdDeepSleep) {
          m_sleepType = 1;
        } else {
          m_sleepType = 2;
        }
      }

      if (currentStatus == ON_DURATION || currentStatus == MEAS) {
        if (tActiveToSleep < drxMeasTimer) {
          setStatus(MEAS);
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(MEAS, PDCCH_NO_DATA);
          }

          a1 = true;
          tActiveToSleep++;
        } else {
          tActiveToSleep = 0;

          if (m_sleepType == 1) {
            if (tActiveToSleep < drxPowerdownLightTimer) {
              currentStatus = POWERDOWN_LIGHT;
            }
          } else {
            if (tActiveToSleep < drxPowerdownDeepTimer) {
              currentStatus = POWERDOWN_DEEP;
            }
          }
        }
      }

      if (m_sleepType == 1 && !a1 && (currentStatus == ON_DURATION || currentStatus == POWERDOWN_LIGHT)) {

        if (tActiveToSleep < drxPowerdownLightTimer) {
          setStatus(POWERDOWN_LIGHT);
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(POWERDOWN_LIGHT, PDCCH_NO_DATA);
          }

          tActiveToSleep++;
        } else {
          if (m_sleepType == 1 && (currentStatus != DEEP && currentStatus != WAKEUP_DEEP && currentStatus != POWERDOWN_DEEP)) {
            setStatus(LIGHT);
            if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
              ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(LIGHT, PDCCH_NO_DATA);
            }
          }
        }
      } else if (m_sleepType == 2 && !a1 && (currentStatus == ON_DURATION || currentStatus == POWERDOWN_DEEP)) {
        if (tActiveToSleep < drxPowerdownDeepTimer) {
          setStatus(POWERDOWN_DEEP);
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(POWERDOWN_DEEP, PDCCH_NO_DATA);
          }

          tActiveToSleep++;
        } else {
          //if (currentStatus != WAKEUP_DEEP) {
          setStatus(DEEP);
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DEEP, PDCCH_NO_DATA);
          }
          //}
        }
      } else if (m_sleepType == 1 && !a1 && (currentStatus != DEEP && currentStatus != WAKEUP_DEEP && currentStatus != POWERDOWN_DEEP)) {
        setStatus(LIGHT);
        if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
          ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(LIGHT, PDCCH_NO_DATA);
        }
      } else if (!a1 && currentStatus != WAKEUP_DEEP && currentStatus != POWERDOWN_DEEP) {
        setStatus(DEEP);
        if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
          ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DEEP, PDCCH_NO_DATA);
        }
      }

#ifdef DRX_DEBUG
      cerr << Simulator::Init()->Now() << " [DRX] Entered Here 1!" << endl;
#endif

    } else if (remainingOnDuration == 0 && remainingInactivity == 0 && expectedOff == 1 && expectedOff > drxWakeupLightTimer) {

      if (currentStatus == LIGHT || currentStatus == POWERDOWN_LIGHT) {
        if (tActiveToSleep < drxPowerdownLightTimer) {
          setStatus(POWERDOWN_LIGHT);
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(POWERDOWN_LIGHT, PDCCH_NO_DATA);
          }

          tActiveToSleep++;
        }
      }

      if (currentStatus == DEEP || currentStatus == POWERDOWN_DEEP) {
        if (tActiveToSleep < drxPowerdownDeepTimer) {
          setStatus(POWERDOWN_DEEP);
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(POWERDOWN_DEEP, PDCCH_NO_DATA);
          }

          tActiveToSleep++;
        }
      }

#ifdef DRX_DEBUG
      cerr << Simulator::Init()->Now() << " [DRX] Entered Here 2!" << endl;
#endif

      if ((expectedOff < drxThresholdDeepSleep) && (currentStatus != DEEP && currentStatus != WAKEUP_DEEP && currentStatus != POWERDOWN_DEEP)) {
        setStatus(LIGHT);
        if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
          ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(LIGHT, PDCCH_NO_DATA);
        }
      } else if (currentStatus != WAKEUP_DEEP && currentStatus != POWERDOWN_DEEP) {
        setStatus(DEEP);
        if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
          ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DEEP, PDCCH_NO_DATA);
        }
      }
    } else if (expectedOff > drxWakeupLightTimer) {
      setStatus(DEEP);
      if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
        ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DEEP, PDCCH_NO_DATA);
      }
    }
  } else if (tSleepToActive < drxWakeupLightTimer && (currentStatus == LIGHT || currentStatus == WAKEUP_LIGHT)) {
    setStatus(WAKEUP_LIGHT);
    if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(WAKEUP_LIGHT, PDCCH_NO_DATA);
    }
#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] LIGHT Sleep to Active" << endl;
#endif

    tSleepToActive++;

  } else if (tSleepToActive < drxWakeupDeepTimer && (currentStatus == DEEP || currentStatus == WAKEUP_DEEP)) {
    setStatus(WAKEUP_DEEP);
    if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(WAKEUP_DEEP, PDCCH_NO_DATA);
    }
#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] DEEP Sleep to Active" << endl;
#endif

    tSleepToActive++;
  } else if (expectedOff <= drxSyncTimer) {
    setStatus(SYNC);
    if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(SYNC, PDCCH_NO_DATA);
    }

    tSleepToActive++;
  } else {
    //setStatus(ACTIVE);
    setStatus(ON_DURATION);
    if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(ACTIVE_NO_DATA, PDCCH_NO_DATA);
    }

    m_sleepType = 0;

    tActiveToSleep = 0;
    tSleepToActive = 0;
  }
  /* ---------------- */

#ifdef DRX_DEBUG
  cerr << Simulator::Init()->Now() << " [DRX] expectedOn " << expectedOn << " ms" << endl;
#endif

  if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
    //((UserEquipment*) GetDevice())->GetConsumptionModel()->updateEnergyConsumption(tempoDecorrido);
  }

  Simulator::Init()->Schedule(0.001, &DRXManager::update, this);

  //PrintTrace(inRandomAccess);
}

void DRXManager::drxSleep() {
  remainingInactivity = 0;
  remainingOnDuration = 0;

  if (enable == ON) {
    cycleType = SHORT;
    remainingShortCycles = drxShortCycleTimer;
  }
}

//------------------- PDCCH DL SCHEDULING RECEPTION ---------------------------

bool DRXManager::drxUpdateDL(Action a) { //<- vai ver se pode receber grant no tti atual(nao sei se depende do tipo de ciclo)
  currentStatus = ACTIVE_NO_DATA;

  if (a == PDCCH_RX || a == PDCCH_MSG2_RX) {//Sucessfully decoding of a PDCCH message

    remainingInactivity = drxInactivityTimer + 1; //Reset DRX inactivity Timer

#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] Successfully decoding of a PDCCH message, remainingInactivity reseted" << endl;
#endif

    PrintToTrace(a);
  }
  /*
      // If this is not a retransmission, start/restart inactivity timer
      //if( !isRetx )
      // use +1 because we need to stay awake for "InactivityTimer" TTIs, starting from the next TTI
      if(a==PDCCH_RX){
        remainingInactivity = drxInactivityTimer + 1;
      }

    if(currentStatus != DRX_LIGHT){//se ele ja esta ON, continua ON
      remainingInactivity = drxInactivityTimer;
      return true;
    }
    else { //If DRX Light && expected off <= 1, It means that a DL transmission can be scheduled.
      if(expectedOff <= 1)
        return true;
      else //UE cannot receive any TX because it is sleeping.
        return false;
    }
   */
  return true;
}

bool DRXManager::drxUpdateUL(Action a) { //TODO : necessario?
  //currentStatus = ACTIVE_TX;
  // If this is not a retransmission or periodic grant, start/restart inactivity timer and update the expected ON time

  if (a == PUSCH_TX) {
    // use +1 because we need to stay awake for "InactivityTimer" TTIs, starting from the next TTI
#ifdef DRX_DEBUG
    cerr << Simulator::Init()->Now() << " [DRX] PUSCH Transmission, remainingInactivity restarted" << endl;
#endif
    //remainingInactivity = drxInactivityTimer + 1;

    if (expectedOn < remainingInactivity) {
      //expectedOn = remainingInactivity;
    }

    PrintToTrace(a);
  }

  return true;
}

unsigned int DRXManager::getExpectedStatusDuration() {
  if (currentStatus == DEEP || currentStatus == LIGHT)
    return expectedOff;

  return expectedOn;
  //Returns expected time to change the current status.
}

void DRXManager::setDrxOnDurationTimer(unsigned int t) {
  drxOnDurationTimer = t;
}

void DRXManager::setDrxInactivityTimer(unsigned int t) {
  drxInactivityTimer = t;
}

void DRXManager::setDrxStartOffset(unsigned int t) {
  drxStartOffset = t;
}

void DRXManager::setLongDRXCycle(unsigned int t) {
  longDrxCycle = t;
}

void DRXManager::setShortDRXCycle(unsigned int t) {
  shortDrxCycle = t;
}

void DRXManager::setDrxShortCycleTimer(unsigned int t) {
  drxShortCycleTimer = t;
}

unsigned int DRXManager::getDrxOnDurationTimer() {
  return drxOnDurationTimer;
}

unsigned int DRXManager::getDrxInactivityTimer() {
  return drxInactivityTimer;
}

unsigned int DRXManager::getDrxStartOffset() {
  return drxStartOffset;
}

unsigned int DRXManager::getlongDRXCycle() {
  return longDrxCycle;
}

unsigned int DRXManager::getShortDRXCycle() {
  return shortDrxCycle;
}

unsigned int DRXManager::getDrxShortCycleTimer() {
  return drxShortCycleTimer;
}

void DRXManager::setExpectedOff(unsigned int t) {
  expectedOff = t;
}

void DRXManager::setExpectedOn(unsigned int t) {
  expectedOn = t;
}

unsigned int DRXManager::getExpectedOff() {
  return expectedOff;
}

unsigned int DRXManager::getExpectedOn() {
  return expectedOn;
}

void DRXManager::setRemainingOnDuration(unsigned int t) {
  remainingOnDuration = t;
}

void DRXManager::setRemainingInactivity(unsigned int t) {
  remainingInactivity = t;
}

void DRXManager::setRemainingShortCycles(unsigned int t) {
  remainingShortCycles = t;
}

unsigned int DRXManager::getRemainingOnDuration() {
  return remainingOnDuration;
}

unsigned int DRXManager::getRemainingInactivity() {
  return remainingInactivity;
}

unsigned int DRXManager::getRemainingShortCycles() {
  return remainingShortCycles;
}

DRXManager::State
DRXManager::getStatus() {
  return currentStatus;
}

void
DRXManager::setStatus(State s) {
  if (s == ACTIVE) {
    if (currentStatus == LIGHT) {
      currentStatus = WAKEUP_LIGHT;
      if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
        ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(WAKEUP_LIGHT, PDCCH_NO_DATA);
      }
      tSleepToActive = 1;
    } else if (currentStatus == DEEP) {
      currentStatus = WAKEUP_DEEP;
      if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
        ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(WAKEUP_DEEP, PDCCH_NO_DATA);
      }
      tSleepToActive = 1;
    } else if (currentStatus == ON_DURATION) {
      if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
        ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(ACTIVE_NO_DATA, PDCCH_NO_DATA);
      }
    }
  } else {
    currentStatus = s;
  }

#ifdef DRX_DEBUG
  cerr << Simulator::Init()->Now() << " [DRX] New State";
  if (currentStatus == DEEP)
    cerr << " DEEP" << endl;
  else if (currentStatus == LIGHT)
    cerr << " LIGHT" << endl;
  else if (currentStatus == ACTIVE)
    cerr << " ACTIVE" << endl;
  else if (currentStatus == ON_DURATION)
    cerr << " ON_DURATION" << endl;
  else if (currentStatus == WAKEUP_LIGHT)
    cerr << " WAKEUP LIGHT" << endl;
  else if (currentStatus == POWERDOWN_LIGHT)
    cerr << " POWERDOWN LIGHT" << endl;
  else if (currentStatus == ON)
    cerr << " ON" << endl;
  else if (currentStatus == WAKEUP_DEEP)
    cerr << " WAKEUP DEEP" << endl;
  else if (currentStatus == POWERDOWN_DEEP)
    cerr << " POWERDOWN DEEP" << endl;
  else if (currentStatus == SYNC)
    cerr << " SYNC" << endl;
  else if (currentStatus == MEAS)
    cerr << " MEAS" << endl;
#endif
}

void
DRXManager::SetDevice(NetworkNode * d) {
  m_device = d;
}

NetworkNode *
DRXManager::GetDevice() {
  return m_device;
}

void
DRXManager::SetExpectedOffTime(int expectedOff) {
  m_nextOnTime = Simulator::Init()->Now() + (expectedOff / 1000.0);
}

double
DRXManager::GetExpectedOffTime() {
  double offTime = m_nextOnTime - Simulator::Init()->Now();

  return offTime;
}

unsigned int
DRXManager::GetSleepToActiveLightTimer() {
  return drxWakeupLightTimer;
};

unsigned int
DRXManager::GetSleepToActiveCounter() {
  return tActiveToSleep;
}

void
DRXManager::PrintTrace() {
#ifdef DRX_DEBUG
  cout << Simulator::Init()->Now() << " [DRX] Calling printTrace() " << endl;
#endif
  string trace;
  ostringstream os;
  os.str("");
  os << endl;
  //Printing Time
  os << Simulator::Init()->Now()*1000;

  os << ";";
  //Printig Type of DRX
  if (getStatus() == LIGHT || getStatus() == DEEP)
    os << "OFF;";
  else
    os << "ON;";
  //Printing  Inactivity Timer
  if (getRemainingInactivity() > 0)
    os << "ON;";
  else
    os << "OFF;";

  if (cycleType == LONG)
    os << "Long Cycle;";

  else
    os << "Short Cycle;";

  os << offset << ";";
  //os << endl;
  trace.append(os.str());
  os.clear();
}

void
DRXManager::PrintTrace(bool inRA) {
#ifdef DRX_DEBUG
  cout << Simulator::Init()->Now() << " [DRX] Calling printTrace() " << endl;
#endif
  string trace;
  ostringstream os;
  os.str("");
  os << endl;
  //Printing Time
  os << Simulator::Init()->Now()*1000;

  os << ";";
  //Printig Type of DRX
  if (getStatus() == LIGHT || getStatus() == DEEP)
    os << "OFF;";
  else
    os << "ON;";
  //Printing  Inactivity Timer
  if (getRemainingInactivity() > 0)
    os << "ON;";
  else
    os << "OFF;";
  //Status RA
  if (inRA)
    os << "ON;";
  else
    os << "OFF;";

  if (cycleType == LONG)
    os << "Long Cycle;";

  else
    os << "Short Cycle;";

  os << offset << ";";
  //os << endl;
  trace.append(os.str());
  os.clear();
}

void
DRXManager::PrintToTrace(Action a) {
#ifdef DRX_DEBUG
  cout << Simulator::Init()->Now() << " [DRX] Calling printTrace() " << endl;
#endif
  string trace;
  ostringstream os;
  os.str("");
  //Printig Type of DRX
  if (a == PDCCH_RX) {//Sucessfully decoding of a PDCCH message
    os << "PDCCH_RX;";
  } else if (a == PDCCH_MSG2_RX) {
    os << "PDCCH_MSG2_RX;";
  } else if (a == PDSCH_RX) {
    os << "PDSCH_RX;";
  } else if (a == PUSCH_TX) {
    os << "PUSCH_TX;";
  }

  trace.append(os.str());

  os.clear();
}

