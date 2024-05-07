/* 
 * File:   ue-consumption-model.h
 * Author: tiago
 *
 * Created on August 25, 2016, 3:15 PM
 */

#ifndef DRX_MANAGER_H
#define DRX_MANAGER_H

#include "../device/NetworkNode.h"

class NetworkNode;

class DRXManager {
public:

  enum State {
    DEEP = 0,
    LIGHT = 1,
    ACTIVE_NO_DATA = 2,
    ACTIVE_RX = 3,
    ACTIVE_TX = 4,
    ACTIVE_RXTX = 5,
    ACTIVE = 6,
    POWERDOWN_LIGHT = 7,
    WAKEUP_LIGHT = 8,
    POWERDOWN_DEEP = 9,
    WAKEUP_DEEP = 10,
    ON_DURATION = 11,
    SYNC = 12,
    MEAS = 13
  };

  enum Action {
    PDCCH_NO_DATA = 0, //Listen to the PDCCH (), no decoding
    PDCCH_RX = 1, //Listen to the PDCCH (). Successful decoding of a Control Message.
    PDCCH_MSG2_RX = 2, //Reception of a RAR plus RX of data on the PDSCH to decode the information about uplink grant to send msg3 on PUSCH
    PDSCH_RX = 3, //Reception of DL data on the PDSCH
    PUSCH_TX = 4, //Transmission of UL Data on the PUSCH
    PUCCH_TX = 5, //Transmission of Control Messages on PUCCH
    PRACH_TX = 6, //Transmission of Control Messages on PRACH
    PRACH_RX = 7, //Reception of Control Messages on PRACH
  };

  enum Cycle {
    LONG = 0,
    SHORT = 1,
  };

  enum EnableShortCycle {
    OFF = 0,
    ON = 1
  };

  static const int N_DRX_STATES = 14;

  static const int N_DRX_TIMERS = 8;

  static const int N_PWR_PARAM = 8;

  DRXManager();
  DRXManager(unsigned int t[6], EnableShortCycle en);

  void update();

  void drxSleep();

  bool drxUpdateDL(Action a);
  bool drxUpdateUL(Action a);

  unsigned int getExpectedStatusDuration();

  void setDrxOnDurationTimer(unsigned int t);
  void setDrxInactivityTimer(unsigned int t);
  void setDrxStartOffset(unsigned int t);
  void setLongDRXCycle(unsigned int t);
  void setShortDRXCycle(unsigned int t);
  void setDrxShortCycleTimer(unsigned int t);

  unsigned int getDrxOnDurationTimer();
  unsigned int getDrxInactivityTimer();
  unsigned int getDrxStartOffset();
  unsigned int getlongDRXCycle();
  unsigned int getShortDRXCycle();
  unsigned int getDrxShortCycleTimer();

  void setExpectedOff(unsigned int t);
  void setExpectedOn(unsigned int t);

  unsigned int getExpectedOff();
  unsigned int getExpectedOn();

  void setRemainingOnDuration(unsigned int t);
  void setRemainingInactivity(unsigned int t);
  void setRemainingShortCycles(unsigned int t);

  unsigned int getRemainingOnDuration();
  unsigned int getRemainingInactivity();
  unsigned int getRemainingShortCycles();

  State getStatus();
  void setStatus(State s);

  void SetDevice(NetworkNode* d);
  NetworkNode* GetDevice();

  void SetExpectedOffTime(int expectedOff);
  double GetExpectedOffTime();

  void DefaultValuesLong(void);
  void DefaultValuesShort(void);

  void PrintTrace(void);
  void PrintTrace(bool ra);
  void PrintToTrace(Action a);

  unsigned int GetSleepToActiveLightTimer(void);
  unsigned int GetSleepToActiveCounter(void);

private:
  void init(); //initialization of variables!

  unsigned int drxOnDurationTimer; /*Specifies the number of consecutive PDCCH-subframe(s) at the beginning of a DRX Cycle. */
  unsigned int drxInactivityTimer; /*Specifies the number of consecutive PDCCH-subframe(s) after the subframe in which a PDCCH indicates an initial UL, DL or SL user data transmission for this MAC entity.*/
  unsigned int drxStartOffset; //Specifies the subframe where the DRX Cycle starts
  unsigned int longDrxCycle; //Long DRX cycle length
  unsigned int shortDrxCycle; //Short DRX cycle length
  // unsigned int drxRetransmissionTimer; //not implemented
  unsigned int drxShortCycleTimer; // Specifies the number of consecutive cycles the MAC entity shall follow the Short DRX cycle

  unsigned int drxActiveToSleepLightTimer, drxSleepToActiveLightTimer;

  unsigned int drxPowerdownLightTimer, drxWakeupLightTimer;
  unsigned int drxPowerdownDeepTimer, drxWakeupDeepTimer;

  unsigned int drxThresholdDeepSleep;

  unsigned int drxSyncTimer, drxMeasTimer;

  unsigned int expectedOff;
  unsigned int expectedOn;

  int offset;

  unsigned int remainingOnDuration, remainingInactivity, remainingShortCycles;

  unsigned int currentTime;

  Cycle cycleType;

  EnableShortCycle enable;

  State currentStatus;

  NetworkNode *m_device;

  double m_nextOnTime;

  unsigned int tActiveToSleep;
  unsigned int tSleepToActive;
  
  int m_sleepType;
};

#endif