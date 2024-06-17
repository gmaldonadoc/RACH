/*

 */

using namespace std;

#include <iostream>
#include "ue-consumption-model.h"
#include "../core/eventScheduler/simulator.h"
#include "baterry-model.h"
#include "../device/UserEquipment.h"

void ConsumptionModel::init() {
  tLightSleep = 0;
  tDeepSleep = 0;
  tActiveNoData = 0;
  tActiveRx = 0;
  tActiveTx = 0;
  tActiveRxTx = 0;
  tDeepLight = 0;
  tLightActive = 0;
  tSleepToActive = 0;
  tActiveToSleep = 0;

  lastChange = 0.0;

  stateDuration = 0;

  consumo = 0.0;
  m_totalConsumption = 0.0;
  m_connectedConsumption = 0.0;

  m_currentRxMCS = 0;
  m_currentRxUsedPRBs = 0;
  m_currentRxRate = 0;
  m_consumptionInRx = 0;
  m_consumptionInTx = 0;
  m_consumptionInRxTx = 0;
  m_consumptionInNoData = 0;

  transLightActive = false;

  state = DRXManager::ACTIVE_NO_DATA;
  action = DRXManager::PDCCH_NO_DATA;

  m_device = NULL;
}

ConsumptionModel::ConsumptionModel() {
  init();

  //cActiveNoData = 300;
  //cActiveRx = 500;
  //cActiveTx = 3000;
  //cActiveRxTx = 3000;

  cDeepSleep = 40;
  cLightSleep = 180;
  cDeepLight = 22;
  cLightActive = 39;

  cWakeupLight = 353.9;
  cPowerdownLight = 354.09;

  cWakeupDeep = 289.48;
  cPowerdownDeep = 344.23;
}

void
ConsumptionModel::setStateUE(DRXManager::State newState, DRXManager::Action newAction) {
  //double current_time = floor(Simulator::Init()->Now() * 10000) / 10000;
  double current_time = Simulator::Init()->Now();

  if (current_time < getLastChange()) {
    cout << "Tempo invalido" << endl;
    return;
  }

  if (newState >= DRXManager::N_DRX_STATES || newState < 0) {
    cout << "Estado invalido" << endl;
    return;
  }

  double t = (int(current_time * 1000) - int(getLastChange() * 1000)) / 1000;

  //cout << "Current " << current_time << " Last Change " << getLastChange() << " t " << t << endl;

  //faz transicao de estados(se puder)
  switch (state) {
    case DRXManager::DEEP:
      //cout << "DEEP" << endl;
      state = newState;
      action = newAction;
      break;
    case DRXManager::LIGHT: //se ele esta em Light sleep, pode assumir qualquer estado (nunca entra em deep)
      //cout << "LIGHT" << endl;
      state = newState;
      action = newAction;
      if (newState == DRXManager::ACTIVE_TX || newState == DRXManager::ACTIVE_RXTX || newState == DRXManager::ACTIVE_RX || newState == DRXManager::ACTIVE_NO_DATA) {
        transLightActive = true;
      }
      break;
    case DRXManager::ACTIVE_RX://se ele esta em RX
      //cout << "RX" << endl;
      if (newState == DRXManager::ACTIVE_TX || newState == DRXManager::ACTIVE_RXTX) {
        if (t == 0.0) {
          state = DRXManager::ACTIVE_RXTX;
        } else {
          state = newState;
        }
        action = newAction;
      } else {
        if (newState != DRXManager::ACTIVE_NO_DATA) {
          state = newState;
          action = newAction;
        }
      }
      break;
    case DRXManager::ACTIVE_TX://se ele esta em TX
      //cout << "TX" << endl;
      if (newState == DRXManager::ACTIVE_RX || newState == DRXManager::ACTIVE_RXTX) {
        if (t == 0.0) {
          state = DRXManager::ACTIVE_RXTX;
        } else {
          state = newState;
        }
        action = newAction;
      } else {
        if (newState != DRXManager::ACTIVE_NO_DATA) {
          state = newState;
          action = newAction;
        }
      }
      break;
    case DRXManager::ACTIVE_NO_DATA:
      //cout << "NODATA" << endl;
      state = newState;
      action = newAction;
      break;
    case DRXManager::ACTIVE_RXTX:
      //cout << "RXTX" << endl;
      if (t != 0) {
        state = newState;
        action = newAction;
      }
      break;
    case DRXManager::WAKEUP_LIGHT:
      //cout << "WAKEUP" << endl;
      state = newState;
      action = newAction;
      break;
    case DRXManager::POWERDOWN_LIGHT:
      //cout << "POWERDOWN" << endl;
      state = newState;
      action = newAction;
      break;
    case DRXManager::WAKEUP_DEEP:
      state = newState;
      action = newAction;
      break;
    case DRXManager::POWERDOWN_DEEP:
      state = newState;
      action = newAction;
      break;
  }

  setLastChange(current_time);
}

void
ConsumptionModel::settLightSleep(unsigned int t) {
  tLightSleep = t;
}

void
ConsumptionModel::settDeepSleep(unsigned int t) {
  tDeepSleep = t;
}

void
ConsumptionModel::settActiveNoData(unsigned int t) {
  tActiveNoData = t;
}

void
ConsumptionModel::settActiveRx(unsigned int t) {
  tActiveRx = t;
}

void
ConsumptionModel::settActiveTx(unsigned int t) {
  tActiveTx = t;
}

void
ConsumptionModel::settActiveRxTx(unsigned int t) {
  tActiveRxTx = t;
}

unsigned int
ConsumptionModel::gettLightSleep() {
  return tLightSleep;
}

unsigned int
ConsumptionModel::gettDeepSleep() {
  return tDeepSleep;
}

unsigned int
ConsumptionModel::gettActiveNoData() {
  return tActiveNoData;
}

unsigned int
ConsumptionModel::gettActiveRx() {
  return tActiveRx;
}

unsigned int
ConsumptionModel::gettActiveTx() {
  return tActiveTx;
}

unsigned int
ConsumptionModel::gettActiveRxTx() {
  return tActiveRxTx;
}

unsigned int
ConsumptionModel::gettSleepToActive() {
  return tSleepToActive;
}

unsigned int
ConsumptionModel::gettActiveToSleep() {
  return tActiveToSleep;
}

void
ConsumptionModel::setcDeepLight(unsigned int c) {
  cDeepLight = c;
}

void
ConsumptionModel::setcLightActive(unsigned int c) {
  cLightActive = c;
}

unsigned int
ConsumptionModel::getcDeepLight() {
  return cDeepLight;
}

unsigned int
ConsumptionModel::getcLightActive() {
  return cLightActive;
}

void
ConsumptionModel::setcDeepSleep(unsigned int c) {
  cDeepSleep = c;
}

void
ConsumptionModel::setcLightSleep(unsigned int c) {
  cLightSleep = c;
}

void
ConsumptionModel::setcActiveNoData(unsigned int c) {
  cActiveNoData = c;
}

void
ConsumptionModel::setcActiveRx(unsigned int c) {
  cActiveRx = c;
}

void
ConsumptionModel::setcActiveTx(unsigned int c) {
  cActiveTx = c;
}

void
ConsumptionModel::setcActiveRxTx(unsigned int c) {
  cActiveRxTx = c;
}

unsigned int
ConsumptionModel::getcDeepSleep() {
  return cDeepSleep;
}

unsigned int
ConsumptionModel::getcLightSleep() {
  return cLightSleep;
}

unsigned int
ConsumptionModel::getcActiveNoData() {
  return cActiveNoData;
}

unsigned int
ConsumptionModel::getcActiveRx() {
  return cActiveRx;
}

unsigned int
ConsumptionModel::getcActiveTx() {
  return cActiveTx;
}

unsigned int
ConsumptionModel::getcActiveRxTx() {
  return cActiveRxTx;
}

void
ConsumptionModel::setLastChange(double l) {
  lastChange = l;
}

void
ConsumptionModel::setStateDuration(unsigned int c) {
  stateDuration = c;
}

double
ConsumptionModel::getLastChange() {
  return lastChange;
}

unsigned int
ConsumptionModel::getStateDuration() {
  return stateDuration;
}

void
ConsumptionModel::setConsumo() {
  consumo = tDeepSleep * cDeepSleep + tLightSleep * cLightSleep + tActiveNoData * cActiveNoData + tActiveRx * cActiveRx + tActiveTx * cActiveTx + tActiveRxTx * cActiveRxTx;
}

double
ConsumptionModel::getConsumo() {
  //setConsumo();
  return consumo;
}

double
ConsumptionModel::getConsumption() {
  //m_totalConsumption += (tDeepSleep / 1000) * (cDeepSleep / 1000) + (tLightSleep / 1000) * (cLightSleep / 1000) + (tSleepToActive / 1000) * (cSleepToActive / 1000);
  //m_totalConsumption += (double) ((tDeepSleep * cDeepSleep) + (tLightSleep * cLightSleep) + (tSleepToActive * cSleepToActive) + (tActiveToSleep * cActiveToSleep)) / ((double) 1000000);

  return m_totalConsumption;
}

double
ConsumptionModel::getConnectedConsumption() {
  return m_connectedConsumption;
}

double
ConsumptionModel::getConsumptionInTx(){
  return m_consumptionInTx;
}

double
ConsumptionModel::getConsumptionInRx(){
  return m_consumptionInRx;
}

double
ConsumptionModel::getConsumptionInRxTx(){
  return m_consumptionInRxTx;
}

double
ConsumptionModel::getConsumptionInNoData(){
  return m_consumptionInNoData;
}

void
ConsumptionModel::settLightActive() {
  tLightActive++;
}

unsigned int
ConsumptionModel::gettLightActive() {
  return tLightActive;
}

void
ConsumptionModel::updateEnergyConsumption(double t) {
  double current_time = Simulator::Init()->Now();

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << current_time << " [EC] Call for updating Energy Consumption" << endl;
#endif

  if (transLightActive) {
    settLightActive();
  }

  transLightActive = false;

  switch (state) {
    case DRXManager::DEEP:
      settDeepSleep(gettDeepSleep() + t);
      measureConsumptionInDeepSleep(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] DEEP " << t << endl;
#endif
      break;
    case DRXManager::LIGHT:
      settLightSleep(gettLightSleep() + t);
      measureConsumptionInLightSleep(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] LIGHT " << t << endl;
#endif
      break;
    case DRXManager::ACTIVE_NO_DATA:
      settActiveNoData(gettActiveNoData() + t);
      measureConsumptionInRx(t);
      //measureConsumptionInRxOnlyPdcch(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] NO_DATA " << t << endl;
#endif
      break;
    case DRXManager::ACTIVE_RX:
      settActiveRx(gettActiveRx() + t);
      measureConsumptionInRx(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] RX " << t << endl;
#endif
      break;
    case DRXManager::ACTIVE_TX:
      settActiveTx(gettActiveTx() + t);
      measureConsumptionInTx(t);
      if (action == DRXManager::PUSCH_TX) {
        //measureConsumptionInTx(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
        cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] TX_PUSCH " << t << endl;
#endif
      } else if (action == DRXManager::PRACH_TX) {
        //measureConsumptionInPrach(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
        cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] TX_PRACH " << t << endl;
#endif
      }
      break;
    case DRXManager::ACTIVE_RXTX:
      settActiveRxTx(gettActiveRxTx() + t);
      measureConsumptionInTxRx(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] RXTX " << t << endl;
#endif
      break;
    case DRXManager::WAKEUP_LIGHT:
      tSleepToActive += t;
      measureConsumptionInTransaction(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] WAKEUP_LIGHT " << t << endl;
#endif
      break;
    case DRXManager::POWERDOWN_LIGHT:
      tActiveToSleep += t;
      measureConsumptionInTransaction(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] POWERDOWN_LIGHT " << t << endl;
#endif
      break;
    case DRXManager::WAKEUP_DEEP:
      tSleepToActive += t;
      measureConsumptionInTransaction(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] WAKEUP_DEEP " << t << endl;
#endif
      break;
    case DRXManager::POWERDOWN_DEEP:
      tActiveToSleep += t;
      measureConsumptionInTransaction(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] POWERDOWN_DEEP " << t << endl;
#endif
      break;
    case DRXManager::SYNC:
      //settActiveNoData(gettActiveNoData() + t);
      //measureConsumptionInRx(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] SYNCHRONIZATION " << t << endl;
#endif
      break;
    case DRXManager::MEAS:
      //settActiveNoData(gettActiveNoData() + t);
      //measureConsumptionInRx(t);
#ifdef ENERGY_CONSUMPTION_DEBUG
      cerr << current_time << " " << m_device->GetIDNetworkNode() << " [EC] MEAS " << t << endl;
#endif
      break;
    default:
      cerr << current_time << " [EC] ERROR STATUS " << t << endl;
      break;
  }

  state = DRXManager::ACTIVE_NO_DATA;
  action = DRXManager::PDCCH_NO_DATA;

  if (UPLINK == true || DOWNLINK == true) {
    Simulator::Init()->Schedule(0.001, &ConsumptionModel::updateEnergyConsumption, this, 1);
  } else if ((RA_ENERGY_TRACE) || ((UeMacEntity*) m_device->GetProtocolStack()->GetMacEntity())->getRAStopped() == false) {
    Simulator::Init()->Schedule(0.001, &ConsumptionModel::updateEnergyConsumption, this, 1);
  }

}

void
ConsumptionModel::SetDevice(UserEquipment *device) {
  m_device = device;
}

void
ConsumptionModel::SetCurrentTxPower(double txPower) {
  m_currentTxPower = txPower;
}

double
ConsumptionModel::GetCurrentTxPower() {
  return m_currentTxPower;
}

void
ConsumptionModel::SetCurrentRxPower(double rxPower) {
  m_currentRxPower = rxPower;
}

double
ConsumptionModel::GetCurrentRxPower() {
  return m_currentRxPower;
}

void ConsumptionModel::SetCurrentRxRate(double rxRate){
  m_currentRxRate = rxRate;
}

double ConsumptionModel::GetCurrentRxRate(void){
  return m_currentRxRate;
}

void ConsumptionModel::SetCurrentRxUsedPRBs(int nPRBs){
  m_currentRxUsedPRBs = nPRBs;
}

int ConsumptionModel::GetCurrentRxUsedPRBs(void){
  return m_currentRxUsedPRBs;
}

void ConsumptionModel::SetCurrentRxMCS(int mcs){
  m_currentRxMCS = mcs;
}

int ConsumptionModel::GetCurrentRxMCS(void){
  return m_currentRxMCS;
}

/*
 */

void
ConsumptionModel::measureConsumptionInTx(double t) {
  double p1, p2, p3, pc;
  double x = pow(10, (GetCurrentTxPower() / 10)); // transmission power in mW

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << Simulator::Init()->Now() << " [EC] ID " << m_device->GetIDNetworkNode();
#endif

  if (action == DRXManager::PUSCH_TX) {
#ifdef ENERGY_CONSUMPTION_DEBUG
    cerr << " TX PUSCH ";
#endif
    if (x > 0 && x <= 2) {
      p1 = 44.09;
      p2 = 607.3;

      pc = (p1 * x) + p2;
    } else if (x > 2) {
      p1 = -0.01768;
      p2 = 7.184;
      p3 = 677.3;

      pc = (p1 * x * x) + (p2 * x) + p3;
    }
  } else if (action == DRXManager::PRACH_TX) {
#ifdef ENERGY_CONSUMPTION_DEBUG
    cerr << " TX PRACH ";
#endif
    p1 = 28.36;
    p2 = 0.674;
    p3 = 645.8;

    pc = (p1 * pow(x, p2)) + p3; // energy power consumption in mW
  }

  double en = (pc / 1000) * (t / 1000); // in J

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << "PC " << pc << " EN " << en << endl;
#endif

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  //m_totalConsumption += en;
}

void
ConsumptionModel::measureConsumptionInRx(double t) {
  double p1, p2, pc;
  double x = m_device->GetReferenceSignalReceivedPower(); // in dBm

  if (state == DRXManager::ACTIVE_NO_DATA) {
    p1 = -0.1803;
    p2 = 441.5;

  } else if (state == DRXManager::ACTIVE_RX) {
    p1 = -0.3212;
    p2 = 708.7;
  }

  pc = (p1 * x) + p2; // in mW

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  //m_totalConsumption += en;
}

void
ConsumptionModel::measureConsumptionInTxRx(double t) {
  double p1, p2, p3, p4, pc;
  double x = GetCurrentTxPower(); // transmission power in dBm

  p1 = 708.4;
  p2 = 0.005494;
  p3 = 13.16;
  p4 = 0.2104;

  pc = (p1 * exp(p2 * x)) + (p3 * exp(p4 * x)); // energy power consumption in mW

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  //m_totalConsumption += en;
}

void
ConsumptionModel::measureConsumptionInRxOnlyPdcch(double t) {
  double p1, p2, pc;
  double x = m_device->GetReferenceSignalReceivedPower(); // in dBm

  p1 = -0.1803;
  p2 = 441.5;

  pc = (p1 * x) + p2; // in mW

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  //m_totalConsumption += en;
}

void
ConsumptionModel::measureConsumptionInLightSleep(double t) {
  double en = (cLightSleep / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(cLightSleep / 1000, t);
  }

  //m_totalConsumption += en;
}

void
ConsumptionModel::measureConsumptionInDeepSleep(double t) {
  double en = (cDeepSleep / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(cDeepSleep / 1000, t);
  }

  //m_totalConsumption += en;
}

void
ConsumptionModel::measureConsumptionInTransaction(double t) {
  double en;

  if (state == DRXManager::WAKEUP_LIGHT) {
    en = (cWakeupLight / 1000) * (t / 1000); // in J

    if (m_device->GetBatteryModel() != NULL) {
      m_device->GetBatteryModel()->UpdateCapacity(cWakeupLight / 1000, t);
    }
  } else if (state == DRXManager::POWERDOWN_LIGHT) {
    en = (cPowerdownLight / 1000) * (t / 1000); // in J

    if (m_device->GetBatteryModel() != NULL) {
      m_device->GetBatteryModel()->UpdateCapacity(cPowerdownLight / 1000, t);
    }

  } else if (state == DRXManager::WAKEUP_DEEP) {
    en = (cWakeupDeep / 1000) * (t / 1000); // in J

    if (m_device->GetBatteryModel() != NULL) {
      m_device->GetBatteryModel()->UpdateCapacity(cWakeupDeep / 1000, t);
    }
  } else if (state == DRXManager::POWERDOWN_DEEP) {
    en = (cPowerdownDeep / 1000) * (t / 1000); // in J

    if (m_device->GetBatteryModel() != NULL) {
      m_device->GetBatteryModel()->UpdateCapacity(cPowerdownDeep / 1000, t);
    }
  }

  //m_totalConsumption += en;
}
