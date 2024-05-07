/*
 
 */

#include <iostream>
#include "ue-miletus-model.h"

MiletusModel::MiletusModel() {
  init();

  cDeepSleep = 40;
  cLightSleep = 180;
  cDeepLight = 22;
  cLightActive = 39;
  cWakeupLight = 353.9;
  cPowerdownLight = 354.09;
  cWakeupDeep = 289.48;
  cPowerdownDeep = 344.23;
}

MiletusModel::~MiletusModel() {

}

/*
 */

void
MiletusModel::measureConsumptionInTx(double t) {
  double p1, p2, p3, pc;
  double x = pow(10, (GetCurrentTxPower() / 10)); // transmission power in mW

  if (action == DRXManager::PUSCH_TX || action == DRXManager::PUCCH_TX) {
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
    p1 = 28.36;
    p2 = 0.674;
    p3 = 645.8;

    pc = (p1 * pow(x, p2)) + p3; // energy power consumption in mW
  }

  //std::cout << "Device " << m_device->GetIDNetworkNode() << " TX Power [dBm] " << GetCurrentTxPower() << " TX Power [mW] " << x << " Consumption " << pc << std::endl;

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  m_totalConsumption += en;
}

void
MiletusModel::measureConsumptionInRx(double t) {
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

  m_totalConsumption += en;
}

void
MiletusModel::measureConsumptionInTxRx(double t) {
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

  m_totalConsumption += en;
}

void
MiletusModel::measureConsumptionInRxOnlyPdcch(double t) {
  double p1, p2, pc;
  double x = m_device->GetReferenceSignalReceivedPower(); // in dBm

  p1 = -0.1803;
  p2 = 441.5;

  pc = (p1 * x) + p2; // in mW

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  m_totalConsumption += en;
}

void
MiletusModel::measureConsumptionInLightSleep(double t) {
  double en = (cLightSleep / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(cLightSleep / 1000, t);
  }

  m_totalConsumption += en;
}

void
MiletusModel::measureConsumptionInDeepSleep(double t) {
  double en = (cDeepSleep / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(cDeepSleep / 1000, t);
  }

  m_totalConsumption += en;
}

void
MiletusModel::measureConsumptionInTransaction(double t) {
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

  m_totalConsumption += en;
}