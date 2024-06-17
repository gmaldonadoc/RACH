/*
 
 */

#include <iostream>
#include "ue-3gpp-model.h"

SimpleModel::SimpleModel() {
  init();

  cDeepSleep = 40;
  cLightSleep = 180;
  cDeepLight = 22;
  cLightActive = 39;
  cWakeupLight = 0;
  cPowerdownLight = 22;
  cWakeupDeep = 0;
  cPowerdownDeep = 22;
}

SimpleModel::~SimpleModel() {

}

/*
 */

void
SimpleModel::measureConsumptionInTx(double t) {
  double pc = 3000; // energy power consumption in mW

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  m_totalConsumption += en;
}

void
SimpleModel::measureConsumptionInRx(double t) {
  double pc = 500; // energy power consumption in mW

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  m_totalConsumption += en;
}

void
SimpleModel::measureConsumptionInTxRx(double t) {
  double pc = 3000; // energy power consumption in mW

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  m_totalConsumption += en;
}

void
SimpleModel::measureConsumptionInRxOnlyPdcch(double t) {
  double pc = 255; // energy power consumption in mW

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  m_totalConsumption += en;
}

void
SimpleModel::measureConsumptionInLightSleep(double t) {
  double en = (cLightSleep / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(cLightSleep / 1000, t);
  }

  m_totalConsumption += en;
}

void
SimpleModel::measureConsumptionInDeepSleep(double t) {
  double en = (cDeepSleep / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(cDeepSleep / 1000, t);
  }

  m_totalConsumption += en;
}

void
SimpleModel::measureConsumptionInTransaction(double t) {
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

/*
 */
