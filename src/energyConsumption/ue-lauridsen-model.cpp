/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This file is part of LTE-Sim
 *
 * LTE-Sim is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * LTE-Sim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Fernando Pereira <fernandopereira@lrc.ic.unicamp.br>
 *
 * File:   ue-lauridsen-model.cpp
 * Author: Fernando Pereira
 *
 * Created on June 20, 2017, 13:47
 */

#include <iostream>
#include "ue-lauridsen-model.h"

LauridsenModel::LauridsenModel() {
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

LauridsenModel::~LauridsenModel() {

}

double LauridsenModel::getPUSCHConsumptionIdle(void) {
  return 853.0;
}

double LauridsenModel::getPUSCHConsumptionInTx(double stx) {
  double a, b, c, pc; // for ax^2 + bx + c

  pc = 853.0;
  // Finding P_{TxRF}
  if (stx <= 0.2) {
    a = 0.0;
    b = 0.78;
    c = 23.6;
  } else if (stx > 0.2 && stx <= 11.4) {
    a = 0.0;
    b = 17.0;
    c = 45.4;
  } else {
    a = 5.9;
    b = -118.0;
    c = 1195;
  }

  // Model for transmission
  pc += 29.9 + 0.62 + a * stx * stx + b * stx + c;

  return pc;
}

void
LauridsenModel::measureConsumptionInTx(double t) {
  double a, b, c, pc = 0; // for ax^2 + bx + c
  double stx = GetCurrentTxPower(); // In dBm
  double x = pow(10, (GetCurrentTxPower() / 10)); // transmission power in mW
  double en = 0;

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << Simulator::Init()->Now() << " [EC-EE] TX ID " << m_device->GetIDNetworkNode() << endl
          << "\tPTX = " << stx << endl;
#endif


  pc = 853.0;
  // Finding P_{TxRF}
  if (stx <= 0.2) {
    a = 0.0;
    b = 0.78;
    c = 23.6;
  } else if (stx > 0.2 && stx <= 11.4) {
    a = 0.0;
    b = 17.0;
    c = 45.4;
  } else {
    a = 5.9;
    b = -118.0;
    c = 1195;
  }

  // Model for transmission
  pc += 29.9 + 0.62 + a * stx * stx + b * stx + c;
  en = (pc / 1000) * (t / 1000); // in J
  m_connectedConsumption += en;
  m_consumptionInTx += en;


#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << "PC " << pc << " EN " << en << endl;
#endif

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  m_totalConsumption += en;
#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << "\tPC after TX = " << pc << endl
          << "\t cons ~now = " << m_totalConsumption << endl;
#endif

}

void
LauridsenModel::measureConsumptionInRx(double t) {
  double a, b, pc;
  double srx = GetCurrentRxPower(); // in dBm
  double rrx = GetCurrentRxRate() / 1000.0; // in bits/s
  double en = 0;

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << Simulator::Init()->Now() << " [EC-EE] RX ID " << m_device->GetIDNetworkNode() << endl
          << "\tPRX = " << srx << endl
          << "\tRRX = " << rrx << endl;
#endif

  pc = 853;

  if (state == DRXManager::ACTIVE_RX) {
    pc += 25.1; // Active state

    if (srx <= -52.5) {
      a = -0.04;
      b = 24.8;
    } else {
      a = -0.11;
      b = 7.86;
    }
    pc += (a * srx + b);

    a = 0.97;
    b = 8.16;
    pc += (a * rrx + b);

    en = (pc / 1000) * (t / 1000); // in J
    m_connectedConsumption += en;
    m_consumptionInRx += en;
  } else if (state == DRXManager::ACTIVE_NO_DATA) {
    en = (pc / 1000) * (t / 1000); // in J
    m_consumptionInNoData += en;
  }

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << "PC " << pc << " EN " << en << endl;
#endif

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }

  m_totalConsumption += en;
#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << "\tPC after RX = " << pc << endl
       << "\t cons ~now = " << m_totalConsumption << endl;
#endif
}

void
LauridsenModel::measureConsumptionInTxRx(double t) {
  double a, b, c, pc; // for ax^2 + bx + c
  double stx = GetCurrentTxPower(); // In dBm
  double srx = GetCurrentRxPower(); // in dBm
  double rrx = GetCurrentRxRate() / 1000.0; // in Mbit/s

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << Simulator::Init()->Now() << " [EC] ID " << m_device->GetIDNetworkNode() << endl
          << "\tPTX = " << stx << endl
          << "\tPRX = " << srx << endl
          << "\tRRX = " << rrx << endl;
#endif

#ifdef POWER_CONTROL_DEBUG
  cerr << Simulator::Init()->Now() << " [PCD-TXRX] ID "
          << m_device->GetIDNetworkNode()
          << " PTX " << stx << endl;
#endif

  pc = 853.0;
  // Finding P_{TxRF}
  if (stx <= 0.2) {
    a = 0.0;
    b = 0.78;
    c = 23.6;
  } else if (stx > 0.2 && stx <= 11.4) {
    a = 0.0;
    b = 17.0;
    c = 45.4;
  } else {
    a = 5.9;
    b = -118.0;
    c = 1195;
  }

  // Model for transmission
  pc += 29.9 + 0.62 + a * stx * stx + b * stx + c;

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << "\tPC after TX = " << pc << endl;
#endif

  pc += 25.1; // Active state

  if (srx <= -52.5) {
    a = -0.04;
    b = 24.8;
  } else {
    a = -0.11;
    b = 7.86;
  }
  pc += (a * srx + b);

  a = 0.97;
  b = 8.16;
  pc += (a * rrx + b);

  double en = (pc / 1000) * (t / 1000); // in J

  if (m_device->GetBatteryModel() != NULL) {
    m_device->GetBatteryModel()->UpdateCapacity(pc / 1000, t);
  }
  m_consumptionInRxTx += en;
  m_totalConsumption += en;
  m_connectedConsumption += en;

#ifdef ENERGY_CONSUMPTION_DEBUG
  cerr << "\tPC after TXRX = " << pc << endl
          << "\t cons ~now = " << m_totalConsumption << endl;
#endif

}

void
LauridsenModel::measureConsumptionInRxOnlyPdcch(double t) {

}

void
LauridsenModel::measureConsumptionInLightSleep(double t) {
  double en = (cLightSleep / 1000) * (t / 1000); // in J
}

void
LauridsenModel::measureConsumptionInDeepSleep(double t) {
  double en = (cDeepSleep / 1000) * (t / 1000); // in J
}

void
LauridsenModel::measureConsumptionInTransaction(double t) {
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

  // m_totalConsumption += en;
}
