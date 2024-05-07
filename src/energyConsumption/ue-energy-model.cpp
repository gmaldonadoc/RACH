/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016
 *
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
 * Author: Tiago Pedroso da Cruz de Andrade
 */

#include <cmath>
#include <iostream>

#include "ue-energy-model.h"
#include "../core/eventScheduler/simulator.h"
#include "../drx/drx-manager.h"
#include "../device/UserEquipment.h"

EnergyModel::EnergyModel() {

}

EnergyModel::EnergyModel(DeviceModel model) {
  m_deviceModel = model;
}

EnergyModel::~EnergyModel() {

}

void
EnergyModel::UpdateEnergyConsumptionOnPrach(double transmissionPower) {
  double p = 0;
  double a, b, c, t;

  switch (m_deviceModel) {
    case DEVICE_MOTOX2G:
      a = 103.8;
      b = 0.437;
      c = 707.4;
      t = pow(10, (transmissionPower / 10)); // in mW

      p = (a * pow(t, b)) + c; // in mW
      break;

    case DEVICE_MOTOXFORCE:
      a = 28.36;
      b = 0.674;
      c = 645.8;
      t = pow(10, (transmissionPower / 10)); // in mW

      p = (a * pow(t, b)) + c; // in mW
      break;
  }

  //std::cout << m_userEquipment->GetIDNetworkNode() << " Power " << transmissionPower << " Consumption-Prach " << (p / 1000.0) * 0.001 << std::endl;
  m_consumedEnergyOnPrach += p;
  m_totalConsumedEnergy += (p / 1000.0) * 0.001;
}

void
EnergyModel::UpdateEnergyConsumptionOnPusch(double transmissionPower, int mcs, int nPrbs) {
  double p = 0;
  double a, b, c, t;

  switch (m_deviceModel) {
    case DEVICE_MOTOX2G:
      a = 103.8;
      b = 0.437;
      c = 707.4;
      t = pow(10, (transmissionPower / 10)); // in mW

      p = (a * pow(t, b)) + c; // in mW
      break;

    case DEVICE_MOTOXFORCE:
      a = 65.5;
      b = 0.58;
      c = 788.4;
      t = pow(10, (transmissionPower / 10)); // in mW

      p = (a * pow(t, b)) + c; // in mW
      break;
  }

  //std::cout << "PUSCH CONSUMPTION UE " << m_userEquipment->GetIDNetworkNode() << " PRBs " << nPrbs << " POWER " << transmissionPower << " ENERGY " << (p / 1000.0) * 0.001 << std::endl;
  m_consumedEnergyOnPusch += p;
  m_totalConsumedEnergy += (p / 1000.0) * 0.001;
}

void
EnergyModel::UpdateEnergyConsumptionOnPucch(double transmissionPower, int mcs, int nPrbs) {
  double p = 0;
  double a, b, c, t;

  switch (m_deviceModel) {
    case DEVICE_MOTOX2G:
      a = 103.8;
      b = 0.437;
      c = 707.4;
      t = pow(10, (transmissionPower / 10)); // in mW

      p = (a * pow(t, b)) + c; // in mW
      break;

    case DEVICE_MOTOXFORCE:
      a = 65.5;
      b = 0.58;
      c = 788.4;
      t = pow(10, (transmissionPower / 10)); // in mW

      p = (a * pow(t, b)) + c; // in mW
      break;
  }

  std::cout << "PUCCH CONSUMPTION UE " << m_userEquipment->GetIDNetworkNode() << " PRBs " << nPrbs << " POWER " << transmissionPower << " ENERGY " << (p / 1000.0) * 0.001 << std::endl;
  m_consumedEnergyOnPucch += p;
  m_totalConsumedEnergy += (p / 1000.0) * 0.001;
}

void
EnergyModel::UpdateEnergyConsumptionOnPdsch(double receptionPower, int mcs, int nPrbs) {
  double p = 300; // in mW

  //std::cout << m_userEquipment->GetIDNetworkNode() << " Power " << receptionPower << " Consumption-Pdsch " << p << std::endl;
  m_consumedEnergyOnPdsch += p;
  m_totalConsumedEnergy += (p / 1000.0) * 0.001; // in Joule
}

void
EnergyModel::UpdateEnergyConsumptionOnPdcch(double receptionPower) {
  double p = 650; // in mW

  //if (((UserEquipment *) m_userEquipment)->GetDrx()->GetDrxCycleState() == DRX::ACTIVE) {
  std::cout << Simulator::Init()->Now() << " " << m_userEquipment->GetIDNetworkNode() << " Power " << receptionPower << " Consumption-Pdcch " << p << std::endl;
  m_consumedEnergyOnPdcch += p;
  m_totalConsumedEnergy += (p / 1000.0) * 0.001;
  //}
}

void
EnergyModel::SetDevice(NetworkNode *userEquipment) {
  m_userEquipment = userEquipment;
}

double
EnergyModel::GetTotalConsumedEnergy(void) {
  return m_totalConsumedEnergy;
}