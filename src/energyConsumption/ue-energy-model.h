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

#ifndef UE_ENERGY_MODEL_H
#define	UE_ENERGY_MODEL_H

#include "../device/NetworkNode.h"

//class UserEquipment;

class EnergyModel {
public:

  enum DeviceModel {
    DEVICE_MOTOX2G,
    DEVICE_MOTOXFORCE
  };

  EnergyModel();
  EnergyModel(DeviceModel model);

  virtual ~EnergyModel();

  void UpdateEnergyConsumptionOnPrach(double transmissionPower);

  void UpdateEnergyConsumptionOnPusch(double transmissionPower, int mcs, int nPrbs);

  void UpdateEnergyConsumptionOnPucch(double transmissionPower, int mcs, int nPrbs);
  
  void UpdateEnergyConsumptionOnPdsch(double receptionPower, int mcs, int nPrbs);

  void UpdateEnergyConsumptionOnPdcch(double receptionPower);

  void SetDevice(NetworkNode *userEquipment);

  double GetTotalConsumedEnergy(void);

private:
  NetworkNode *m_userEquipment;
  DeviceModel m_deviceModel;

  double m_initialEnergy;
  double m_totalConsumedEnergy; // in mW
  double m_consumedEnergyOnPrach; // in mW
  double m_consumedEnergyOnPusch; // in mW
  double m_consumedEnergyOnPucch; // in mW
  double m_consumedEnergyOnPdcch; // in mW
  double m_consumedEnergyOnPdsch; // in mW
};

#endif