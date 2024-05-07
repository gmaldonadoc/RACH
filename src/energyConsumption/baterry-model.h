/*
 * File:   baterry-model.h
 * Author: tiago
 *
 * Created on September 6, 2016, 6:00 PM
 */

#ifndef BATERRY_MODEL_H
#define	BATERRY_MODEL_H

#include "../device/NetworkNode.h"
#include "../device/UserEquipment.h"

class BatteryModel {
public:

  BatteryModel();

  virtual ~BatteryModel();

  void UpdateCapacity(double ePower, double t);

  double EnergyCurrentConsumption(double ePower);
  
  double GetBatteryCharge(void);
  
  void SetDevice(UserEquipment *device);
  
  double GetBatteryCurrentVoltage(void);
  
  double GetCurrentCapacity(void);

private:

  void SetMaximumCapacity(double energy);

  UserEquipment *m_device;

  double m_maxCapacity; //carga maxima da bateria in mAh
  double m_currentCapacity; //carga atual da bateria in mAh
};

#endif