#include "baterry-model.h"

#include "../core/eventScheduler/simulator.h"

BatteryModel::BatteryModel() {
  SetMaximumCapacity(3750);
}

BatteryModel::~BatteryModel() {

}

void
BatteryModel::UpdateCapacity(double ePower, double t) {
  double mAh = EnergyCurrentConsumption(ePower) * (t / 3600000);
  m_currentCapacity -= mAh;

#ifdef BATTERY_DEBUG
  cerr << Simulator::Init()->Now() << " [BATTERY] consumption " << mAh << " mAh and current energy " << m_currentCapacity << " mAh" << endl;
#endif
}

double
BatteryModel::EnergyCurrentConsumption(double ePower) {
  double voltage = GetBatteryCurrentVoltage(); // in millivolts

  double a = ePower / (voltage / 1000); // in A
  double ah = (a * 1000); // in mA

  return ah; // in mA
}

void
BatteryModel::SetMaximumCapacity(double energy) {
  m_maxCapacity = energy;
  m_currentCapacity = m_maxCapacity;
}

void
BatteryModel::SetDevice(UserEquipment *device) {
  m_device = device;
}

double
BatteryModel::GetBatteryCharge() {
  double c = (m_currentCapacity * 100) / m_maxCapacity;
  return c;
}

double
BatteryModel::GetBatteryCurrentVoltage() {
  double p1, p2, p3;
  double batteryCharge = GetBatteryCharge();

  if (batteryCharge > 0 && batteryCharge <= 7) {
    p1 = -7.585;
    p2 = 147.7;
    p3 = 3008;
  } else if (batteryCharge > 7 && batteryCharge <= 100) {
    p1 = 0.06806;
    p2 = 0.2405;
    p3 = 3661;
  }

  return ((p1 * batteryCharge * batteryCharge) + (p2 * batteryCharge) + p3); // in mV
}

double
BatteryModel::GetCurrentCapacity() {
  return m_currentCapacity;
}