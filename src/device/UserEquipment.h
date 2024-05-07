/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
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
 * Author: Giuseppe Piro <g.piro@poliba.it>
 */

#ifndef USEREQUIPMENT_H_
#define USEREQUIPMENT_H_

#include "NetworkNode.h"

#include "../drx/drx-manager.h"
#include "../energyConsumption/ue-energy-model.h"
#include "../energyConsumption/baterry-model.h"

class ENodeB;
class Gateway;
class CqiManager;
class ConsumptionModel;
class DRXManager;
class BatteryModel;

static int PagingOccasion[3][4] = {
    {9, -1, -1, -1},
    {4, 9, -1, -1},
    {0, 4, 5, 9}};

class UserEquipment : public NetworkNode
{
public:
  struct UserEquipmentOnCoverage
  {
    UserEquipmentOnCoverage();
    virtual ~UserEquipmentOnCoverage();
    UserEquipmentOnCoverage(UserEquipment *UE);

    UserEquipment *m_UE;
    void SetUE(UserEquipment *UE);
    UserEquipment *GetUE(void) const;

    std::vector<double> m_uplinkChannelEstimation;
    void SetUplinkChannelEstimation(std::vector<double> vet);
    std::vector<double> GetUplinkChannelEstimation(void) const;
  };

  typedef std::vector<UserEquipmentOnCoverage *> UserEquipmentsOnCoverage;

  UserEquipment();
  UserEquipment(int idElement, double posx, double posy, Cell *cell, NetworkNode *target, bool handover, Mobility::MobilityModel model);
  UserEquipment(int idElement, double posx, double posy, int speed, double speedDirection, Cell *cell, NetworkNode *target, bool handover, Mobility::MobilityModel mobilityModel);
  // UserEquipment(int idElement, double posx, double posy, int speed, double speedDirection, Cell *cell, NetworkNode *target, bool handover, Mobility::MobilityModel mobilityModel, EnergyModel::DeviceModel deviceModel);
  // UserEquipment(int idElement, double posx, double posy, int speed, double speedDirection, Cell *cell, NetworkNode *target, bool handover, Mobility::MobilityModel mobilityModel, EnergyModel::DeviceModel deviceModel, bool d2dAble);

  virtual ~UserEquipment();

  void SetTargetNode(NetworkNode *n);
  NetworkNode *GetTargetNode(void);

  void UpdateUserPosition(double time);

  void SetCqiManager(CqiManager *cm);
  CqiManager *GetCqiManager(void);

  void SetIndoorFlag(bool flag);

  bool IsIndoor(void);

  //
  void SetRegister(bool r);
  bool IsRegister(void);

  void StartRandomAccessProcedure(double time);

  /*
   * Author: Tiago Pedroso da Cruz de Andrade
   * 0 - HTC
   * 1 - MTC
   */
  void SetDeviceType(int type);
  int GetDeviceType(void);

  void SetDeviceAccessClass(int accessClass);
  int GetDeviceAccessClass(void);

  void SetSemiSchEnable(bool enable);
  bool GetSemiSchEnable(void);

  int GetPagingFrame(void);
  int GetPagingOccasion(void);

  void FindPagingOpportunitie(void);

  void SchedulingRequestConfiguration(bool allocated, int srProhibitTimer);
  bool IsSchedulingRequestAllocated(void);

  void SchedulingRequestStart(int SRstart, int SRperiodicity);

  int GetSRperiodicity(void);

  int GetSRProhibitTimer(void);

  void SetReferenceSignalReceivedPower(double power);
  double GetReferenceSignalReceivedPower(void);

  void SetReferenceSignalSinr(double sinr);
  double GetReferenceSignalSinr(void);

  void SetEnergyModel(EnergyModel *energyModel);
  EnergyModel *GetEnergyModel(void);

  ConsumptionModel *GetConsumptionModel(void);
  void SetConsumptionModel(std::string model);

  // void SetDrx(DRX *drxMechanism);
  // DRX* GetDrx(void);

  // DRX implementation
  DRXManager *GetDRXManager(void);
  void SetDRXManager(void);

  // Battery Model
  void SetBatteryModel();
  BatteryModel *GetBatteryModel(void);

  bool GetD2DAble(void);
  void SetD2DAble(bool d2dAble);

  // Debug
  void Print(void);

  UserEquipmentsOnCoverage *GetUserEquipmentsOnCoverage(void);
  UserEquipmentOnCoverage *GetUserEquipmentOnCoverage(int idUE);
  void UpdateUplinkChannelEstimation(UserEquipment *UE, std::vector<double> vet);

  std::vector<CartesianCoordinates> GetAllUsersCoordinates(void);

private:
  UserEquipmentsOnCoverage *m_userEquipmentsOnCoverage;

  NetworkNode *m_targetNode;
  CqiManager *m_cqiManager;
  DRXManager *m_drxManager;
  ConsumptionModel *m_consumptionModel;
  BatteryModel *m_batteryModel;

  bool m_isIndoor;

  double m_timePositionUpdate;

  bool m_isRegistered;

  std::vector<CartesianCoordinates> get_all_users_coordinates;

  /*
   * Author: Tiago Pedroso da Cruz de Andrade
   * 0 - HTC device
   * 1 - MTC device
   */
  int m_deviceType;

  /*
   */
  int PF; // Paging Frame
  int PO; // Paging Occasion

  /**/
  int m_deviceAccessClass;

  /**/
  bool m_semiSchEnable;

  bool m_srAllocated;
  int m_srPeriodicity;
  int m_srProhibitTimer;

  /* Reference Signal Received Power [dBm] */
  double m_RSRP;
  double m_RS_SINR;

  EnergyModel *m_energyModel;

  double m_currentPowerheadroom;
  int m_lastAllocatedPRBs;

  /* Used to indicate D2D communication */
  bool m_d2dAble;
};

#endif
