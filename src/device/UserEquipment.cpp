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

#include "UserEquipment.h"
#include "NetworkNode.h"
#include "ENodeB.h"
#include "HeNodeB.h"
#include "Gateway.h"
#include "../phy/ue-lte-phy.h"
#include "CqiManager/cqi-manager.h"
#include "../core/eventScheduler/simulator.h"
#include "../componentManagers/NetworkManager.h"
#include "../protocolStack/rrc/ho/handover-entity.h"
#include "../protocolStack/rrc/ho/ho-manager.h"

#include "../protocolStack/mac/ue-mac-entity.h"
#include "../componentManagers/InformationManager.h"
#include "../energyConsumption/ue-consumption-model.h"
#include "../energyConsumption/ue-miletus-model.h"
#include "../energyConsumption/ue-3gpp-model.h"
#include "../energyConsumption/ue-lauridsen-model.h"
#include "../drx/drx-manager.h"
#include "../energyConsumption/baterry-model.h"
#include "../core/spectrum/bandwidth-manager.h"

UserEquipment::UserEquipment()
{
}

UserEquipment::UserEquipment(int idElement, double posx, double posy, Cell *cell, NetworkNode *target, bool handover, Mobility::MobilityModel mobilityModel)
{
  m_userEquipmentsOnCoverage = new UserEquipmentsOnCoverage;

  SetIDNetworkNode(idElement);
  SetNodeType(NetworkNode::TYPE_UE);
  SetCell(cell);

  m_deviceType = 0;
  m_targetNode = target;
  m_srAllocated = false;

  FindPagingOpportunitie();

  ProtocolStack *stack = new ProtocolStack(this);
  SetProtocolStack(stack);

  Classifier *classifier = new Classifier();
  classifier->SetDevice(this);
  SetClassifier(classifier);
  SetNodeState(STATE_IDLE);

  CartesianCoordinates *position = new CartesianCoordinates(posx, posy);
  get_all_users_coordinates.push_back(*position);

  // Setup Mobility Model
  Mobility *m;
  if (mobilityModel == Mobility::RANDOM_DIRECTION)
  {
    m = new RandomDirection();
  }
  else if (mobilityModel == Mobility::RANDOM_WALK)
  {
    m = new RandomWalk();
  }
  else if (mobilityModel == Mobility::RANDOM_WAYPOINT)
  {
    m = new RandomWaypoint();
  }
  else if (mobilityModel == Mobility::CONSTANT_POSITION)
  {
    m = new ConstantPosition();
  }
  else if (mobilityModel == Mobility::MANHATTAN)
  {
    m = new Manhattan();
  }
  else
  {
    std::cout << "ERROR: incorrect Mobility Model" << std::endl;
    m = new RandomDirection();
  }

  m->SetHandover(handover);
  m->SetAbsolutePosition(position);
  m->SetNodeID(idElement);
  SetMobilityModel(m);

  // SetEnergyModel(NULL);

  if (!ONLY_RANDOM_ACCESS && (UPLINK || DOWNLINK))
  {
    m_timePositionUpdate = 0.010;
    Simulator::Init()->Schedule(m_timePositionUpdate, &UserEquipment::UpdateUserPosition, this, Simulator::Init()->Now());
  }

  delete position;

  UeLtePhy *phy = new UeLtePhy();
  phy->SetDevice(this);
  phy->SetBandwidthManager(target->GetPhy()->GetBandwidthManager());
  SetPhy(phy);

  m_cqiManager = NULL;
  m_isIndoor = false;

  m_isRegistered = false;

  m_drxManager = NULL;
  m_consumptionModel = NULL;
  m_batteryModel = NULL;

  m_d2dAble = false;
}

UserEquipment::UserEquipment(int idElement, double posx, double posy, int speed, double speedDirection, Cell *cell, NetworkNode *target, bool handover, Mobility::MobilityModel mobilityModel)
{
  m_userEquipmentsOnCoverage = new UserEquipmentsOnCoverage;

  SetIDNetworkNode(idElement);
  SetNodeType(NetworkNode::TYPE_UE);
  // SetNodeType(NetworkNode::TYPE_ENODEB);
  SetCell(cell);

  m_deviceType = 0;
  m_targetNode = target;
  m_srAllocated = false;

  FindPagingOpportunitie();

  ProtocolStack *stack = new ProtocolStack(this);
  SetProtocolStack(stack);

  Classifier *classifier = new Classifier();
  classifier->SetDevice(this);
  SetClassifier(classifier);
  SetNodeState(STATE_IDLE);

  CartesianCoordinates *position = new CartesianCoordinates(posx, posy);
  get_all_users_coordinates.push_back(*position);

  Mobility *m;
  if (mobilityModel == Mobility::RANDOM_DIRECTION)
  {
    m = new RandomDirection();
  }
  else if (mobilityModel == Mobility::RANDOM_WALK)
  {
    m = new RandomWalk();
  }
  else if (mobilityModel == Mobility::RANDOM_WAYPOINT)
  {
    m = new RandomWaypoint();
  }
  else if (mobilityModel == Mobility::CONSTANT_POSITION)
  {
    m = new ConstantPosition();
  }
  else if (mobilityModel == Mobility::MANHATTAN)
  {
    m = new Manhattan();
  }
  else
  {
    std::cout << "ERROR: incorrect Mobility Model" << std::endl;
    m = new RandomDirection();
  }

  m->SetHandover(handover);
  m->SetAbsolutePosition(position);
  m->SetNodeID(idElement);
  m->SetSpeed(speed);
  m->SetSpeedDirection(speedDirection);
  SetMobilityModel(m);

  // SetEnergyModel(NULL);

  if (!ONLY_RANDOM_ACCESS && (UPLINK || DOWNLINK))
  {
    m_timePositionUpdate = 0.010;
    Simulator::Init()->Schedule(m_timePositionUpdate, &UserEquipment::UpdateUserPosition, this, Simulator::Init()->Now());
  }

  delete position;

  UeLtePhy *phy = new UeLtePhy();
  phy->SetDevice(this);
  phy->SetBandwidthManager(target->GetPhy()->GetBandwidthManager());
  SetPhy(phy);

  m_cqiManager = NULL;
  m_isIndoor = false;

  m_isRegistered = false;

  m_drxManager = NULL;
  m_consumptionModel = NULL;
  m_batteryModel = NULL;

  m_d2dAble = false;
}

UserEquipment::~UserEquipment()
{
  m_targetNode = NULL;

  delete m_cqiManager;
  delete m_drxManager;
  delete m_batteryModel;

  Destroy();
}

std::vector<CartesianCoordinates> UserEquipment::GetAllUsersCoordinates(void)
{
  return get_all_users_coordinates;
}

void UserEquipment::SetTargetNode(NetworkNode *n)
{
  m_targetNode = n;
  SetCell(n->GetCell());
}

NetworkNode *
UserEquipment::GetTargetNode(void)
{
  return m_targetNode;
}

void UserEquipment::UpdateUserPosition(double time)
{
  GetMobilityModel()->UpdatePosition(time);

  SetIndoorFlag(NetworkManager::Init()->CheckIndoorUsers(this));

  if (GetMobilityModel()->GetHandover() == true)
  {
    NetworkNode *targetNode = GetTargetNode();

    if (targetNode->GetProtocolStack()->GetRrcEntity()->GetHandoverEntity()->CheckHandoverNeed(this))
    {
      NetworkNode *newTagertNode = targetNode->GetProtocolStack()->GetRrcEntity()->GetHandoverEntity()->GetHoManager()->m_target;
      NetworkManager::Init()->HandoverProcedure(time, this, targetNode, newTagertNode);
    }
  }

  // schedule the new update after m_timePositionUpdate
  Simulator::Init()->Schedule(m_timePositionUpdate, &UserEquipment::UpdateUserPosition, this, Simulator::Init()->Now());
}

void UserEquipment::SetCqiManager(CqiManager *cm)
{
  m_cqiManager = cm;
}

CqiManager *
UserEquipment::GetCqiManager(void)
{
  return m_cqiManager;
}

void UserEquipment::SetIndoorFlag(bool flag)
{
  m_isIndoor = flag;
}

bool UserEquipment::IsIndoor(void)
{
  return m_isIndoor;
}

void UserEquipment::Print(void)
{
  std::cout << " UserEquipment object:"
               "\n\t m_idNetworkNode = "
            << GetIDNetworkNode() << "\n\t idCell = " << GetCell()->GetIdCell() << "\n\t idtargetNode = " << GetTargetNode()->GetIDNetworkNode() << "\n\t m_AbsolutePosition_X = " << GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX() << "\n\t m_AbsolutePosition_Y = " << GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY() << "\n\t m_speed = " << GetMobilityModel()->GetSpeed() << "\n\t m_speedDirection = " << GetMobilityModel()->GetSpeedDirection() << "\n\t m_d2dAble = " << GetD2DAble() << std::endl;
}

void UserEquipment::SetRegister(bool r)
{
  m_isRegistered = r;
}

bool UserEquipment::IsRegister()
{
  return m_isRegistered;
}

void UserEquipment::StartRandomAccessProcedure(double time)
{
  Simulator::Init()->Schedule(time, &UeMacEntity::StartRAProcedure, (UeMacEntity *)GetProtocolStack()->GetMacEntity());
}

void UserEquipment::SetDeviceType(int type)
{
  m_deviceType = type;
}

int UserEquipment::GetDeviceType()
{
  return m_deviceType;
}

void UserEquipment::SetDeviceAccessClass(int accessClass)
{
  m_deviceAccessClass = accessClass;
}

int UserEquipment::GetDeviceAccessClass()
{
  return m_deviceAccessClass;
}

void UserEquipment::SetSemiSchEnable(bool enable)
{
  m_semiSchEnable = enable;
}

bool UserEquipment::GetSemiSchEnable()
{
  return m_semiSchEnable;
}

int UserEquipment::GetPagingFrame()
{
  return PF;
}

int UserEquipment::GetPagingOccasion()
{
  return PO;
}

void UserEquipment::FindPagingOpportunitie()
{
  int T = InformationManager::Init()->length_pagingCycle; // 32, 64, 128, 192, 256, 320, 384
  int nB = 1 * T;                                         // 4T, 2T, T, T/2, T/4, T/8, T/16, T/32
  int ue_id = GetIDNetworkNode() % 1024;                  // is the IMSI mod 1024

  int N;
  if (T < nB)
  {
    N = T; // N = min(T, nB)
  }
  else
  {
    N = nB; // N = min(T, nB)
  }

  PF = (T / N) * (ue_id % N);

  int Ns;
  if (1 > (nB / T))
  {
    Ns = 1; // Ns = max(1, nB / T)
  }
  else
  {
    Ns = (nB / T); // Ns = max(1, nB / T)
  }

  int i_s = int(floor(ue_id / N)) % Ns;
  PO = PagingOccasion[int(log2(Ns))][i_s];
}

void UserEquipment::SchedulingRequestConfiguration(bool allocated, int srProhibitTimer)
{
  m_srAllocated = allocated;
  m_srProhibitTimer = srProhibitTimer;
}

bool UserEquipment::IsSchedulingRequestAllocated(void)
{
  return m_srAllocated;
}

void UserEquipment::SchedulingRequestStart(int SRstart, int SRperiodicity)
{
  m_srPeriodicity = SRperiodicity;
  Simulator::Init()->Schedule((double)SRstart / (double)1000, &UeMacEntity::TimeoutSchedulingRequestTimer, (UeMacEntity *)GetProtocolStack()->GetMacEntity());
}

int UserEquipment::GetSRperiodicity()
{
  return m_srPeriodicity;
}

int UserEquipment::GetSRProhibitTimer()
{
  return m_srProhibitTimer;
}

void UserEquipment::SetReferenceSignalReceivedPower(double power)
{
  m_RSRP = power;
}

double
UserEquipment::GetReferenceSignalReceivedPower()
{
  return m_RSRP;
}

void UserEquipment::SetReferenceSignalSinr(double sinr)
{
  m_RS_SINR = sinr;
}

double
UserEquipment::GetReferenceSignalSinr()
{
  return m_RS_SINR;
}

void UserEquipment::SetEnergyModel(EnergyModel *energyModel)
{
  m_energyModel = energyModel;
}

EnergyModel *
UserEquipment::GetEnergyModel(void)
{
  return m_energyModel;
}

/*DRX*
UserEquipment::GetDrx(void) {
  return m_drx;
}

void
UserEquipment::SetDrx(DRX *drxMechanism) {
  m_drx = drxMechanism;
}*/

ConsumptionModel *
UserEquipment::GetConsumptionModel(void)
{
  return m_consumptionModel;
}

void UserEquipment::SetConsumptionModel(std::string model)
{
  if (m_consumptionModel == NULL)
  {
    ConsumptionModel *cModel;
    if (model == "MILETUS")
    {
      cModel = new MiletusModel();
    }
    else if (model == "3GPP")
    {
      cModel = new SimpleModel();
    }
    else if (model == "LAURIDSEN")
    {
      cModel = new LauridsenModel();
    }
    cModel->SetDevice(this);
    m_consumptionModel = cModel;
  }
}

DRXManager *
UserEquipment::GetDRXManager(void)
{
  return m_drxManager;
}

void UserEquipment::SetDRXManager()
{
  // if (GetConsumptionModel() == NULL) {//Verify if the UE does not have a consumption model attached to it
  //   SetConsumptionModel();
  // }
  if (m_drxManager == NULL)
  { // Verify if the UE does not have a DRX manager attached to it
    DRXManager *drxManager = new DRXManager();
    drxManager->SetDevice(this);
    m_drxManager = drxManager;
  }
}

void UserEquipment::SetBatteryModel()
{
  if (m_batteryModel == NULL)
  {
    BatteryModel *batteryModel = new BatteryModel();
    batteryModel->SetDevice(this);
    m_batteryModel = batteryModel;
  }
}

BatteryModel *
UserEquipment::GetBatteryModel()
{
  return m_batteryModel;
}

bool UserEquipment::GetD2DAble(void)
{
  return m_d2dAble;
}

void UserEquipment::SetD2DAble(bool d2dAble)
{
  m_d2dAble = d2dAble;
}

UserEquipment::UserEquipmentsOnCoverage *
UserEquipment::GetUserEquipmentsOnCoverage(void)
{
  return m_userEquipmentsOnCoverage;
}

UserEquipment::UserEquipmentOnCoverage *
UserEquipment::GetUserEquipmentOnCoverage(int idUE)
{
  UserEquipmentsOnCoverage *records = GetUserEquipmentsOnCoverage();
  UserEquipmentOnCoverage *record;
  UserEquipmentsOnCoverage::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++)
  {
    record = *iter;
    if (record->GetUE()->GetIDNetworkNode() == idUE)
    {
      return record;
    }
  }

  return NULL;
}

UserEquipment::UserEquipmentOnCoverage::UserEquipmentOnCoverage()
{
  m_UE = NULL;
}

UserEquipment::UserEquipmentOnCoverage::~UserEquipmentOnCoverage()
{
  m_uplinkChannelEstimation.clear();
}

UserEquipment::UserEquipmentOnCoverage::UserEquipmentOnCoverage(UserEquipment *UE)
{
  m_UE = UE;

  int nbRbs = m_UE->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();

  m_uplinkChannelEstimation.clear();
  for (int i = 0; i < nbRbs; i++)
  {
    m_uplinkChannelEstimation.push_back(-500.0);
  }
}

void UserEquipment::UserEquipmentOnCoverage::SetUE(UserEquipment *UE)
{
  m_UE = UE;
}

UserEquipment *
UserEquipment::UserEquipmentOnCoverage::GetUE(void) const
{
  return m_UE;
}

void UserEquipment::UserEquipmentOnCoverage::SetUplinkChannelEstimation(std::vector<double> vet)
{
  m_uplinkChannelEstimation.clear();
  m_uplinkChannelEstimation = vet;
}

std::vector<double>
UserEquipment::UserEquipmentOnCoverage::GetUplinkChannelEstimation() const
{
  return m_uplinkChannelEstimation;
}

void UserEquipment::UpdateUplinkChannelEstimation(UserEquipment *UE, std::vector<double> vet)
{
  UserEquipmentOnCoverage *record = GetUserEquipmentOnCoverage(UE->GetIDNetworkNode());

  if (record == NULL)
  {
    record = new UserEquipmentOnCoverage(UE);
    GetUserEquipmentsOnCoverage()->push_back(record);
  }

  record->SetUplinkChannelEstimation(vet);
}