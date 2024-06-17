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

#include "NetworkNode.h"
#include "UserEquipment.h"
#include "ENodeB.h"
#include "Gateway.h"
#include "../protocolStack/mac/packet-scheduler/packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/enhanced-uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/roundrobin-uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/mt-uplink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-pf-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-mlwdf-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-exp-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/dl-fls-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/exp-rule-downlink-packet-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/log-rule-downlink-packet-scheduler.h"

#include "../protocolStack/mac/pdcch-scheduler/pdcch-scheduler.h"
#include "../protocolStack/mac/packet-scheduler/zbqos-uplink-packet-scheduler.h"

#include "../phy/enb-lte-phy.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../protocolStack/packet/packet-burst.h"

#include "../protocolStack/mac/ue-mac-entity.h"
#include "../core/eventScheduler/simulator.h"

#include "../flows/QoS/QoSParameters.h"

#include "../utility/eesm-effective-sinr.h"
#include "../protocolStack/mac/AMCModule.h"
#include "../drx/drx-manager.h"
#include "../energyConsumption/ue-consumption-model.h"

#include "../protocolStack/mac/rrm/rrm-entity.h"

ENodeB::ENodeB()
{
}

ENodeB::ENodeB(int idElement, Cell *cell)
{
  SetIDNetworkNode(idElement);
  SetNodeType(NetworkNode::TYPE_ENODEB);
  SetCell(cell);

  CartesianCoordinates *position = new CartesianCoordinates(cell->GetCellCenterPosition()->GetCoordinateX(), cell->GetCellCenterPosition()->GetCoordinateY());

  Mobility *m = new ConstantPosition();
  m->SetAbsolutePosition(position);
  SetMobilityModel(m);

  delete position;

  m_userEquipmentRecords = new UserEquipmentRecords;

  EnbLtePhy *phy = new EnbLtePhy();
  phy->SetDevice(this);
  SetPhy(phy);

  ProtocolStack *stack = new ProtocolStack(this);
  SetProtocolStack(stack);

  Classifier *classifier = new Classifier();
  classifier->SetDevice(this);
  SetClassifier(classifier);

  SetReferenceSensitivityPowerLevel(-110);

  SetP0NominalPusch(-103);
  SetAlphaPL(1);

  SetPreambleInitialReceivedTargetPower(-110);
  SetPowerRampingStep(4);
}

ENodeB::ENodeB(int idElement, Cell *cell, double posx, double posy)
{
  SetIDNetworkNode(idElement);
  SetNodeType(NetworkNode::TYPE_ENODEB);
  SetCell(cell);

  CartesianCoordinates *position = new CartesianCoordinates(posx, posy);

  Mobility *m = new ConstantPosition();
  m->SetAbsolutePosition(position);
  SetMobilityModel(m);

  delete position;

  m_userEquipmentRecords = new UserEquipmentRecords;

  EnbLtePhy *phy = new EnbLtePhy();
  phy->SetDevice(this);
  SetPhy(phy);

  ProtocolStack *stack = new ProtocolStack(this);
  SetProtocolStack(stack);

  Classifier *classifier = new Classifier();
  classifier->SetDevice(this);
  SetClassifier(classifier);

  SetReferenceSensitivityPowerLevel(-110);

  SetP0NominalPusch(-103);
  SetAlphaPL(1);

  SetPreambleInitialReceivedTargetPower(-110);
  SetPowerRampingStep(4);
}

ENodeB::~ENodeB()
{
  Destroy();
  m_userEquipmentRecords->clear();
  delete m_userEquipmentRecords;
}

void ENodeB::RegisterUserEquipment(UserEquipment *UE)
{
  UserEquipmentRecord *record = new UserEquipmentRecord(UE);
  GetUserEquipmentRecords()->push_back(record);
  if (UE->GetD2DAble())
  {
    // std::cout << "@@@@ UE " << UE->GetIDNetworkNode() << " is D2D"<< std::endl;
    UE->GetPhy()->GetUlChannel()->AddDevice(UE);
  }
  else
  {
    // std::cout << "@@@@ UE " << UE->GetIDNetworkNode() << " is NOT D2D"<< std::endl;
  }

  UE->SetRegister(true);
}

void ENodeB::DeleteUserEquipment(UserEquipment *UE)
{
  UserEquipmentRecords *records = GetUserEquipmentRecords();
  UserEquipmentRecord *record;
  UserEquipmentRecords::iterator iter;

  UserEquipmentRecords *new_records = new UserEquipmentRecords();

  for (iter = records->begin(); iter != records->end(); iter++)
  {
    record = *iter;
    if (record->GetUE()->GetIDNetworkNode() != UE->GetIDNetworkNode())
    {
      // records->erase(iter);
      // break;
      new_records->push_back(record);
    }
    else
    {
      delete record;
    }
  }

  m_userEquipmentRecords->clear();
  delete m_userEquipmentRecords;
  m_userEquipmentRecords = new_records;

  UE->SetRegister(false);
}

int ENodeB::GetNbOfUserEquipmentRecords(void)
{
  return GetUserEquipmentRecords()->size();
}

void ENodeB::CreateUserEquipmentRecords(void)
{
  m_userEquipmentRecords = new UserEquipmentRecords();
}

void ENodeB::DeleteUserEquipmentRecords(void)
{
  m_userEquipmentRecords->clear();
  delete m_userEquipmentRecords;
}

ENodeB::UserEquipmentRecords *
ENodeB::GetUserEquipmentRecords(void)
{
  return m_userEquipmentRecords;
}

ENodeB::UserEquipmentRecord *
ENodeB::GetUserEquipmentRecord(int idUE)
{
  UserEquipmentRecords *records = GetUserEquipmentRecords();
  UserEquipmentRecord *record;
  UserEquipmentRecords::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++)
  {
    record = *iter;
    if (record->GetUE()->GetIDNetworkNode() == idUE)
    {
      return record;
    }
  }

  return nullptr;
}

ENodeB::UserEquipmentRecord::UserEquipmentRecord()
{
  m_UE = NULL;
  m_d2dCommunicationUE = NULL;

  // Create initial CQI values:
  m_cqiFeedback.clear();
  m_uplinkChannelStatusIndicator.clear();
  m_schedulingRequest = 0;
  m_averageSchedulingGrants = 1;

  m_averageUERate = 1.0;   // Initial value for the moving average rate in the FD
  m_averageUERateTD = 1.0; // Initial value for the moving average rate in the TD
  m_scheduledData = 0;     // Scheduled data in one TTI

  m_semiRBs = 0;
  m_semiMCS = 0;
  m_semiSchTime = 0;
  m_semiSchPeriod = 0;
  m_semiReliase = 0;
  m_semiWait = 0;

  m_currentPowerHeadroom = 0;

  for (int i = 0; i++; i < 8)
  {
    // m_harqBuffer.at(i) = NULL;
  }
}

ENodeB::UserEquipmentRecord::~UserEquipmentRecord()
{
  m_cqiFeedback.clear();
  m_uplinkChannelStatusIndicator.clear();
}

ENodeB::UserEquipmentRecord::UserEquipmentRecord(UserEquipment *UE)
{
  m_UE = UE;
  m_d2dCommunicationUE = NULL;

  int nbRbs;

  nbRbs = m_UE->GetPhy()->GetBandwidthManager()->GetDlSubChannels().size();
  m_cqiFeedback.clear();
  for (int i = 0; i < nbRbs; i++)
  {
    m_cqiFeedback.push_back(10);
  }

  nbRbs = m_UE->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();
  m_uplinkChannelStatusIndicator.clear();
  for (int i = 0; i < nbRbs; i++)
  {
    m_uplinkChannelStatusIndicator.push_back(10.);
  }

  m_schedulingRequest = 0;
  m_averageSchedulingGrants = 1;

  m_averageUERate = 1.0;
  m_averageUERateTD = 1.0;
  m_scheduledData = 0; // Amount of scheduled data in the last TTI

  m_semiRBs = 0;
  m_semiMCS = 0;
  m_semiSchTime = 0;
  m_semiSchPeriod = 0;
  m_semiReliase = 0;
  m_semiWait = 0;

  for (int i = 0; i < 8; i++)
  {
    m_harqBuffer[i] = NULL;
  }
}

void ENodeB::UserEquipmentRecord::SetUE(UserEquipment *UE)
{
  m_UE = UE;
}

UserEquipment *
ENodeB::UserEquipmentRecord::GetUE(void) const
{
  return m_UE;
}

void ENodeB::UserEquipmentRecord::SetCQI(std::vector<int> cqi)
{
  m_cqiFeedback = cqi;
}

std::vector<int> ENodeB::UserEquipmentRecord::GetCQI(void) const
{
  return m_cqiFeedback;
}

int ENodeB::UserEquipmentRecord::GetSchedulingRequest(void)
{
  return m_schedulingRequest;
}

void ENodeB::UserEquipmentRecord::SetSchedulingRequest(int r)
{
  m_schedulingRequest = r;
}

void ENodeB::UserEquipmentRecord::UpdateSchedulingGrants(int b)
{
  m_averageSchedulingGrants = (0.9 * m_averageSchedulingGrants) + (0.1 * b);
}

int ENodeB::UserEquipmentRecord::GetSchedulingGrants(void)
{
  return m_averageSchedulingGrants;
}

void ENodeB::UserEquipmentRecord::SetUlMcs(int mcs)
{
  m_ulMcs = mcs;
}

int ENodeB::UserEquipmentRecord::GetUlMcs(void)
{
  return m_ulMcs;
}

void ENodeB::UserEquipmentRecord::SetUplinkChannelStatusIndicator(std::vector<double> vet)
{
  m_uplinkChannelStatusIndicator = vet;

  std::cout << "UL_SRS_SINR " << GetEesmEffectiveSinr(m_uplinkChannelStatusIndicator) << std::endl;
}

std::vector<double> ENodeB::UserEquipmentRecord::GetUplinkChannelStatusIndicator() const
{
  return m_uplinkChannelStatusIndicator;
}

void ENodeB::UserEquipmentRecord::SetD2DCommunicationUE(UserEquipment *UE)
{
  m_d2dCommunicationUE = UE;
}

UserEquipment *
ENodeB::UserEquipmentRecord::GetD2DCommunicationUE(void)
{
  return m_d2dCommunicationUE;
}

void ENodeB::SetDLScheduler(ENodeB::DLSchedulerType type)
{
  EnbMacEntity *mac = (EnbMacEntity *)GetProtocolStack()->GetMacEntity();
  PacketScheduler *scheduler;
  switch (type)
  {
  case ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR:
    scheduler = new DL_PF_PacketScheduler();
    scheduler->SetMacEntity(mac);
    mac->SetDownlinkPacketScheduler(scheduler);
    break;

  case ENodeB::DLScheduler_TYPE_FLS:
    scheduler = new DL_FLS_PacketScheduler();
    scheduler->SetMacEntity(mac);
    mac->SetDownlinkPacketScheduler(scheduler);
    break;

  case ENodeB::DLScheduler_TYPE_EXP:
    scheduler = new DL_EXP_PacketScheduler();
    scheduler->SetMacEntity(mac);
    mac->SetDownlinkPacketScheduler(scheduler);
    break;

  case ENodeB::DLScheduler_TYPE_MLWDF:
    scheduler = new DL_MLWDF_PacketScheduler();
    scheduler->SetMacEntity(mac);
    mac->SetDownlinkPacketScheduler(scheduler);
    break;

  case ENodeB::DLScheduler_EXP_RULE:
    scheduler = new ExpRuleDownlinkPacketScheduler();
    scheduler->SetMacEntity(mac);
    mac->SetDownlinkPacketScheduler(scheduler);
    break;

  case ENodeB::DLScheduler_LOG_RULE:
    scheduler = new LogRuleDownlinkPacketScheduler();
    scheduler->SetMacEntity(mac);
    mac->SetDownlinkPacketScheduler(scheduler);
    break;

  default:
    std::cout << "ERROR: invalid scheduler type" << std::endl;
    scheduler = new DL_PF_PacketScheduler();
    scheduler->SetMacEntity(mac);
    mac->SetDownlinkPacketScheduler(scheduler);
    break;
  }
}

PacketScheduler *
ENodeB::GetDLScheduler(void) const
{
  EnbMacEntity *mac = (EnbMacEntity *)GetProtocolStack()->GetMacEntity();
  return mac->GetDownlinkPacketScheduler();
}

void ENodeB::SetULScheduler(ULSchedulerType type)
{
  EnbMacEntity *mac = (EnbMacEntity *)GetProtocolStack()->GetMacEntity();
  UplinkPacketScheduler *scheduler;

  scheduler = new UplinkPacketScheduler();
  scheduler->SetMacEntity(mac);
  mac->SetUplinkPacketScheduler(scheduler);
}

void ENodeB::SetULScheduler()
{
  EnbMacEntity *mac = (EnbMacEntity *)GetProtocolStack()->GetMacEntity();
  UplinkPacketScheduler *scheduler;

  scheduler = new UplinkPacketScheduler();
  scheduler->SetMacEntity(mac);
  mac->SetUplinkPacketScheduler(scheduler);
}

PacketScheduler *
ENodeB::GetULScheduler(void) const
{
  EnbMacEntity *mac = (EnbMacEntity *)GetProtocolStack()->GetMacEntity();
  return mac->GetUplinkPacketScheduler();
}

void ENodeB::SetPdcchScheduler()
{
  EnbMacEntity *mac = (EnbMacEntity *)GetProtocolStack()->GetMacEntity();
  PdcchScheduler *scheduler = new PdcchScheduler();
  scheduler->SetMacEntity(mac);
  mac->SetPdcchScheduler(scheduler);
}

PdcchScheduler *
ENodeB::GetPdcchScheduler(void) const
{
  EnbMacEntity *mac = (EnbMacEntity *)GetProtocolStack()->GetMacEntity();
  return mac->GetPdcchScheduler();
}

void ENodeB::ResourceBlocksAllocation(void)
{
  if (ONLY_RANDOM_ACCESS == false)
  {
    m_RrmEntity->DoSchedule();
  }
}

void ENodeB::UplinkResourceBlockAllocation(void)
{
  if (GetULScheduler() != NULL && GetNbOfUserEquipmentRecords() > 0)
  {
    GetULScheduler()->Schedule();
  }
}

void ENodeB::DownlinkResourceBlokAllocation(void)
{
  if (GetDLScheduler() != NULL && GetNbOfUserEquipmentRecords() > 0)
  {
    GetDLScheduler()->Schedule();
  }
}

void ENodeB::Print(void)
{
  std::cout << " ENodeB object:"
               "\n\t m_idNetworkNode = "
            << GetIDNetworkNode()
            << "\n\t m_idCell = " << GetCell()->GetIdCell()
            << "\n\t Served Users: " << std::endl;

  vector<UserEquipmentRecord *> *records = GetUserEquipmentRecords();
  UserEquipmentRecord *record;
  vector<UserEquipmentRecord *>::iterator iter;
  for (iter = records->begin(); iter != records->end(); iter++)
  {
    record = *iter;
    std::cout << "\t\t idUE = " << record->GetUE()->GetIDNetworkNode()
              << std::endl;
  }
}

void ENodeB::SetQoSParametersforUERecord(int idUE, QoSParameters *QoS)
{
  UserEquipmentRecord *record = GetUserEquipmentRecord(idUE);
  // Set the type of the bearer
  /*if (QoS->GetQCI() > 0 && QoS->GetQCI() <= 4){
    record->SetIsGBR(true);
  }
  else{
    record->IsGBR();
  }*/
  // record->SetTypeofBearer(QoS->typeOfBearer());
  // record->SetGBR(QoS->GetQCI()); //Inform the UERecord about his QCI. Useful for Compare2 Scheduler
  record->SetGBR(QoS->GetGBR());                          // Configura para cada usuario conectado a la eNB el GBR, GBR=0 No-GBR
  record->SetMaximumDelay(QoS->GetMaxDelay());            // Configura para cada usuario el Maximo Delay
  record->SetInitialUERate(QoS->GetInitialRate() * 1000); // en bps
}

void ENodeB::SetSemiSchConfigUERecord(int idUE, int startTime, int schPeriod)
{
  UserEquipmentRecord *record = GetUserEquipmentRecord(idUE);
  record->SetSemiSchConfig(startTime, schPeriod);

  if (schPeriod > 0)
  {
    // Simulator::Init()->Schedule(0.000, &ENodeB::SemiSchRBCalculation, this, record);
    SemiSchRBCalculation(record);
  }
}

void ENodeB::SemiSchRBCalculation(UserEquipmentRecord *record)
{
  double effectiveSinr = GetEesmEffectiveSinr(record->GetUplinkChannelStatusIndicator());
  int mcs = GetProtocolStack()->GetMacEntity()->GetAmcModule()->GetImcsFromSinr(effectiveSinr);

  /* Determining the required number of PRBs according to buffer size and MCS */
  int prbsRequired = 1;
  while ((360 > (GetProtocolStack()->GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(mcs, prbsRequired))) && (prbsRequired < 110))
  {
    prbsRequired++;
  }

  record->m_semiMCS = mcs;
  record->m_semiRBs = prbsRequired;

  // record->m_semiRBs = ceil((double) 45. / (double) (GetProtocolStack()->GetMacEntity()->GetAmcModule()->GetTBSizeFromMCS(mcs, 1) / 8));
}

void ENodeB::UserEquipmentRecord::UpdateAverageSchRate(void)
{
  /*
   * Update Transmission Data Rate with
   * a
   * R'(t+1) = ((1-beta) * R'(t)) + ( beta * r(t))
   */
  // This variable is update each TTI by the Scheduler
  int b = m_scheduledData; // [Bytes]
  double rate;             // Rate actual!

  //(Simulator::Init()->Now() - GetLastUpdate()) != 0 é porque o usuário foi escalonado nesse TTI
  if ((Simulator::Init()->Now() - GetLastUpdate()) != 0)
  {
    rate = (b * 8) / (Simulator::Init()->Now() - GetLastUpdate()); // [bps] +0.001 que é o que demora para ser transmitido
  }
  else
  { // Caso esse usuário nao foi escalonado nesse TTI
    rate = 0;
    // m_averageUERate = 0; //TODO:Borrar, solo para pruebas!
  }

  double betaTD = 0.05; // valor de 50 ms TODO imlementar isto dinamicamente!
  double betaFD = 0.05; // valor de 50 ms TODO imlementar isto dinamicamente!

  // http://en.wikipedia.org/wiki/Moving_average
  double beta = 0.05; // 1/50ms=0.05, Default: 0.01 // 1/20(TODO: seria melhor 1/1000 = 0.001??)
  double averageUERateOld = (double)m_averageUERate;

  m_averageUERate = (double)(((1 - betaFD) * averageUERateOld) + (betaFD * rate));
  // m_averageUERate = (double) (((1 - beta) * averageUERateOld) + (beta * rate)); //Old Vesion, Mod:19/09/2013

  // for avoiding division by 0
  /*if (m_averageUERate < 1) {
    m_averageUERate = 1; //bps
  }*/

  if (m_averageUERate == 0)
  {
    m_averageUERate = 0.001; // bps
  }

  double averageUERateOldTD = (double)m_averageUERateTD;
  m_averageUERateTD = (double)(((1 - betaTD) * averageUERateOldTD) + (betaTD * rate));
  // if (m_averageUERateTD < 1) {
  //   m_averageUERateTD = 1; //bps
  // }

  if (m_averageUERate == 0)
  {
    m_averageUERate = 0.001; // bps
  }

  /*TODO ativar este debug
  #ifdef UL_AVERATE_DEBUG
    std::cout << "UPDATE AVG RATE for User" << this->m_UE->GetIDNetworkNode() <<
        "\n\t TX Byte " << b <<
        "\n\t Current time " << Simulator::Init()->Now() <<
        "\n\t Last update time " << GetLastUpdate() <<
        "\n\t Interval " << Simulator::Init()->Now() - GetLastUpdate() <<
        "\n\t Instantaneus rate " << rate <<
        "\n\t Old TD rate [Kbps] " << averageUERateOldTD/1000 <<
        "\n\t New estimated TD rate [Kbps] " << m_averageUERateTD/1000 <<
        "\n\t Old rate [Kbps] " << averageUERateOld/1000 <<
            "\n\t New estimated rate [Kbps] " << m_averageUERate/1000 << std::endl;
  #endif
   */

  m_scheduledData = 0; // Reset Scheduled Data
  SetLastUpdate();
}

double
ENodeB::UserEquipmentRecord::GetGBR(void) const
{
  return m_GBR;
}

void ENodeB::UserEquipmentRecord::SetGBR(double gbr)
{
  m_GBR = gbr; // in kbps
}

double
ENodeB::UserEquipmentRecord::GetUERate()
{
  return m_averageUERate;
}

double
ENodeB::UserEquipmentRecord::GetUERateTD()
{
  return m_averageUERateTD;
}

double
ENodeB::UserEquipmentRecord::GetLastUpdate()
{
  return m_lastUpdate;
}

void ENodeB::UserEquipmentRecord::SetLastUpdate()
{
  m_lastUpdate = Simulator::Init()->Now();
}

void ENodeB::UserEquipmentRecord::SetInitialUERate(double inirate)
{
  m_averageUERate = inirate;
  m_averageUERateTD = inirate;
}

void ENodeB::UserEquipmentRecord::SetMaximumDelay(double maximumDelay)
{
  m_maximumDelay = maximumDelay;
}

double
ENodeB::UserEquipmentRecord::GetMaximumDelay(void)
{
  return m_maximumDelay;
}

void ENodeB::UserEquipmentRecord::SetSemiSchConfig(int startTime, int schPeriod)
{
  m_semiSchTime = startTime;
  m_semiSchPeriod = schPeriod;
}

double
ENodeB::UserEquipmentRecord::GetSemiSchTime()
{
  return m_semiSchTime;
}

void ENodeB::UserEquipmentRecord::UpdateSemiSchTime()
{
  m_semiSchTime += m_semiSchPeriod;
}

void ENodeB::UserEquipmentRecord::SetPowerHeadroom(double phr)
{
  m_currentPowerHeadroom = phr;
}

double
ENodeB::UserEquipmentRecord::GetPowerHeadroom(void)
{
  return m_currentPowerHeadroom;
}

void ENodeB::SchedulingRequestConfiguration(int nbSRsPerPUCCHRegion)
{
  UserEquipmentRecords *records = GetUserEquipmentRecords();
  UserEquipmentRecord *record;
  UserEquipmentRecords::iterator iter;

  UserEquipmentRecords *SR_records = new UserEquipmentRecords();

  for (iter = records->begin(); iter != records->end(); iter++)
  {
    record = *iter;
    if (record->GetUE()->IsSchedulingRequestAllocated() == true)
    {
      SR_records->push_back(record);
    }
  }

  SRperiodicity = ceil(SR_records->size() / (nbSRsPerPUCCHRegion * GetPhy()->GetBandwidthManager()->GetPUCCHTotalPrbs())); // in ms

  if (SRperiodicity <= 1)
  {
    SRperiodicity = 1;
  }
  else if (SRperiodicity <= 2)
  {
    SRperiodicity = 2;
  }
  else if (SRperiodicity <= 5)
  {
    SRperiodicity = 5;
  }
  else if (SRperiodicity <= 10)
  {
    SRperiodicity = 10;
  }
  else if (SRperiodicity <= 20)
  {
    SRperiodicity = 20;
  }
  else if (SRperiodicity <= 40)
  {
    SRperiodicity = 40;
  }
  else if (SRperiodicity <= 80)
  {
    SRperiodicity = 80;
  }

  SRperiodicity = 5;

  int t = 0;
  int i = 0;
  for (iter = SR_records->begin(); iter != SR_records->end(); iter++)
  {
    i++;
    if (i > (nbSRsPerPUCCHRegion * GetPhy()->GetBandwidthManager()->GetPUCCHTotalPrbs()))
    {
      t++;
      i = 1;
    }

    record = *iter;
    record->GetUE()->SchedulingRequestStart(t, SRperiodicity);
  }
}

void ENodeB::SetSinrFromReferenceSignal(int idUE, double CQI)
{
  ENodeB::UserEquipmentRecord *record = GetUserEquipmentRecord(idUE);
  record->m_CqiFromReferenceSignal = CQI;
}

void ENodeB::SetP0NominalPusch(double power)
{
  P0_Nominal_PUSCH = power;
}

double
ENodeB::GetP0NominalPusch()
{
  return P0_Nominal_PUSCH;
}

void ENodeB::SetP0NominalPucch(double power)
{
  P0_Nominal_PUCCH = power;
}

double
ENodeB::GetP0NominalPucch()
{
  return P0_Nominal_PUCCH;
}

void ENodeB::SetReferenceSensitivityPowerLevel(double power)
{
  m_refSen = power;
}

double
ENodeB::GetReferenceSensitivityPowerLevel()
{
  return m_refSen;
}

void ENodeB::SetAlphaPL(double alpha)
{
  alpha_PL = alpha;
}

double
ENodeB::GetAlphaPL()
{
  return alpha_PL;
}

void ENodeB::SetReferenceSignalPower(double power)
{
  referenceSignalPower = power;
}

double
ENodeB::GetReferenceSignalPower()
{
  return referenceSignalPower;
}

void ENodeB::SetPreambleInitialReceivedTargetPower(double power)
{
  preambleInitialReceivedTargetPower = power;
}

double
ENodeB::GetPreambleInitialReceivedTargetPower()
{
  return preambleInitialReceivedTargetPower;
}

void ENodeB::SetPowerRampingStep(double ramping)
{
  powerRampingStep = ramping;
}

double
ENodeB::GetPowerRampingStep()
{
  return powerRampingStep;
}

void ENodeB::PowerHeadroomReport(int idUE, double phr)
{
  UserEquipmentRecord *record = GetUserEquipmentRecord(idUE);
  record->SetPowerHeadroom(phr);
}

void ENodeB::InitDRXCycle()
{
  UserEquipmentRecords *records = m_userEquipmentRecords;
  UserEquipmentRecord *record;
  UserEquipmentRecords::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++)
  {
    record = *iter;
    DRXManager *drxManager = record->GetUE()->GetDRXManager();
    if (drxManager != NULL)
    { // If DRX is configured
      // drxManager->update();
      Simulator::Init()->Schedule(0.0, &DRXManager::update, drxManager);
      // If UE is in sleep mode, it does not call update(). Call it before the end of the simulation.
      // Simulator::Init()->Schedule(stopTime - 0.001, &DRXManager::update, drxManager);
    }
  }
}

void ENodeB::InitConsumptionEnergyCycle()
{
  UserEquipmentRecords *records = m_userEquipmentRecords;
  UserEquipmentRecord *record;
  UserEquipmentRecords::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++)
  {
    record = *iter;
    ConsumptionModel *consManager = record->GetUE()->GetConsumptionModel();
    if (consManager != NULL)
    {
      Simulator::Init()->Schedule(0.0009, &ConsumptionModel::updateEnergyConsumption, consManager, 1);
    }
  }
}

void ENodeB::SetRrmEntity()
{
  m_RrmEntity = new RrmEntity();
  m_RrmEntity->initialize(this);
}

RrmEntity *ENodeB::GetRrmEntity()
{
  return m_RrmEntity;
}