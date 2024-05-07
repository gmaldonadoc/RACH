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

#include "uplink-packet-scheduler.h"

#include <stddef.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <list>
#include <vector>

#include "../../../componentManagers/InformationManager.h"
#include "../../../core/eventScheduler/simulator.h"
#include "../../../core/idealMessages/ideal-control-messages.h"
#include "../../../flows/radio-bearer.h"
#include "../../../device/ENodeB.h"
#include "../../../device/UserEquipment.h"
#include "../../../phy/lte-phy.h"
#include "../../../utility/eesm-effective-sinr.h"
#include "../AMCModule.h"
#include "../enb-mac-entity.h"
#include "../pdcch-scheduler/pdcch-scheduler.h"
#include "../../../flows/QoS/QoSParameters.h"

#include "../../../componentManagers/FrameManager.h"
#include "../../../core/spectrum/bandwidth-manager.h"

#include "enhanced-uplink-packet-scheduler.h"
#include "roundrobin-uplink-packet-scheduler.h"
#include "mt-uplink-packet-scheduler.h"
#include "zbqos-uplink-packet-scheduler.h"
#include "ul-lioumpas-a1-packet-scheduler.h"
#include "ul-lioumpas-a2-packet-scheduler.h"

#include "../../../componentManagers/FrameManager.h"

#include "../../../load-parameters.h"
#include "../../../drx/drx-manager.h"
#include "../../packet/packet-burst.h"

UplinkPacketScheduler::UplinkPacketScheduler() {
  m_uplinkTDScheduler = NULL;
  m_uplinkFDScheduler = NULL;
}

UplinkPacketScheduler::~UplinkPacketScheduler() {
  Destroy();

  DeleteUsersToSchedule();

  delete m_uplinkTDScheduler;
  delete m_uplinkFDScheduler;
}

void
UplinkPacketScheduler::CreateUsersToSchedule(void) {
  m_usersToRetransmissionSchedule = new UsersToSchedule();

  if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
    m_usersHTCToSchedule = new UsersToSchedule();
    m_usersMTCToSchedule = new UsersToSchedule();
  } else {
    m_usersToSchedule = new UsersToSchedule();
  }
}

void
UplinkPacketScheduler::DeleteUsersToSchedule(void) {
  if (m_usersToRetransmissionSchedule != NULL) {
    if (m_usersToRetransmissionSchedule->size() > 0) {
      //ClearUsersHTCToSchedule();
    }
    delete m_usersToRetransmissionSchedule;
  }

  if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
    if (m_usersHTCToSchedule != NULL) {
      if (m_usersHTCToSchedule->size() > 0) {
        ClearUsersHTCToSchedule();
      }
      delete m_usersHTCToSchedule;
    }
    if (m_usersMTCToSchedule != NULL) {
      if (m_usersMTCToSchedule->size() > 0) {
        ClearUsersMTCToSchedule();
      }
      delete m_usersMTCToSchedule;
    }
  } else {
    if (m_usersToSchedule != NULL) {
      if (m_usersToSchedule->size() > 0) {
        ClearUsersToSchedule();
      }
      delete m_usersToSchedule;
    }
  }
}

void
UplinkPacketScheduler::ClearUsersToSchedule() {
  UsersToSchedule* records = GetUsersToSchedule();
  UsersToSchedule::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++) {
    delete *iter;
  }

  GetUsersToSchedule()->clear();
}

void
UplinkPacketScheduler::ClearUsersHTCToSchedule() {
  UsersToSchedule* records = GetUsersHTCToSchedule();
  UsersToSchedule::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++) {
    delete *iter;
  }

  GetUsersHTCToSchedule()->clear();
}

void
UplinkPacketScheduler::ClearUsersMTCToSchedule() {
  UsersToSchedule* records = GetUsersMTCToSchedule();
  UsersToSchedule::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++) {
    delete *iter;
  }

  GetUsersMTCToSchedule()->clear();
}

UplinkPacketScheduler::UsersToSchedule*
UplinkPacketScheduler::GetUsersToSchedule(void) {
  return m_usersToSchedule;
}

UplinkPacketScheduler::UsersToSchedule*
UplinkPacketScheduler::GetUsersHTCToSchedule(void) {
  return m_usersHTCToSchedule;
}

UplinkPacketScheduler::UsersToSchedule*
UplinkPacketScheduler::GetUsersMTCToSchedule(void) {
  return m_usersMTCToSchedule;
}

double
UplinkPacketScheduler::ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel) {
  return 0;
}

double
UplinkPacketScheduler::ComputeSchedulingMetric(UserToSchedule *user, int subchannel) {
  return 0;
}

void
UplinkPacketScheduler::SelectUsersToSchedule() {
  CreateUsersToSchedule();

  ENodeB *eNB = (ENodeB*) GetMacEntity()->GetDevice();
  ENodeB::UserEquipmentRecords *records = eNB->GetUserEquipmentRecords();
  ENodeB::UserEquipmentRecord *record;
  ENodeB::UserEquipmentRecords::iterator iter;

#ifdef SCHEDULER_DEBUG
  std::cerr << "[DEBUG] UplinkPacketScheduler::SelectUsersToSchedule() " << records->size() << std::endl;
#endif

  bool canSch = true;

  m_nSemiSchRBs = 0;

  for (int i = 0; i < 110; i++) {
    m_PRBsAllocatedToRetransmission[i] = false;
  }

  int TTI = round(Simulator::Init()->Now() * double(1000.0));
  
  for (iter = records->begin(); iter != records->end(); iter++) {
    record = *iter;

    //std::cout << "HARQ Scheduling in TTI " << int(Simulator::Init()->Now() * 1000) << " Process ID " << (int(Simulator::Init()->Now() * 1000) + 4) % 8 << std::endl;

    if (record->m_harqBuffer[(TTI + 4) % 8] != NULL) {
      canSch = false;

      UserToSchedule *user = new UserToSchedule();
      user->m_userToSchedule = (NetworkNode*) record->GetUE();
      m_usersToRetransmissionSchedule->push_back(user);

      //std::cout << "UE " << record->GetUE()->GetIDNetworkNode() << " allocated to retransmission" << std::endl;

      for (int it = 0; it != record->m_harqBuffer[(TTI + 4) % 8]->GetUsedPRBs().size(); it++) {
        //std::cout << "PRB " << record->m_harqBuffer[(int(Simulator::Init()->Now() * 1000) + 4) % 8]->GetUsedPRBs().at(it) << " allocated to retransmission of the UE " << record->GetUE()->GetIDNetworkNode() << std::endl;

        m_PRBsAllocatedToRetransmission[record->m_harqBuffer[(TTI + 4) % 8]->GetUsedPRBs().at(it)] = true;
      }
    }

    if (canSch == true) {
      if (record->GetSemiSchTime() > 0 && record->GetSemiSchTime() == FrameManager::Init()->GetTTICounter() - 1) {
        if (record->m_semiWait) {
          record->m_semiReliase++;
        }

        if (record->m_semiReliase == 2) {
          record->m_semiSchTime = 0;
          record->m_semiWait = 0;
        } else {
          record->m_semiWait = 1;
          m_nUEsSemiSch++;
          m_nSemiSchRBs += record->m_semiRBs;
          record->UpdateSemiSchTime();
        }
      }

      if (record->GetSchedulingRequest() > 0 && record->GetSemiSchTime() == 0) {
        double HoLPacketdelay = 0;
        RrcEntity *rrc = ((NetworkNode*) record->GetUE())->GetProtocolStack()->GetRrcEntity();

        /* ---[DRX]--- */
        DRXManager *drxMan = record->GetUE()->GetDRXManager();
        canSch = true;
        if (drxMan != NULL) {
          DRXManager::State drxState = drxMan->getStatus();
          if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
            if (drxMan->GetExpectedOffTime() > 0.001)
              canSch = false;
          }
        }
#ifdef DRX_DEBUG
        if (canSch == false) {
          std::cerr << Simulator::Init()->Now() << " [DRX] UE " << ((NetworkNode*) record->GetUE())->GetIDNetworkNode() << " excluded from Scheduling List" << std::endl;
        }
#endif
        /* --- */

        if (rrc->GetRadioBearerContainer()->at(0)->HasPackets() && canSch == true) {
          UserToSchedule *user = new UserToSchedule();
          user->m_userToSchedule = (NetworkNode*) record->GetUE();
          user->m_dataToTransmit = record->GetSchedulingRequest();
          user->m_listOfAllocatedRBs.clear();
          user->m_selectedMCS = 0;
          user->m_transmittedData = 0;

          if (record->GetD2DCommunicationUE()) {
            user->m_channelCondition = record->GetUE()->GetUserEquipmentOnCoverage(record->GetD2DCommunicationUE()->GetIDNetworkNode())->GetUplinkChannelEstimation();
          } else {
            user->m_channelCondition = record->GetUplinkChannelStatusIndicator();
          }

          user->m_averageSchedulingGrant = record->GetSchedulingGrants();
          user->m_averageUERate = record->GetUERate() / 1000;
          user->m_gbr = record->GetGBR();

          user->m_currentPowerHeadroom = record->GetPowerHeadroom();

          if (rrc->GetRadioBearerContainer()->size() > 0) {
            user->m_maximumDelay = rrc->GetRadioBearerContainer()->at(0)->GetQoSParameters()->GetMaxDelay();
            HoLPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetHeadOfLinePacketDelay();
          }

          user->real_HoL = HoLPacketdelay;

          if (eNB->GetPdcchScheduler()->GetLimitUEsToFD() == true) {
            if (GetUplinkTDScheduler() != NULL) {
              user->m_allowToSchedule = false;
            } else {
              user->m_allowToSchedule = true;
            }
          } else {
            user->m_allowToSchedule = true;
          }

          if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
            if (((UserEquipment*) (user->m_userToSchedule))->GetDeviceType() == 0) {
              GetUsersHTCToSchedule()->push_back(user);
            } else {
              GetUsersMTCToSchedule()->push_back(user);
            }
          } else {
            GetUsersToSchedule()->push_back(user);
          }
        } else {
          record->SetSchedulingRequest(0);
        }
      }
    }
  }
#ifdef SCHEDULER_DEBUG
  std::cout << "[DEBUG] Users to be schedule = " << GetUsersToSchedule()->size() << std::endl;
#endif
}

void
UplinkPacketScheduler::DoSchedule() {
#ifdef UL_SCHEDULER_DEBUG
  std::cout << "Start UPLINK packet scheduler for eNB with ID " << GetMacEntity()->GetDevice()->GetIDNetworkNode() << std::endl; //eNB or User??
#endif

  m_nUEsSemiSch = 0;

  UpdateAverageScheduladedRate(); //This function updates the average tx rate for all users in the system

  SelectUsersToSchedule(); //This Function Selects Schedulable Users

  ((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->StartSchedule();

  if (UPLINK == true) {

    if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
      std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-TO-SCHEDULE " << GetUsersHTCToSchedule()->size() + GetUsersMTCToSchedule()->size() << " USERS-SEMI-PERSISTENT " << m_nUEsSemiSch << std::endl;
    } else {
      std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-TO-SCHEDULE " << GetUsersToSchedule()->size() << " USERS-SEMI-PERSISTENT " << m_nUEsSemiSch << std::endl;
    }

    if (GetUplinkTDScheduler() != NULL) {
      if (GetUsersToSchedule()->size() > 0) {
        //std::cerr << "TDScheduling" << std::endl;
        GetUplinkTDScheduler()->TDScheduling(GetUsersToSchedule());
        limitUEsInFD();
      }
    }

    if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
      std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-TO-SCHEDULE-FD " << GetUsersHTCToSchedule()->size() + GetUsersMTCToSchedule()->size() << std::endl;
    } else {
      std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-TO-SCHEDULE-FD " << GetUsersToSchedule()->size() << std::endl;
    }
  }


  ((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->DoSchedule();

  if (UPLINK == true) {

    m_nAvailableRBs = GetMacEntity()->GetRealNbOfRBsForULScheduling();
    if (GetUplinkFDScheduler() != NULL) {

      if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
        if ((GetUsersHTCToSchedule()->size() > 0) || (GetUsersMTCToSchedule()->size() > 0) || (m_nUEsSemiSch > 0)) {
          GetUplinkFDScheduler()->FDScheduling(NULL, GetUsersHTCToSchedule(), GetUsersMTCToSchedule());
          m_nAvailableRBs = GetUplinkFDScheduler()->GetAvailablePRBs();
        }
      } else {
        if ((GetUsersToSchedule()->size() > 0) || (m_nUEsSemiSch > 0)) {
          GetUplinkFDScheduler()->FDScheduling(GetUsersToSchedule(), NULL, NULL);
          m_nAvailableRBs = GetUplinkFDScheduler()->GetAvailablePRBs();
        }
      }
    }

  }

  DoStopSchedule();

  ((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->StopSchedule();

  DeleteUsersToSchedule();
}

void
UplinkPacketScheduler::RBsAllocation() {
}

void
UplinkPacketScheduler::DoStopSchedule(void) {
  int nUsersScheduled = 0;
  int nRBsToRACH = 0;

  if (UPLINK == true) {
    int TTI = round(Simulator::Init()->Now() * double(1000.0));
    
    PdcchMapIdealControlMessage *pdcchMsg = new PdcchMapIdealControlMessage();
    UsersToSchedule *users = NULL;

    if (m_usersToRetransmissionSchedule->size() > 0) {
      users = m_usersToRetransmissionSchedule;

      for (UsersToSchedule::iterator it = users->begin(); it != users->end(); it++) {
        UserToSchedule *user = (*it);

        ENodeB *enb = (ENodeB*) GetMacEntity()->GetDevice();
        ENodeB::UserEquipmentRecord *record = enb->GetUserEquipmentRecord(user->m_userToSchedule->GetIDNetworkNode());

        for (int rb = 0; rb < record->m_harqBuffer[(TTI + 4) % 8]->GetUsedPRBs().size(); rb++) {
          //std::cerr << "PRB Allocated to retransmission == " << record->m_harqBuffer[(int(Simulator::Init()->Now() * 1000) + 4) % 8]->GetUsedPRBs().at(rb) << std::endl;
          pdcchMsg->AddNewRecord(PdcchMapIdealControlMessage::UPLINK, record->m_harqBuffer[(TTI + 4) % 8]->GetUsedPRBs().at(rb), user->m_userToSchedule, record->m_harqBuffer[(TTI + 4) % 8]->GetMcsUsed());
        }

        //std::cerr << Simulator::Init()->Now() << " UE " << record->GetUE()->GetIDNetworkNode() << " " << user->m_dataToTransmit << " " << record->m_schedulingRequest << std::endl;
      }
    }

    if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
      users = GetUsersHTCToSchedule();
      for (UsersToSchedule::iterator it = users->begin(); it != users->end(); it++) {
        UserToSchedule *user = (*it);
        if (user->m_transmittedData > 0) {
          nUsersScheduled++;

          for (int rb = 0; rb < user->m_listOfAllocatedRBs.size(); rb++) {
            //std::cerr << "Channel Allocated == " << user->m_listOfAllocatedRBs.at(rb) << std::endl;

            pdcchMsg->AddNewRecord(PdcchMapIdealControlMessage::UPLINK, user->m_listOfAllocatedRBs.at(rb), user->m_userToSchedule, user->m_selectedMCS);
          }

          ENodeB *enb = (ENodeB*) GetMacEntity()->GetDevice();
          ENodeB::UserEquipmentRecord *record = enb->GetUserEquipmentRecord(user->m_userToSchedule->GetIDNetworkNode());
          record->m_schedulingRequest -= user->m_transmittedData;
          record->m_scheduledData += user->m_transmittedData;

          if (record->m_schedulingRequest < 0) {
            record->m_schedulingRequest = 0;
          }

          record->UpdateSchedulingGrants(user->m_dataToTransmit);

          std::cerr << Simulator::Init()->Now() << " UE " << record->GetUE()->GetIDNetworkNode() << " " << user->m_dataToTransmit << " " << record->m_schedulingRequest << std::endl;
        }
      }

      users = GetUsersMTCToSchedule();
      for (UsersToSchedule::iterator it = users->begin(); it != users->end(); it++) {
        UserToSchedule *user = (*it);
        if (user->m_transmittedData > 0) {
          nUsersScheduled++;

          for (int rb = 0; rb < user->m_listOfAllocatedRBs.size(); rb++) {
            //std::cerr << "Channel Allocated == " << user->m_listOfAllocatedRBs.at(rb) << std::endl;

            pdcchMsg->AddNewRecord(PdcchMapIdealControlMessage::UPLINK, user->m_listOfAllocatedRBs.at(rb), user->m_userToSchedule, user->m_selectedMCS);
          }

          ENodeB *enb = (ENodeB*) GetMacEntity()->GetDevice();
          ENodeB::UserEquipmentRecord *record = enb->GetUserEquipmentRecord(user->m_userToSchedule->GetIDNetworkNode());
          record->m_schedulingRequest -= user->m_transmittedData;
          record->m_scheduledData += user->m_transmittedData;

          if (record->m_schedulingRequest < 0) {
            record->m_schedulingRequest = 0;
          }

          record->UpdateSchedulingGrants(user->m_dataToTransmit);

          std::cerr << Simulator::Init()->Now() << " UE " << record->GetUE()->GetIDNetworkNode() << " " << user->m_dataToTransmit << " " << record->m_schedulingRequest << " " << user->m_listOfAllocatedRBs.size() << std::endl;
        }
      }
    } else {
      users = GetUsersToSchedule();

      for (UsersToSchedule::iterator it = users->begin(); it != users->end(); it++) {
        UserToSchedule *user = (*it);
        if (user->m_transmittedData > 0) {
          nUsersScheduled++;

          for (int rb = 0; rb < user->m_listOfAllocatedRBs.size(); rb++) {
            //std::cerr << "PRB Allocated == " << user->m_listOfAllocatedRBs.at(rb) << std::endl;

            pdcchMsg->AddNewRecord(PdcchMapIdealControlMessage::UPLINK, user->m_listOfAllocatedRBs.at(rb), user->m_userToSchedule, user->m_selectedMCS);
          }

          ENodeB *enb = (ENodeB*) GetMacEntity()->GetDevice();
          ENodeB::UserEquipmentRecord *record = enb->GetUserEquipmentRecord(user->m_userToSchedule->GetIDNetworkNode());
          record->m_schedulingRequest -= (user->m_transmittedData - 1);
          record->m_scheduledData += (user->m_transmittedData - 1);

          if (record->m_schedulingRequest < 0) {
            record->m_schedulingRequest = 0;
          }

          record->UpdateSchedulingGrants((user->m_transmittedData - 1));

          std::cerr << Simulator::Init()->Now() << " UE " << record->GetUE()->GetIDNetworkNode() << " " << user->m_dataToTransmit << " " << user->m_transmittedData - 1 << " " << record->m_schedulingRequest << " " << user->m_listOfAllocatedRBs.size() << std::endl;
        }
      }
    }

    if (FrameManager::Init()->IsRACHSubframe() == true) {
      //std::cerr << Simulator::Init()->Now() << " is RACH!" << std::endl;

      nRBsToRACH = 6;
    }

    std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-SCHEDULED " << nUsersScheduled << " Total-PRBs " << GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size() - nRBsToRACH << " Used-PRBs " << GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size() - nRBsToRACH - m_nAvailableRBs << std::endl;

    ((EnbMacEntity*) GetMacEntity())->SendPdcchMapIdealControlMessage(pdcchMsg);
  }
}

void
UplinkPacketScheduler::SetnUsersFD(int usersFD) {
  m_nUEsFD = usersFD;
}

int
UplinkPacketScheduler::GetUsersFD() const {
  return m_nUEsFD;
}

/*
 * This function Update the Average Scheduled Rate for each user
 */
void
UplinkPacketScheduler::UpdateAverageScheduladedRate(void) {
  /*
   * Update Transmission Data Rate with
   * a
   * R'(t+1) = (0.9 * R'(t)) + (0.1 * r(t))
   */

  ENodeB *node = (ENodeB*) GetMacEntity()->GetDevice();
  ENodeB::UserEquipmentRecords *records = node->GetUserEquipmentRecords();
  ENodeB::UserEquipmentRecord *record;
  ENodeB::UserEquipmentRecords::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++) {
    record = *iter;
    record->UpdateAverageSchRate();
  }
}

double
UplinkPacketScheduler::ZShapeFunction(double max, double x, double a, double b) {
  /*
   * Implementation of the relaxed Z-Shaped Function
   * http://www.mathworks.com/help/fuzzy/zmf.html
   *
   */
  double valorMax = max;
  double y;
  double aux;

  if (x <= a) {
    y = 1;
  } else if (x > a && x <= ((a + b) / 2)) {
    aux = (x - a) / (b - a);
    y = 1 - (2 * pow(aux, 2));
  } else if (x >= ((a + b) / 2) && x < b) {
    aux = (x - b) / (b - a);
    y = 2 * pow(aux, 2);
  } else {
    y = 0.0;
  }
  return (double) (y * valorMax);
}

void
UplinkPacketScheduler::limitUEsInFD() {
  UsersToSchedule *users = GetUsersToSchedule();

  int limitUEs = GetUsersFD();

  int removedUsers = 0;

  while (users->size() > limitUEs) {
    users->pop_back();
    removedUsers++;
  }
}

void
UplinkPacketScheduler::limitUEs1(int maxUEs) {
  UsersToSchedule *users = GetUsersToSchedule();

  int removedUsers = 0;

  while (users->size() > maxUEs) {
    users->pop_back();
    removedUsers++;
  }
}

double
UplinkPacketScheduler::ComputeTDSchedulingMetric(UserToSchedule* user) {
  return 0;
}

double
UplinkPacketScheduler::ComputeFDSchedulingMetric(UserToSchedule* user, int subchannel) {
  return 0;
}

void
UplinkPacketScheduler::TDScheduling(UsersToSchedule *usersToSchedule) {
}

void
UplinkPacketScheduler::FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule) {
}

void
UplinkPacketScheduler::SetUplinkTDScheduler(ENodeB::ULSchedulerType type) {
  switch (type) {
    case ENodeB::ULScheduler_TYPE_NONE:
      m_uplinkTDScheduler = NULL;
      break;
    case ENodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT:
      m_uplinkTDScheduler = new MaximumThroughputUplinkPacketScheduler();
      break;
    case ENodeB::ULScheduler_TYPE_FME:
      m_uplinkTDScheduler = new EnhancedUplinkPacketScheduler();
      break;
    case ENodeB::ULScheduler_TYPE_ROUNDROBIN:
      m_uplinkTDScheduler = new RoundRobinUplinkPacketScheduler();
      break;
    case ENodeB::ULScheduler_TYPE_ZBQoS:
      m_uplinkTDScheduler = new ZBQoSUplinkPacketScheduler();
      break;
  }

  if (m_uplinkTDScheduler != NULL) {
    m_uplinkTDScheduler->SetMacEntity(GetMacEntity());
  }
}

void
UplinkPacketScheduler::SetUplinkFDScheduler(ENodeB::ULSchedulerType type) {
  switch (type) {
    case ENodeB::ULScheduler_TYPE_NONE:
      m_uplinkFDScheduler = NULL;
      break;
    case ENodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT:
      m_uplinkFDScheduler = new MaximumThroughputUplinkPacketScheduler(this);
      break;
    case ENodeB::ULScheduler_TYPE_FME:
      m_uplinkFDScheduler = new EnhancedUplinkPacketScheduler(this);
      break;
    case ENodeB::ULScheduler_TYPE_ROUNDROBIN:
      m_uplinkFDScheduler = new RoundRobinUplinkPacketScheduler(this);
      break;
    case ENodeB::ULScheduler_TYPE_ZBQoS:
      m_uplinkFDScheduler = new ZBQoSUplinkPacketScheduler(this);
      break;
    case ENodeB::ULScheduler_TYPE_LIOUMPAS_A1:
      m_uplinkFDScheduler = new LioumpasA1UplinkPacketScheduler(this);
      break;
    case ENodeB::ULScheduler_TYPE_LIOUMPAS_A2:
      m_uplinkFDScheduler = new LioumpasA2UplinkPacketScheduler(this);
      break;
  }

  if (m_uplinkFDScheduler != NULL) {
    m_ulFDSchedulerType = type;
    m_uplinkFDScheduler->SetMacEntity(GetMacEntity());
  }
}

UplinkPacketScheduler*
UplinkPacketScheduler::GetUplinkTDScheduler() {
  return m_uplinkTDScheduler;
}

UplinkPacketScheduler*
UplinkPacketScheduler::GetUplinkFDScheduler() {
  return m_uplinkFDScheduler;
}

int
UplinkPacketScheduler::GetAvailablePRBs() {

}

int
UplinkPacketScheduler::GetSemiSchRBs() {
  return m_nSemiSchRBs;
}

void UplinkPacketScheduler::DoSchedulingTD() {
#ifdef UL_SCHEDULER_DEBUG
  std::cout << "Start UPLINK packet scheduler for eNB with ID " << GetMacEntity()->GetDevice()->GetIDNetworkNode() << std::endl; //eNB or User??
#endif

  m_nUEsSemiSch = 0;

  UpdateAverageScheduladedRate(); //This function updates the average tx rate for all users in the system

  if (UPLINK == true) {

    if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
      std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-TO-SCHEDULE " << GetUsersHTCToSchedule()->size() + GetUsersMTCToSchedule()->size() << " USERS-SEMI-PERSISTENT " << m_nUEsSemiSch << std::endl;
    } else {
      std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-TO-SCHEDULE " << GetUsersToSchedule()->size() << " USERS-SEMI-PERSISTENT " << m_nUEsSemiSch << std::endl;
    }

    if (GetUplinkTDScheduler() != NULL) {
      if (GetUsersToSchedule()->size() > 0) {
        //std::cerr << "TDScheduling" << std::endl;
        GetUplinkTDScheduler()->TDScheduling(GetUsersToSchedule());
        limitUEsInFD();
      }
    }

    if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
      std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-TO-SCHEDULE-FD " << GetUsersHTCToSchedule()->size() + GetUsersMTCToSchedule()->size() << std::endl;
    } else {
      std::cerr << Simulator::Init()->Now() << " UL TTI " << FrameManager::Init()->GetTTICounter() << " USERS-TO-SCHEDULE-FD " << GetUsersToSchedule()->size() << std::endl;
    }
  }
}

void UplinkPacketScheduler::DoSchedulingFD() {

  if (UPLINK == true) {

    m_nAvailableRBs = GetMacEntity()->GetRealNbOfRBsForULScheduling();
    if (GetUplinkFDScheduler() != NULL) {

      if (m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A1 || m_ulFDSchedulerType == ENodeB::ULScheduler_TYPE_LIOUMPAS_A2) {
        if ((GetUsersHTCToSchedule()->size() > 0) || (GetUsersMTCToSchedule()->size() > 0) || (m_nUEsSemiSch > 0)) {
          GetUplinkFDScheduler()->FDScheduling(NULL, GetUsersHTCToSchedule(), GetUsersMTCToSchedule());
          m_nAvailableRBs = GetUplinkFDScheduler()->GetAvailablePRBs();
        }
      } else {
        if ((GetUsersToSchedule()->size() > 0) || (m_nUEsSemiSch > 0)) {
          GetUplinkFDScheduler()->FDScheduling(GetUsersToSchedule(), NULL, NULL);
          m_nAvailableRBs = GetUplinkFDScheduler()->GetAvailablePRBs();
        }
      }
    }
  }
}