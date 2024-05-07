/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012
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
 * Author: Khaled Elsayed <khaled@ieee.org>
 */

#include <limits>

#include "enhanced-uplink-packet-scheduler.h"

#include "../mac-entity.h"
#include "../../packet/Packet.h"
#include "../../packet/packet-burst.h"
#include "../../../device/NetworkNode.h"
#include "../../../flows/radio-bearer.h"
#include "../../../protocolStack/rrc/rrc-entity.h"
#include "../../../flows/application/Application.h"
#include "../../../device/ENodeB.h"
#include "../../../protocolStack/mac/AMCModule.h"
#include "../../../phy/lte-phy.h"
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../core/idealMessages/ideal-control-messages.h"
#include "../../../flows/QoS/QoSParameters.h"
#include "../../../flows/MacQueue.h"
#include "../../../utility/eesm-effective-sinr.h"

#include "../../../utility/spectral-efficiency.h"
#include "../../../device/UserEquipment.h"
#include "../../../componentManagers/FrameManager.h"
#include "../pdcch-scheduler/pdcch-scheduler.h"

EnhancedUplinkPacketScheduler::EnhancedUplinkPacketScheduler() {
  SetMacEntity(0);
  CreateUsersToSchedule();
}

EnhancedUplinkPacketScheduler::EnhancedUplinkPacketScheduler(UplinkPacketScheduler *sch) {
  SetMacEntity(0);
  objScheduler = sch;
  CreateUsersToSchedule();
}

EnhancedUplinkPacketScheduler::~EnhancedUplinkPacketScheduler() {
  Destroy();
}

double
EnhancedUplinkPacketScheduler::ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel) {
  return 0;
}

double
EnhancedUplinkPacketScheduler::ComputeSchedulingMetric(UserToSchedule *user, int subchannel) {
  double channelCondition = user->m_channelCondition.at(subchannel);
  double spectralEfficiency = channelCondition;

  double metric = (spectralEfficiency * 180000) / user->m_averageUERate;

  return metric;
}

void
EnhancedUplinkPacketScheduler::RBsAllocation() {
}

double
EnhancedUplinkPacketScheduler::ComputeTDSchedulingMetric(UserToSchedule *user) {
  std::vector<double> sinrs;
  for (std::vector<double>::iterator c = user->m_channelCondition.begin(); c != user->m_channelCondition.end(); c++) {
    sinrs.push_back(*c);
  }

  double effectiveSinr = GetEesmEffectiveSinr(sinrs);
  double metric = (effectiveSinr * 180000) / user->m_averageUERate;

  return metric;
}

double
EnhancedUplinkPacketScheduler::ComputeFDSchedulingMetric(UserToSchedule *user, int subchannel) {
  double channelCondition = user->m_channelCondition.at(subchannel);

  double metric = (channelCondition * 180000) / user->m_averageUERate;

  return metric;
}

/*
 * TDPS: Escolhe os usuarios que podem ser alocados na proxima rodada
 */
void
EnhancedUplinkPacketScheduler::TDScheduling(UsersToSchedule *usersToSchedule) {
  UsersToSchedule *users = usersToSchedule;

  double metricsTD[users->size()];

  for (int u = 0; u < users->size(); u++) {
    metricsTD[u] = ComputeTDSchedulingMetric(users->at(u));
    users->at(u)->m_TDMetric = metricsTD[u];
  }

  int tam = users->size();
  int k, m;
  double aux;
  UserToSchedule *auxScheduledUser;

  for (k = tam - 1; k > 0; k--) {
    for (m = 0; m < k; m++) {
      if (metricsTD[m] < metricsTD[m + 1]) {
        aux = metricsTD[m];
        auxScheduledUser = users->at(m);
        metricsTD[m] = metricsTD[m + 1];
        users->at(m) = users->at(m + 1);
        metricsTD[m + 1] = aux;
        users->at(m + 1) = auxScheduledUser;
      }
    }
  }
}

void
EnhancedUplinkPacketScheduler::FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule) {
  /*----------------------------------Start FD Part----------------------------------*/
#ifdef UL_SCHEDULER_DEBUG
  std::cout << "----eNB Calculating FD Metric for UL Scheduling----" << std::endl;
#endif

  int nbOfRBs = GetMacEntity()->GetRealNbOfRBsForULScheduling();

  int availableRBs; // No of RB's not allocated
  int unallocatedUsers; // No of users who remain unallocated
  int selectedUser; // user to be selected for allocation
  int selectedPRB; // PRB to be selected for allocation
  double bestMetric; // best metric to identify user/RB combination
  int left, right, first, last; // index of left and left PRB's to check
  bool Allocated[nbOfRBs];
  bool allocationMade;

  UsersToSchedule *users = usersToSchedule;
  UserToSchedule *scheduledUser;

  double metricsFD[nbOfRBs][users->size()];
  int requiredPRBs[users->size()];

  //Some initialization
  availableRBs = nbOfRBs;
  unallocatedUsers = users->size();
  for (int i = 0; i < nbOfRBs; i++) {
    //Allocated[i] = false;
    Allocated[i] = GetPRBsAllocatedToRetransmission()[i];
  }

  //create a matrix of flow metricsFD
  for (int i = 0; i < nbOfRBs; i++) {
    for (int j = 0; j < users->size(); j++) {
      metricsFD[i][j] = ComputeFDSchedulingMetric(users->at(j), i);
    }
  }

  //create number of required PRB's per scheduled users
  for (int j = 0; j < users->size(); j++) {
    if (users->at(j)->m_allowToSchedule) {
      scheduledUser = users->at(j);

      double effectiveSinr = GetEesmEffectiveSinr(scheduledUser->m_channelCondition);

      //int mcs = GetMacEntity()->GetAmcModule()->GetMCSFromCQI(GetMacEntity()->GetAmcModule()->GetCQIFromSinr(effectiveSinr));
      int mcs = GetMacEntity()->GetAmcModule()->GetImcsFromSinr(effectiveSinr);
      //int mcs2 = GetMacEntity()->GetAmcModule()->GetMCSIndexFromEfficiency(GetSpectralEfficiency(effectiveSinr));

      std::cout << "UL Scheduler -- UE " << scheduledUser->m_userToSchedule->GetIDNetworkNode() << " SINR = " << effectiveSinr << " MCS-1 = " << mcs << " DATA = " << scheduledUser->m_dataToTransmit << std::endl;

      scheduledUser->m_selectedMCS = mcs;

      /* Determining the required number of PRBs according to buffer size and MCS */
      int prbsRequired = 1;
      while (((scheduledUser->m_dataToTransmit * 8) > (GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(mcs, prbsRequired))) && (prbsRequired < 110)) {
        prbsRequired++;
      }
      /* -------- */

      /* Determining the maximum number of PRBs according to power RSRP */
      double P0_PUSCH = ((ENodeB *) GetMacEntity()->GetDevice())->GetP0NominalPusch();
      double alpha_PL = ((ENodeB *) GetMacEntity()->GetDevice())->GetAlphaPL();
      double PL = ((ENodeB*) ((UserEquipment*) scheduledUser->m_userToSchedule)->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) scheduledUser->m_userToSchedule)->GetReferenceSignalReceivedPower();
      int prbsMaximum = pow(10, ((scheduledUser->m_userToSchedule->GetPhy()->GetTxPower() - P0_PUSCH - (alpha_PL * PL)) / 10.0));
      /* -------- */

      if (prbsRequired <= prbsMaximum) {
        requiredPRBs[j] = prbsRequired;
      } else {
        requiredPRBs[j] = prbsMaximum;
      }

      std::cout << "Uplink Scheduling UE " << users->at(j)->m_userToSchedule->GetIDNetworkNode() << " " << scheduledUser->m_userToSchedule->GetPhy()->GetTxPower() << " " << alpha_PL << " " << P0_PUSCH << " " << PL << " TTI " << FrameManager::Init()->GetTTICounter() << " prbsRq " << prbsRequired << " prbsAl " << prbsMaximum << " prbsSch " << requiredPRBs[j] << " Power Headroom " << users->at(j)->m_currentPowerHeadroom << std::endl;
    } else {
      requiredPRBs[j] = 0;
    }
  }
  /*----------------------END FD Part----------------------*/

#ifdef UL_SCHEDULER_DEBUG
  std::cout << "------------------END FD Part----------------------" << std::endl;
  std::cout << "----UL RBs Allocation----" << std::endl;
#endif

  /*--Start RB Allocations Part----*/
  //RBs allocation
  int nScheduledUEs = 0;
  int nMaxUEsToSchedule;

  if (objScheduler->GetUplinkTDScheduler() != NULL) {
    nMaxUEsToSchedule = ((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->GetMaxUEsToScheduleInUL();
    //std::cout << "MAX-1 = " << nMaxUEsToSchedule << std::endl;
  } else {
    nMaxUEsToSchedule = users->size();
    //std::cout << "MAX-2 = " << nMaxUEsToSchedule << std::endl;
  }

  while (availableRBs > 0 && unallocatedUsers > 0 && nScheduledUEs <= nMaxUEsToSchedule) {
    // First step: find the best user-RB combo
    selectedPRB = -1;
    selectedUser = -1;
    //bestMetric = (double) (-(1 << 35));
    bestMetric = (-1) * std::numeric_limits<double>::max();

    for (int i = 0; i < nbOfRBs; i++) {
      if (!Allocated[i]) {
        for (int j = 0; j < users->size(); j++) {
          if (users->at(j)->m_listOfAllocatedRBs.size() == 0 && requiredPRBs[j] > 0) {
            if (bestMetric < metricsFD[i][j]) {
              selectedPRB = i;
              selectedUser = j;
              bestMetric = metricsFD[i][j];
            }
          }
        }
      }
    }

    std::cout << "UE escolhido = " << selectedUser << std::endl;

    // Now start allocating for the selected user at the selected PRB the required blocks
    // using how many PRB's are needed for the user
    if (selectedUser != -1) {
      scheduledUser = users->at(selectedUser);
      scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
      Allocated[selectedPRB] = true;
      left = selectedPRB - 1;
      right = selectedPRB + 1;
      availableRBs--;
      unallocatedUsers--;

      first = selectedPRB;
      last = selectedPRB;

      allocationMade = true;
      for (int i = 1; i < requiredPRBs[selectedUser] && availableRBs > 0 && allocationMade; i++) {
        //allocationMade = false;
        if (left >= 0 && Allocated[left] && right < nbOfRBs && Allocated[right]) {
          break;
        }

        if ((right < nbOfRBs) && (!Allocated[right]) && (((left >= 0) && (metricsFD[right][selectedUser] >= metricsFD[left][selectedUser])) || (left < 0) || Allocated[left])) {
          //Allocate PRB at right to the user
          Allocated[right] = true;
          scheduledUser->m_listOfAllocatedRBs.push_back(right);
          right++;
          allocationMade = true;
          availableRBs--;

          last = right - 1;

        } else if ((left >= 0) && (!Allocated[left]) && (((right < nbOfRBs) && (metricsFD[left][selectedUser] > metricsFD[right][selectedUser])) || (right >= nbOfRBs) || Allocated[right])) {
          //Allocate PRB at left to the user
          Allocated[left] = true;
          scheduledUser->m_listOfAllocatedRBs.push_back(left);
          left--;
          allocationMade = true;
          availableRBs--;

          first = left + 1;
        }
      }

      if (allocationMade) {
        if (objScheduler->GetUplinkTDScheduler() == NULL) {
          if (((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->PDCCHResourceAllocation(scheduledUser->m_userToSchedule->GetIDNetworkNode()) == true) {
            //std::cerr << "UE Scheduled 1" << std::endl;
            nScheduledUEs++;
            scheduledUser->m_transmittedData = GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(scheduledUser->m_selectedMCS, scheduledUser->m_listOfAllocatedRBs.size()) / 8;
          } else {
            //std::cerr << "UE not Scheduled 1" << std::endl;
            for (int j = first; j <= last; j++) {
              Allocated[j] = false;
              availableRBs++;
            }
            scheduledUser->m_listOfAllocatedRBs.clear();
            scheduledUser->m_transmittedData = 0;
          }
        } else {
          nScheduledUEs++;
          scheduledUser->m_transmittedData = GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(scheduledUser->m_selectedMCS, scheduledUser->m_listOfAllocatedRBs.size()) / 8;
        }
      } else {
        scheduledUser->m_listOfAllocatedRBs.clear();
        scheduledUser->m_transmittedData = 0;
      }

      std::cerr << Simulator::Init()->Now() << " UE " << scheduledUser->m_userToSchedule->GetIDNetworkNode() << " " << scheduledUser->m_dataToTransmit << " " << scheduledUser->m_transmittedData << " " << scheduledUser->m_selectedMCS << " " << scheduledUser->m_listOfAllocatedRBs.size() << std::endl;

    } else {
      break;
    }
  }

  m_nAvailableRBs = availableRBs;
}

int
EnhancedUplinkPacketScheduler::GetAvailablePRBs() {
  return m_nAvailableRBs;
}