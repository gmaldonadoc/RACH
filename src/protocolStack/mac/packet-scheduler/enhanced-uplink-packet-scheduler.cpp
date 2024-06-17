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
 * TDPS: Escolhe os usuarios que deveriam ser alocados na proxima rodada.
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
  int nbOfRBs = GetMacEntity()->GetRealNbOfRBsForULScheduling();
  int availableRBs; // No of RB's not allocated
  int unallocatedUsers; // No of users who remain unallocated
  int selectedUser; // user to be selected for allocation
  int mcs;
  int selectedPRB; // PRB to be selected for allocation
  double bestMetric; // best metric to identify user/RB combination
  int left, right, first, last; // index of left and left PRB's to check
  bool Allocated[nbOfRBs];
  bool allocationMade;
  bool dynamic_mcs = true;
  UsersToSchedule *users = usersToSchedule;
  UserToSchedule *scheduledUser;

  double metricsFD[nbOfRBs][users->size()];
  int bestUserForRB[nbOfRBs];
  AMCModule* amc = GetMacEntity()->GetAmcModule();

  /*  */
  availableRBs = nbOfRBs - objScheduler->GetSemiSchRBs();
  unallocatedUsers = users->size();
  for (int i = 0; i < nbOfRBs; i++) {
    if (nbOfRBs < objScheduler->GetSemiSchRBs()) {
      Allocated[i] = true;
    } else {
      Allocated[i] = false;
    }
  }

  //create a matrix of flow metricsFD
  for (int i = 0; i < nbOfRBs; i++) {
    for (int j = 0; j < users->size(); j++) {
      if (users->at(j)->m_allowToSchedule) {
        metricsFD[i][j] = ComputeFDSchedulingMetric(users->at(j), i);
      }
    }
  }

  /*--Start RB Allocations Part----*/
  int nScheduledUEs = 0;
  int nMaxUEsToSchedule;

  if (objScheduler->GetUplinkTDScheduler() != NULL) {
    nMaxUEsToSchedule = ((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->GetMaxUEsToScheduleInUL();
  } else {
    nMaxUEsToSchedule = users->size();
  }

  while (availableRBs > 0 && unallocatedUsers > 0 && nScheduledUEs < nMaxUEsToSchedule) {
    // First step: find the best user-RB combo
    selectedPRB = -1;
    selectedUser = -1;
    bestMetric = (-1) * std::numeric_limits<double>::max();

    for (int i = 0; i < nbOfRBs; i++) {
      if (!Allocated[i]) {
        for (int j = 0; j < users->size(); j++) {
          if (users->at(j)->m_listOfAllocatedRBs.size() == 0 && users->at(j)->m_dataToTransmit > 0) {
            if (bestMetric < metricsFD[i][j]) {
              selectedPRB = i;
              selectedUser = j;
              bestMetric = metricsFD[i][j];
            }
          }
        }
      }
    }

    //for each rb, who has the best metric?
    for (int i = 0; i < nbOfRBs; i++) {
      int max_usr = -1;
      double max_metr_usr = (-1) * std::numeric_limits<double>::max();
      for (int j = 0; j < users->size(); j++) {
        if (users->at(j)->m_listOfAllocatedRBs.size() == 0 && users->at(j)->m_dataToTransmit > 0) {
          if (metricsFD[i][j] > max_metr_usr || max_usr < 0) {
            max_usr = j;
            max_metr_usr = metricsFD[i][j];
          }
        }
      }
      bestUserForRB[i] = max_usr;
    }

#ifdef SCHEDULER_DEBUG
    std::cout << "[DEBUG] Best user for RB\n";
    for (int i = 0; i < nbOfRBs; i++) {
      if (Allocated[i]) std::cout << " x ";
      else std::cout << " " << bestUserForRB[i] << " ";
    }
    std::cout << std::endl;
#endif

    // Now start allocating for the selected user at the selected PRB the required blocks using how many PRB's are needed for the user
    if (selectedUser != -1) {
      scheduledUser = users->at(selectedUser);
      scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);
      mcs = amc->GetImcsFromSinr(scheduledUser->m_channelCondition.at(selectedPRB));
      Allocated[selectedPRB] = true;
      int prbs = 1;
      left = selectedPRB - 1;
      right = selectedPRB + 1;
      availableRBs--;
      unallocatedUsers--;

      first = selectedPRB;
      last = selectedPRB;

      allocationMade = true;

      /* Determining the maximum number of PRBs according to power RSRP */
      double P0_PUSCH = ((ENodeB *) GetMacEntity()->GetDevice())->GetP0NominalPusch();
      double alpha_PL = ((ENodeB *) GetMacEntity()->GetDevice())->GetAlphaPL();
      double PL = ((ENodeB*) ((UserEquipment*) scheduledUser->m_userToSchedule)->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) scheduledUser->m_userToSchedule)->GetReferenceSignalReceivedPower();
      int prbsMaximum = pow(10, ((scheduledUser->m_userToSchedule->GetPhy()->GetTxPower() - P0_PUSCH - (alpha_PL * PL)) / 10.0));
      /* -------- */

      while ((scheduledUser->m_dataToTransmit * 8 > (amc->GetTBSizeToUplinkFromMCS(mcs, prbs))) && (availableRBs > 0)) {
        //allocationMade = false;
        if (left >= 0 && Allocated[left] && right < nbOfRBs && Allocated[right]) {
          break;
        }

        // There is right, it is free, i am the best option and it is better than left, (if exists) => Allocate right
        if (((right < nbOfRBs) && (!Allocated[right]) && (bestUserForRB[right] == selectedUser))
                && (((left >= 0) && ((metricsFD[right][selectedUser] >= metricsFD[left][selectedUser]) || (bestUserForRB[left] != selectedUser)))
                || (left < 0)
                || Allocated[left])) {
          //Allocate PRB at right to the user
          Allocated[right] = true;

          if (dynamic_mcs) {
            if (mcs > amc->GetImcsFromSinr(scheduledUser->m_channelCondition.at(right))) {
              mcs = amc->GetImcsFromSinr(scheduledUser->m_channelCondition.at(right));
            }
          }

          scheduledUser->m_listOfAllocatedRBs.push_back(right);
          prbs++;
          right++;
          allocationMade = true;
          availableRBs--;

          last = right - 1;

        } else if (((left >= 0) && (!Allocated[left]) && (bestUserForRB[left] == selectedUser))
                && (((right < nbOfRBs) && ((metricsFD[left][selectedUser] > metricsFD[right][selectedUser]) || (bestUserForRB[right] != selectedUser)))
                || (right >= nbOfRBs)
                || Allocated[right])) {
          //Allocate PRB at left to the user
          Allocated[left] = true;

          if (dynamic_mcs) {
            if (mcs > amc->GetImcsFromSinr(scheduledUser->m_channelCondition.at(left))) {
              mcs = amc->GetImcsFromSinr(scheduledUser->m_channelCondition.at(left));
            }
          }

          scheduledUser->m_listOfAllocatedRBs.push_back(left);
          prbs++;
          left--;
          allocationMade = true;
          availableRBs--;

          first = left + 1;

        } else {
          break;
        }
      }

#ifdef SCHEDULER_DEBUG
      std::cout << "[DEBUG] UE " << selectedUser << " ID = " << scheduledUser->m_userToSchedule->GetIDNetworkNode()
              << " scheduled in " << scheduledUser->m_listOfAllocatedRBs.size()
              << " PRBs and MCS = " << mcs
              << std::endl << "\tPRBs Used:";
      for (int i = 0; i < scheduledUser->m_listOfAllocatedRBs.size(); i++) {
        std::cout << " " << scheduledUser->m_listOfAllocatedRBs.at(i) << " ";
      }
      std::cout << std::endl
              << "\t Data to Transmit [bits] = " << scheduledUser->m_dataToTransmit * 8
              << "\n\t Data Transmitted [bits] = " << amc->GetTBSizeToUplinkFromMCS(mcs, scheduledUser->m_listOfAllocatedRBs.size())
              << "\n\t Maximum PRBs allowed = " << prbsMaximum
              << std::endl;

#endif

      scheduledUser->m_selectedMCS = mcs;

      if (allocationMade) {

        if (objScheduler->GetUplinkTDScheduler() == NULL) {
          if (((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->PDCCHResourceAllocation(scheduledUser->m_userToSchedule->GetIDNetworkNode()) == true) {
            nScheduledUEs++;
            scheduledUser->m_transmittedData = amc->GetTBSizeToUplinkFromMCS(scheduledUser->m_selectedMCS, scheduledUser->m_listOfAllocatedRBs.size()) / 8;
          } else {
            for (int j = first; j <= last; j++) {
              Allocated[j] = false;
            }
          }
        } else {
          nScheduledUEs++;
          scheduledUser->m_transmittedData = amc->GetTBSizeToUplinkFromMCS(scheduledUser->m_selectedMCS, scheduledUser->m_listOfAllocatedRBs.size()) / 8;
        }
      } else {
        scheduledUser->m_transmittedData = 0;
      }

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
