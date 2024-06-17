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

#include "roundrobin-uplink-packet-scheduler.h"
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

#include "../../../componentManagers/InformationManager.h"
#include "../../../device/UserEquipment.h"
#include "../../../componentManagers/FrameManager.h"
#include "../pdcch-scheduler/pdcch-scheduler.h"


#include "../../../energyConsumption/ue-consumption-model.h"
//#include "interscheduler-info-hub.h"

#include <algorithm>

RoundRobinUplinkPacketScheduler::RoundRobinUplinkPacketScheduler() {
  SetMacEntity(0);
  CreateUsersToSchedule();
  m_roundRobinId = 0;
}

RoundRobinUplinkPacketScheduler::RoundRobinUplinkPacketScheduler(UplinkPacketScheduler *sch) {
  SetMacEntity(0);
  CreateUsersToSchedule();
  m_roundRobinId = 0;

  objScheduler = sch;
}

RoundRobinUplinkPacketScheduler::~RoundRobinUplinkPacketScheduler() {
  Destroy();
}

double
RoundRobinUplinkPacketScheduler::ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel) {
  return 0;
}

double
RoundRobinUplinkPacketScheduler::ComputeSchedulingMetric(UserToSchedule *user, int subchannel) {
  return 0;
}

void
RoundRobinUplinkPacketScheduler::RBsAllocation() {
}

double
RoundRobinUplinkPacketScheduler::ComputeTDSchedulingMetric(UserToSchedule* user) {
  return 0;
}

double
RoundRobinUplinkPacketScheduler::ComputeFDSchedulingMetric(UserToSchedule* user, int subchannel) {
  return 0;
}

void
RoundRobinUplinkPacketScheduler::TDScheduling(UsersToSchedule *usersToSchedule) {
#ifdef SCHEDULER_DEBUG
  std::cout << " ---- RR UL RBs Allocation";
#endif

  // Does nothing
}

void
RoundRobinUplinkPacketScheduler::FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule) {
#ifdef SCHEDULER_DEBUG
  std::cout << " ---- RR UL RBs Allocation";
#endif

  int availableRBs; // No of RB's not allocated
  int unallocatedUsers; // No of users who remain unallocated
  int nbOfRBs = GetMacEntity()->GetRealNbOfRBsForULScheduling();
  int mcs;
  int i;
  UsersToSchedule *users = usersToSchedule;
  UserToSchedule *scheduledUser;

  AMCModule* amc = GetMacEntity()->GetAmcModule();

  bool Allocated[nbOfRBs];

  std::random_shuffle(users->begin(), users->end());

  availableRBs = nbOfRBs - objScheduler->GetSemiSchRBs();
  unallocatedUsers = users->size();
  for (i = 0; i < nbOfRBs; i++) {
    if (i < objScheduler->GetSemiSchRBs()) { // FP: Changed: nbOfRBs for i @if
      Allocated[i] = true;
    } else {
      Allocated[i] = false;
    }
  }

  for (UsersToSchedule::iterator userit = users->begin(); userit != users->end(); userit++) {
    //((UserEquipment*) (*userit)->m_userToSchedule)->GetConsumptionModel()->m_FDScheduler_counter++;
  }

  /* Define how many RBs will be allocate for each UE */
  int nMaxUEsToSchedule;
  if (objScheduler->GetUplinkTDScheduler() != NULL) {
    nMaxUEsToSchedule = ((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->GetMaxUEsToScheduleInUL();
  } else {
    nMaxUEsToSchedule = users->size();
  }

  // TODO: REMOVE!! TEST ONLY
  //nMaxUEsToSchedule = 100;

  int requiredPRBs = 0;
  if (nMaxUEsToSchedule > 0 && users->size() > 0) {
    int min = nMaxUEsToSchedule;
    if (min > users->size()) min = users->size();
    requiredPRBs = availableRBs / min;
  }

  /*--Start RB Allocations Part----*/
  int first_free_prb = 0;
  int stop_nbOfRBs = nbOfRBs;

  if (objScheduler->GetUplinkTDScheduler() != NULL) {
    nMaxUEsToSchedule = ((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->GetMaxUEsToScheduleInUL();
  } else {
    nMaxUEsToSchedule = users->size();
  }

  first_free_prb = 0;

  int nUsersAllocated = 0;
  for (int usr = 0; usr < users->size(); usr++) {
    scheduledUser = users->at(usr);

    if (nUsersAllocated < nMaxUEsToSchedule) {
      nUsersAllocated++;

      scheduledUser->m_listOfAllocatedRBs.push_back(first_free_prb);
      Allocated[first_free_prb] = true;
      availableRBs--;
      mcs = amc->GetImcsFromSinr(scheduledUser->m_channelCondition.at(first_free_prb));

      for (i = 1; i < requiredPRBs; i++) {

        if (mcs > amc->GetImcsFromSinr(scheduledUser->m_channelCondition.at(i + first_free_prb))) {
          mcs = amc->GetImcsFromSinr(scheduledUser->m_channelCondition.at(first_free_prb));
        }

        scheduledUser->m_listOfAllocatedRBs.push_back(i + first_free_prb);
        Allocated[i + first_free_prb] = true;
        availableRBs--;
      }
      first_free_prb = i;
      scheduledUser->m_selectedMCS = mcs;
      scheduledUser->m_transmittedData = amc->GetTBSizeToUplinkFromMCS(scheduledUser->m_selectedMCS,
              scheduledUser->m_listOfAllocatedRBs.size()) / 8;


      // For Completeness transmission statistics
      if (scheduledUser->m_dataToTransmit * 8 <= amc->GetTBSizeToUplinkFromMCS(scheduledUser->m_selectedMCS, scheduledUser->m_listOfAllocatedRBs.size())) {
        //((UserEquipment*) scheduledUser->m_userToSchedule)->GetConsumptionModel()->m_complete_tx_counter++;
        //hub->m_complete_tx_counter++;
      } else {
        //((UserEquipment*) scheduledUser->m_userToSchedule)->GetConsumptionModel()->m_incomplete_tx_counter++;
        //hub->m_incomplete_tx_counter++;
      }

    } else {
      break;
    }
  }


  int max_ue = 0, all_ue = 0, prb_end = 0;

  if (nMaxUEsToSchedule == 0) all_ue = 1;
  else if (nMaxUEsToSchedule > users->size()) max_ue = 1;
  else prb_end = 1;

  m_nAvailableRBs = availableRBs;

#ifdef PRB_UTILIZATION_INFO
  std::cout << Simulator::Init()->Now()
          << " [PRBUTIL-INFO] " << m_nAvailableRBs
          << " left from " << nbOfRBs
          << " PRBEND: " << prb_end
          << " ALLUE: " << all_ue
          << " MAXUE: " << max_ue << std::endl;
#endif


}

int
RoundRobinUplinkPacketScheduler::GetAvailablePRBs() {
  return m_nAvailableRBs;
}
