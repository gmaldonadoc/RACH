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

  UsersToSchedule *users = GetUsersToSchedule();
  //int nbOfRBs = GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();
  int nbOfRBs = GetMacEntity()->GetRealNbOfRBsForULScheduling();

  //std::cerr << "Nb of RBs RR" << nbOfRBs << std::endl;

  //RBs allocation
  int nbPrbToAssign = 1;
  int stop_nbOfRBs = nbOfRBs;
  if ((nbOfRBs / users->size()) > nbPrbToAssign) {
    nbPrbToAssign = ceil(nbOfRBs / users->size());
    stop_nbOfRBs = nbPrbToAssign * users->size();
  }

#ifdef SCHEDULER_DEBUG
  std::cout << "  PRB to assign " << nbOfRBs << ", PRB for user " << nbPrbToAssign << std::endl;
#endif

  int s = 0;
  while (s < stop_nbOfRBs) {
    if (m_roundRobinId >= users->size()) m_roundRobinId = 0; //restart again from the beginning

    UserToSchedule *scheduledUser = users->at(m_roundRobinId);

    std::vector<double> sinrs;
    for (int i = 0; i < nbPrbToAssign; i++) {
      double chCondition = scheduledUser->m_channelCondition.at(s + i);
      //std::cerr << "Node " << scheduledUser->m_userToSchedule->GetIDNetworkNode() << " Condition " << chCondition << std::endl;
      double sinr = chCondition;
      sinrs.push_back(sinr);
      scheduledUser->m_listOfAllocatedRBs.push_back(s + i);
    }

    double effectiveSinr = GetEesmEffectiveSinr(sinrs);
    int mcs = GetMacEntity()->GetAmcModule()->GetMCSFromCQI(GetMacEntity()->GetAmcModule()->GetCQIFromSinr(effectiveSinr));
    int tbs = ((GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(mcs, nbPrbToAssign)) / 8);

    //std::cerr << "Node " << scheduledUser->m_userToSchedule->GetIDNetworkNode() << " SINR " << effectiveSinr << " MCS " << mcs << " TBS " << tbs << std::endl;

    scheduledUser->m_transmittedData = tbs;
    scheduledUser->m_selectedMCS = mcs;

    s = s + nbPrbToAssign;
    m_roundRobinId++;
  }
}

void
RoundRobinUplinkPacketScheduler::FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule) {
#ifdef SCHEDULER_DEBUG
  std::cout << " ---- RR UL RBs Allocation";
#endif

  int availableRBs; // No of RB's not allocated
  int unallocatedUsers; // No of users who remain unallocated
  int nbOfRBs = GetMacEntity()->GetRealNbOfRBsForULScheduling();

  bool Allocated[nbOfRBs];

  UsersToSchedule *users = usersToSchedule;
  UserToSchedule *scheduledUser;

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

  //int nbPrbToAssign = 1;
  int nbPrbToAssign = InformationManager::Init()->qtdPRBsTest;

  int stop_nbOfRBs = nbOfRBs;

  /*if ((nbOfRBs / users->size()) > nbPrbToAssign) {
    nbPrbToAssign = ceil(nbOfRBs / users->size());
    stop_nbOfRBs = nbPrbToAssign * users->size();
  }*/

#ifdef SCHEDULER_DEBUG
  std::cout << "  PRB to assign " << nbOfRBs << ", PRB for user " << nbPrbToAssign << std::endl;
#endif

  /*int s = 0;
  while (s < stop_nbOfRBs) {
    if (m_roundRobinId >= users->size()) m_roundRobinId = 0;

    UserToSchedule *scheduledUser = users->at(m_roundRobinId);

    std::vector<double> sinrs;
    for (int i = 0; i < nbPrbToAssign; i++) {
      double chCondition = scheduledUser->m_channelCondition.at(s + i);
      double sinr = chCondition;
      sinrs.push_back(sinr);
      scheduledUser->m_listOfAllocatedRBs.push_back(s + i);
    }

    double effectiveSinr = GetEesmEffectiveSinr(sinrs);
    int mcs = GetMacEntity()->GetAmcModule()->GetMCSFromCQI(GetMacEntity()->GetAmcModule()->GetCQIFromSinr(effectiveSinr));
    int tbs = ((GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(mcs, nbPrbToAssign)) / 8);

    scheduledUser->m_transmittedData = tbs;
    scheduledUser->m_selectedMCS = mcs;

    s = s + nbPrbToAssign;
    m_roundRobinId++;
  }*/

  if (users->size() > 0) {
    scheduledUser = users->at(0);

    if (nbPrbToAssign > stop_nbOfRBs) {
      nbPrbToAssign = stop_nbOfRBs;
    }

    //std::vector<double> sinrs;
    for (int i = 0; i < nbPrbToAssign; i++) {
      //double chCondition = scheduledUser->m_channelCondition.at(i);
      //double sinr = chCondition;
      //sinrs.push_back(sinr);
      scheduledUser->m_listOfAllocatedRBs.push_back(i);

      availableRBs--;
    }

    //double effectiveSinr = GetEesmEffectiveSinr(sinrs);
    double effectiveSinr = GetEesmEffectiveSinr(scheduledUser->m_channelCondition);
    
    int mcs = GetMacEntity()->GetAmcModule()->GetMCSFromCQI(GetMacEntity()->GetAmcModule()->GetCQIFromSinr(effectiveSinr));
    
    int tbs = ((GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(mcs, nbPrbToAssign)) / 8);

      /* Determining the required number of PRBs according to buffer size and MCS */
      int prbsRequired = 1;
      while (((scheduledUser->m_dataToTransmit * 8) > (GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(mcs, prbsRequired))) && (prbsRequired < 110)) {
        prbsRequired++;
      }
      /* -------- */

    scheduledUser->m_transmittedData = tbs;
    scheduledUser->m_selectedMCS = mcs;
  }

  m_nAvailableRBs = availableRBs;
}

int
RoundRobinUplinkPacketScheduler::GetAvailablePRBs() {
  return m_nAvailableRBs;
}