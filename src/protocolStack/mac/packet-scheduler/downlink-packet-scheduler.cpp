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
 *         Lukasz Rajewski <lukasz.rajewski@gmail.com> (optimized PRB allocation)
 */

#include "downlink-packet-scheduler.h"

#include <stddef.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <list>
#include <vector>

#include "../mac-entity.h"
#include "../../packet/Packet.h"
#include "../../packet/packet-burst.h"
#include "../../../device/NetworkNode.h"
#include "../../../flows/radio-bearer.h"
#include "../../../protocolStack/rrc/rrc-entity.h"
#include "../../../flows/application/Application.h"
#include "../../../device/ENodeB.h"
#include "../../../protocolStack/mac/AMCModule.h"
#include "../pdcch-scheduler/pdcch-scheduler.h"
#include "../../../phy/lte-phy.h"
#include "../../../device/UserEquipment.h"
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../flows/QoS/QoSParameters.h"
#include "../../../flows/MacQueue.h"
#include "../../../utility/eesm-effective-sinr.h"

#include "dl-pf-packet-scheduler.h"

#include "../../../componentManagers/FrameManager.h"

#include "../../../load-parameters.h"
#include "../../../drx/drx-manager.h"

#include "../rrm/rrm-entity.h"

DownlinkPacketScheduler::DownlinkPacketScheduler() {
  m_downlinkTDScheduler = NULL;
  m_downlinkFDScheduler = NULL;
}

DownlinkPacketScheduler::~DownlinkPacketScheduler() {
  Destroy();

  //DeleteFlowsToSchedule();

  delete m_downlinkTDScheduler;
  delete m_downlinkFDScheduler;
}

void DownlinkPacketScheduler::SelectFlowsToSchedule() {
#ifdef SCHEDULER_DEBUG
  std::cout << "\t Select Flows to schedule" << std::endl;
#endif

  CreateFlowsToSchedule();

  ENodeB *enb = (ENodeB*) GetMacEntity()->GetDevice();
  RrcEntity *rrc = GetMacEntity()->GetDevice()->GetProtocolStack()->GetRrcEntity();
  RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer();

  for (std::vector<RadioBearer* >::iterator it = bearers->begin(); it != bearers->end(); it++) {
    //SELECT FLOWS TO SCHEDULE
    RadioBearer *bearer = (*it);

    if (bearer->HasPackets() && bearer->GetDestination()->GetNodeState() == NetworkNode::STATE_ACTIVE) {
      bool canSch = true;

      /* ---[DRX]--- */
      DRXManager *drxMan = ((UserEquipment*) bearer->GetDestination())->GetDRXManager();
      if (drxMan != NULL) {
        DRXManager::State drxState = drxMan->getStatus();
        if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::POWERDOWN_LIGHT || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::SYNC || drxState == DRXManager::MEAS) {
          std::cerr << Simulator::Init()->Now() << " [DRX] UE " << ((UserEquipment*) bearer->GetDestination())->GetIDNetworkNode() << " expected OFF time " << drxMan->GetExpectedOffTime() << std::endl;
          //if (drxMan->GetExpectedOffTime() > 0.001)
          canSch = false;
        }
      }

#ifdef DRX_DEBUG
      if (canSch == false) {
        std::cerr << Simulator::Init()->Now() << " [DRX] UE " << ((UserEquipment*) bearer->GetDestination())->GetIDNetworkNode() << " excluded from Scheduling List" << std::endl;
      }
#endif

      /* --- */

      if (canSch == false) {
        continue;
      }

      bool m_allowToSchedule = true;

      if (enb->GetPdcchScheduler()->GetLimitUEsToFD() == true) {
        if (GetDownlinkTDScheduler() != NULL) {
          m_allowToSchedule = false;
        } else {
          m_allowToSchedule = true;
        }
      } else {
        m_allowToSchedule = true;
      }

      int dataToTransmit;
      if (bearer->GetApplication()->GetApplicationType() == Application::APPLICATION_TYPE_INFINITE_BUFFER) {
        dataToTransmit = 100000000;
      } else {
        dataToTransmit = bearer->GetQueueSize();
      }

      //compute spectral efficiency
      ENodeB::UserEquipmentRecord *ueRecord = enb->GetUserEquipmentRecord(bearer->GetDestination()->GetIDNetworkNode());
      std::vector<double> spectralEfficiency;
      std::vector<int> cqiFeedbacks = ueRecord->GetCQI();
      int numberOfCqi = cqiFeedbacks.size();

      for (int i = 0; i < numberOfCqi; i++) {
        int t = cqiFeedbacks.at(i);
        double sEff = GetMacEntity()->GetAmcModule()->GetEfficiencyFromCQI(t);

        //std::cout << "Eff " << sEff << std::endl;

        spectralEfficiency.push_back(sEff);
      }

      //create flow to scheduler record
      InsertFlowToSchedule(bearer, dataToTransmit, spectralEfficiency, cqiFeedbacks, m_allowToSchedule);
    } else {

    }
  }
}

void
DownlinkPacketScheduler::DoSchedule(void) {

#ifdef SCHEDULER_DEBUG
  std::cout << "Start DL packet scheduler for node "
          << GetMacEntity()->GetDevice()->GetIDNetworkNode() << std::endl;
#endif

  UpdateAverageTransmissionRate();
  SelectFlowsToSchedule();

  if (GetFlowsToSchedule()->size() == 0) {
  } else {
    RBsAllocation();
  }

  StopSchedule();
}

void
DownlinkPacketScheduler::DoStopSchedule(void) {
  int nbFlowsScheduled = 0;

  std::list<PacketBurst*> pb3;

#ifdef SCHEDULER_DEBUG
  std::cout << "\t Creating Packet Burst" << std::endl;
#endif

  //PacketBurst *pb = new PacketBurst();

  FlowsToSchedule *flows = GetFlowsToSchedule();
  FlowToSchedule *flow;

  for (FlowsToSchedule::iterator it = flows->begin(); it != flows->end(); it++) {
    flow = (*it);

    //printf("Get Allocated Bits = %d\n", flow->GetAllocatedBits());

    int availableBytes = flow->GetAllocatedBits() / 8;

    if (availableBytes > 0) {

      flow->GetBearer()->UpdateTransmittedBytes(availableBytes);

#ifdef SCHEDULER_DEBUG
      std::cout << "\t  --> add packets for flow " << flow->GetBearer()->GetApplication()->GetApplicationID() << std::endl;
#endif

      RlcEntity *rlc = flow->GetBearer()->GetRlcEntity();
      PacketBurst *pb2 = rlc->TransmissionProcedure(availableBytes);

#ifdef SCHEDULER_DEBUG
      std::cout << "\t\t  nb of packets: " << pb2->GetNPackets() << std::endl;
#endif

      if (pb2->GetNPackets() > 0) {
        nbFlowsScheduled++;

        pb2->SetMcsUsed(flow->m_selectedMCS);
        pb3.push_back(pb2);

        //std::list<Packet*> packets = pb2->GetPackets();
        //std::list<Packet* >::iterator it;
        //for (it = packets.begin(); it != packets.end(); it++) {

#ifdef SCHEDULER_DEBUG
        //std::cout << "\t\t  added packet of bytes " << (*it)->GetSize() << std::endl;
#endif

        //Packet *p = (*it);
        //pb->AddPacket(p->Copy());
        //}
      }
      //delete pb2;
    } else {
    }
  }

  //UpdateAverageTransmissionRate ();

  //SEND PACKET BURST

#ifdef SCHEDULER_DEBUG
  //if (pb2->GetNPackets() == 0)
  //std::cout << "\t Send only reference symbols" << std::endl;
#endif

  std::cerr << Simulator::Init()->Now() << " DL TTI " << FrameManager::Init()->GetTTICounter() << " FLOWS-SCHEDULED " << nbFlowsScheduled << " Total-PRBs " << (int) GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetDlSubChannels().size() << " Used-PRBs " << GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetDlSubChannels().size() - m_nAvailableRBs << std::endl;

  //if (pb->GetSize() > 0) {
  //  std::cerr << Simulator::Init()->Now() << " ENB " << GetMacEntity()->GetDevice()->GetIDNetworkNode() << " sent Data with " << pb->GetSize() << " byte(s)" << std::endl;
  //  GetMacEntity()->GetDevice()->SendPacketBurst(pb);
  //}

  if (pb3.size() > 0) {
    int dataSize = 0;
    for (std::list<PacketBurst* >::const_iterator iter = pb3.begin(); iter != pb3.end(); iter++) {
      PacketBurst *pbIt = *iter;
      if (pbIt->GetSize() > 0) {
        dataSize += pbIt->GetSize();
        GetMacEntity()->GetDevice()->SendPacketBurst(pbIt);
      }
    }

    if (dataSize > 0) {
      std::cerr << Simulator::Init()->Now() << " ENB " << GetMacEntity()->GetDevice()->GetIDNetworkNode() << " sent Data with " << dataSize << " byte(s)" << std::endl;
    }
  }

  pb3.clear();
}

double
DownlinkPacketScheduler::ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel) {
  return 0;
}

void
DownlinkPacketScheduler::RBsAllocation() {
}

void
DownlinkPacketScheduler::limitUEsInFD() {
  FlowsToSchedule *flows = GetFlowsToSchedule();

  int limitUEs = GetFlowsFD();

  int removedFlows = 0;

  while (flows->size() > limitUEs) {
    flows->pop_back();
    removedFlows++;
  }
}

void
DownlinkPacketScheduler::ClearFlowsMTCToSchedule() {
  FlowsToSchedule* records = GetFlowsMTCToSchedule();

  for (FlowsToSchedule::iterator iter = records->begin(); iter != records->end(); iter++) {
    delete *iter;
  }

  GetFlowsMTCToSchedule()->clear();
}

/*
 * This function Update the Average Scheduled Rate for each user
 */
void
DownlinkPacketScheduler::UpdateAverageScheduladedRate(void) {
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

void
DownlinkPacketScheduler::FDScheduling(FlowsToSchedule *flowsToSchedule, FlowsToSchedule *flowsHTCToSchedule, FlowsToSchedule *flowsMTCToSchedule) {
}

void
DownlinkPacketScheduler::TDScheduling(FlowsToSchedule *flowsToSchedule) {
}

void
DownlinkPacketScheduler::UpdateAverageTransmissionRate(void) {
  RrcEntity *rrc = GetMacEntity()->GetDevice()->GetProtocolStack()->GetRrcEntity();
  RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer();

  for (std::vector<RadioBearer* >::iterator it = bearers->begin(); it != bearers->end(); it++) {
    RadioBearer *bearer = (*it);
    bearer->UpdateAverageTransmissionRate();
  }
}

DownlinkPacketScheduler::FlowsToSchedule*
DownlinkPacketScheduler::GetFlowsMTCToSchedule(void) {
  return m_flowsMTCToSchedule;
}

void
DownlinkPacketScheduler::SetnFlowsFD(int flowsFD) {
  m_nUEsFD = flowsFD;
}

int
DownlinkPacketScheduler::GetFlowsFD() const {
  return m_nUEsFD;
}

int
DownlinkPacketScheduler::GetAvailablePRBs() {

}

DownlinkPacketScheduler::FlowsToSchedule* DownlinkPacketScheduler::GetFlowsHTCToSchedule(void) {
  return m_flowsHTCToSchedule;
}

void
DownlinkPacketScheduler::SetDownlinkTDScheduler(ENodeB::DLSchedulerType type) {
  switch (type) {
    case ENodeB::DLScheduler_TYPE_NONE:
      m_downlinkTDScheduler = NULL;
      break;
    case ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR:
      m_downlinkTDScheduler = new DL_PF_PacketScheduler();
      break;
  }

  if (m_downlinkTDScheduler != NULL) {
    m_downlinkTDScheduler->SetMacEntity(GetMacEntity());
  }
}

void
DownlinkPacketScheduler::SetDownlinkFDScheduler(ENodeB::DLSchedulerType type) {
  switch (type) {
    case ENodeB::DLScheduler_TYPE_NONE:
      m_downlinkFDScheduler = NULL;
      break;
    case ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR:
      m_downlinkFDScheduler = new DL_PF_PacketScheduler();
      break;
  }

  if (m_downlinkTDScheduler != NULL) {
    m_dlFDSchedulerType = type;
    m_downlinkFDScheduler->SetMacEntity(GetMacEntity());
  }
}

DownlinkPacketScheduler*
DownlinkPacketScheduler::GetDownlinkTDScheduler() {
  return m_downlinkTDScheduler;
}

DownlinkPacketScheduler*
DownlinkPacketScheduler::GetDownlinkFDScheduler() {
  return m_downlinkFDScheduler;
}

void
DownlinkPacketScheduler::DoSchedulingTD() {
#ifdef UL_SCHEDULER_DEBUG
  std::cout << "Start DOWNLINK packet scheduler for eNB with ID " << GetMacEntity()->GetDevice()->GetIDNetworkNode() << std::endl; //eNB or User??
#endif

  m_nUEsSemiSch = 0;

  UpdateAverageTransmissionRate(); //This function updates the average tx rate for all Flows in the system

  if (DOWNLINK == true) {
    std::cerr << Simulator::Init()->Now() << " DL TTI " << FrameManager::Init()->GetTTICounter() << " FLOWS-TO-SCHEDULE " << GetFlowsToSchedule()->size() << " FLOWS-SEMI-PERSISTENT " << m_nUEsSemiSch << std::endl;

    if (GetFlowsToSchedule() != NULL) {
      if (GetFlowsToSchedule()->size() > 0) {
        std::cerr << "DL-TDScheduling" << std::endl;
        FlowsToSchedule *tempFlowsToSchedule = GetFlowsToSchedule();
        GetDownlinkTDScheduler()->TDScheduling(tempFlowsToSchedule);
        limitUEsInFD();
      }
    }

    std::cerr << Simulator::Init()->Now() << " DL TTI " << FrameManager::Init()->GetTTICounter() << " FLOWS-TO-SCHEDULE-FD " << GetFlowsToSchedule()->size() << std::endl;
  }
}

void
DownlinkPacketScheduler::DoSchedulingFD() {
  if (DOWNLINK == true) {

    m_nAvailableRBs = GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetDlSubChannels().size();

    if (GetDownlinkFDScheduler() != NULL) {
      if ((GetFlowsToSchedule()->size() > 0) || (m_nUEsSemiSch > 0)) {
        FlowsToSchedule *tempFlowsToSchedule = GetFlowsToSchedule();
        GetDownlinkFDScheduler()->FDScheduling(tempFlowsToSchedule, NULL, NULL);
        m_nAvailableRBs = GetDownlinkFDScheduler()->GetAvailablePRBs();
      }
    }
  }
}