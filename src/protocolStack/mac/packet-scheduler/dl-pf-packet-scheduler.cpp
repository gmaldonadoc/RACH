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

#include <limits>

#include "dl-pf-packet-scheduler.h"
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
#include "../../../componentManagers/InformationManager.h"
#include "../../../utility/eesm-effective-sinr.h"

DL_PF_PacketScheduler::DL_PF_PacketScheduler() {
  SetMacEntity(0);
  CreateFlowsToSchedule();
}

DL_PF_PacketScheduler::~DL_PF_PacketScheduler() {
  Destroy();
}

double
DL_PF_PacketScheduler::ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel) {
  /*
   * For the PF scheduler the metric is computed
   * as follows:
   *
   * metric = spectralEfficiency / averageRate
   */
  double metric = (spectralEfficiency * 180000.) / bearer->GetAverageTransmissionRate();
  return metric;
}

double
DL_PF_PacketScheduler::ComputeTDSchedulingMetric(FlowToSchedule *flow) {
  std::vector<double> sinrs;
  for (std::vector<double>::iterator c = flow->m_spectralEfficiency.begin(); c != flow->m_spectralEfficiency.end(); c++) {
    double value = (*c);
    sinrs.push_back(value);
    // std::cout<<" + "<<value;
  }
  // std::cout<<"\n";

  double effectiveSinr = GetEesmEffectiveSinr(sinrs);
  double metric = (effectiveSinr * 180000.) / flow->m_bearer->GetAverageTransmissionRate();

  // if (effectiveSinr > 0.0f) {
  // std::cout<<"/effectiveSinr = "<<effectiveSinr<<" metric = "<<metric<<" rate = "<< flow->m_bearer->GetAverageTransmissionRate()<< "\n";
  // }

  return metric;
}

double
DL_PF_PacketScheduler::ComputeFDSchedulingMetric(FlowToSchedule *flow, int subChannel) {
  double channelCondition = flow->GetSpectralEfficiency().at(subChannel);

  //printf("Channel = %f ------ Average = %f\n", channelCondition, flow->GetBearer()->GetAverageTransmissionRate());

  double metric = (channelCondition * 180000.) / flow->GetBearer()->GetAverageTransmissionRate();

  return metric;
}

void
DL_PF_PacketScheduler::FDScheduling(FlowsToSchedule *flowsToSchedule, FlowsToSchedule *flowsHTCToSchedule, FlowsToSchedule *flowsMTCToSchedule) {
#ifdef SCHEDULER_DEBUG
  std::cout << "\t DownlinkPacketScheduler::RBsAllocation" << std::endl;
#endif

  FlowsToSchedule* flows = flowsToSchedule;
  int nbOfRBs = GetMacEntity()->GetDevice()->GetPhy()->GetBandwidthManager()->GetDlSubChannels().size();
  int availableRBs = nbOfRBs;

  //create a matrix of flow metrics
  double metrics[nbOfRBs][flows->size()];

  for (int i = 0; i < nbOfRBs; i++) {
    for (int j = 0; j < flows->size(); j++) {
      metrics[i][j] = ComputeFDSchedulingMetric(flows->at(j), i);
    }
  }

#ifdef SCHEDULER_DEBUG
  std::cout << "\t Available RBs = " << nbOfRBs << "\n\t # Flows = " << flows->size() << std::endl;
  for (int ii = 0; ii < nbOfRBs; ii++) {
    // std::cout << "\t metrics for flow "
    //         << flows->at(ii)->GetBearer()->GetApplication()->GetApplicationID() << ":";
    for (int jj = 0; jj < flows->size(); jj++) {
      std::cout << " " << metrics[jj][ii];
    }
  }
  std::cout << std::endl;
#endif

  AMCModule *amc = GetMacEntity()->GetAmcModule();

  int l_dAllocatedRBCounter = 0;

  int l_iNumberOfUsers = ((ENodeB*)this->GetMacEntity()->GetDevice())->GetNbOfUserEquipmentRecords();

  bool *l_bFlowScheduled = new bool[flows->size()];
  int l_iScheduledFlows = 0;
  std::vector<double> * l_bFlowScheduledSINR = new std::vector<double>[flows->size()];
  for (int k = 0; k < flows->size(); k++) {
    l_bFlowScheduled[k] = false;
  }

  //RBs allocation
  for (int s = 0; s < nbOfRBs; s++) {

    if (l_iScheduledFlows == flows->size()) {
      break;
    }

    double targetMetric = (-1) * std::numeric_limits<double>::max();
    bool RBIsAllocated = false;
    FlowToSchedule *scheduledFlow;
    int l_iScheduledFlowIndex = 0;

    for (int k = 0; k < flows->size(); k++) {
      if (metrics[s][k] > targetMetric && !l_bFlowScheduled[k] && flows->at(k)->m_allowToSchedule) {
        targetMetric = metrics[s][k];
        RBIsAllocated = true;
        scheduledFlow = flows->at(k);
        l_iScheduledFlowIndex = k;
      }
    }

    if (RBIsAllocated) {
      availableRBs--;
      l_dAllocatedRBCounter++;

      scheduledFlow->GetListOfAllocatedRBs()->push_back(s); // the s RB has been allocated to that flow!

#ifdef SCHEDULER_DEBUG
      std::cout << "\t RB " << s << " assigned to the flow " << scheduledFlow->GetBearer()->GetApplication()->GetApplicationID() << std::endl;
#endif

      double sinr = amc->GetSinrFromCQI(scheduledFlow->GetCqiFeedbacks().at(s));
      l_bFlowScheduledSINR[l_iScheduledFlowIndex].push_back(sinr);

      double effectiveSinr = GetEesmEffectiveSinr(l_bFlowScheduledSINR[l_iScheduledFlowIndex]);
      int mcs = amc->GetImcsFromSinr(effectiveSinr);
      int transportBlockSize = amc->GetTBSizeFromMCS(mcs, scheduledFlow->GetListOfAllocatedRBs()->size());

      //std::cout << " sinr = " << sinr << " mcs = " << mcs << " transportBlockSize = " << transportBlockSize << std::endl;

      if (transportBlockSize >= (scheduledFlow->GetDataToTransmit() * 8)) {
        l_bFlowScheduled[l_iScheduledFlowIndex] = true;
        l_iScheduledFlows++;
      }
    }
  }

  delete [] l_bFlowScheduled;
  delete [] l_bFlowScheduledSINR;

  //Finalize the allocation assembling the Downlink Assignments
  PdcchMapIdealControlMessage *pdcchMsg = new PdcchMapIdealControlMessage();

  for (FlowsToSchedule::iterator it = flows->begin(); it != flows->end(); it++) {
    FlowToSchedule *flow = (*it);
    if (flow->GetListOfAllocatedRBs()->size() > 0) {
      //this flow has been scheduled
      std::vector<double> estimatedSinrValues;
      for (int rb = 0; rb < flow->GetListOfAllocatedRBs()->size(); rb++) {
        double sinr = amc->GetSinrFromCQI(flow->GetCqiFeedbacks().at(flow->GetListOfAllocatedRBs()->at(rb)));
        estimatedSinrValues.push_back(sinr);
      }

      //compute the effective sinr
      double effectiveSinr = GetEesmEffectiveSinr(estimatedSinrValues);

      int mcs = amc->GetImcsFromSinr(effectiveSinr);

      flow->m_selectedMCS = mcs;

      std::cout << "DL Scheduler -- UE " << flow->m_bearer->GetDestination()->GetIDNetworkNode() << " SINR = " << effectiveSinr << " MCS-1 = " << mcs << std::endl;

      //define the amount of bytes to transmit
      int transportBlockSize = amc->GetTBSizeFromMCS(mcs, flow->GetListOfAllocatedRBs()->size());
      double bitsToTransmit = flow->GetDataToTransmit() * 8;
      if (transportBlockSize < bitsToTransmit) {
        bitsToTransmit = transportBlockSize;
      }
      flow->UpdateAllocatedBits(bitsToTransmit);

#ifdef SCHEDULER_DEBUG
      std::cout << "\t --> flow " << flow->GetBearer()->GetApplication()->GetApplicationID()
              << " has been scheduled" <<
              "\n\t\t nb of RBs = " << flow->GetListOfAllocatedRBs()->size() <<
              "\n\t\t effectiveSinr = " << effectiveSinr <<
              "\n\t\t tbs = " << transportBlockSize <<
              "\n\t\t bitsToTransmit = " << bitsToTransmit
              << std::endl;
#endif

      //create PDCCH messages
      for (int rb = 0; rb < flow->GetListOfAllocatedRBs()->size(); rb++) {
        pdcchMsg->AddNewRecord(PdcchMapIdealControlMessage::DOWNLINK, flow->GetListOfAllocatedRBs()->at(rb), flow->GetBearer()->GetDestination(), mcs);
      }
    }
  }

  if (pdcchMsg->GetMessage()->size() > 0) {
    GetMacEntity()->GetDevice()->GetPhy()->SendIdealControlMessage(pdcchMsg);
  }

  delete pdcchMsg;

  m_nAvailableRBs = availableRBs;
}

/*
 * TDPS: Escolhe os usuarios que deveriam ser alocados na proxima rodada.
 */
void
DL_PF_PacketScheduler::TDScheduling(FlowsToSchedule *flowsToSchedule) {
  int k, m;
  double aux;
  FlowsToSchedule *flows = flowsToSchedule;
  FlowToSchedule *auxScheduledFlow;

  int tam = flows->size();
  double metricsTD[tam];

  for (k = 0; k < tam; k++) {
    metricsTD[k] = ComputeTDSchedulingMetric(flows->at(k));
    flows->at(k)->m_TDMetric = metricsTD[k];
  }

  for (k = tam - 1; k > 0; k--) {
    for (m = 0; m < k; m++) {
      if (metricsTD[m] < metricsTD[m + 1]) {
        aux = metricsTD[m];
        auxScheduledFlow = flows->at(m);
        metricsTD[m] = metricsTD[m + 1];
        flows->at(m) = flows->at(m + 1);
        metricsTD[m + 1] = aux;
        flows->at(m + 1) = auxScheduledFlow;
      }
    }
  }
}

int
DL_PF_PacketScheduler::GetAvailablePRBs() {
  return m_nAvailableRBs;
}

void
DL_PF_PacketScheduler::RBsAllocation() {

}