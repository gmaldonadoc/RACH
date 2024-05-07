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

#include "ue-lte-phy.h"
#include "enb-lte-phy.h"
#include "../device/NetworkNode.h"
#include "../channel/LteChannel.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../protocolStack/packet/packet-burst.h"
#include "../core/spectrum/transmitted-signal.h"
#include "../core/idealMessages/ideal-control-messages.h"
#include "../protocolStack/mac/AMCModule.h"
#include "../device/UserEquipment.h"
#include "../device/ENodeB.h"
#include "interference.h"
#include "error-model.h"
#include "../device/CqiManager/cqi-manager.h"
#include "../load-parameters.h"
#include "../core/eventScheduler/simulator.h"
#include "../protocolStack/mac/ue-mac-entity.h"
#include "../channel/propagation-model/propagation-loss-model.h"
#include "../utility/eesm-effective-sinr.h"
#include "../utility/ComputePathLoss.h"
#include "../componentManagers/InformationManager.h"


#include "../device/HeNodeB.h"
#include "../flows/application/WEB.h"

#include "../energyConsumption/ue-energy-model.h"
#include "../energyConsumption/ue-consumption-model.h"
#include "../drx/drx-manager.h"

/*
 * Noise is computed as follows:
 *  - noise_figure = 2.5 dBm
 *  - n0 = -174 dBm
 *  - subchannel_bandwidth = 180 kHz (in natural unit)
 *
 *  noise[dBm] = noise_figure[dBm] + n0[dBm] + (10 * log10(180000) + 30) = -88.95
 */

#define NOISE -148.95

UeLtePhy::UeLtePhy() {
  m_channelsForRx.clear();
  m_channelsForTx.clear();
  m_mcsIndexForRx.clear();
  m_mcsIndexForTx.clear();

  SetDevice(NULL);
  SetDlChannel(NULL);
  SetUlChannel(NULL);
  SetBandwidthManager(NULL);
  SetTxSignal(NULL);
  SetErrorModel(NULL);

  Interference *interference = new Interference();
  SetInterference(interference);

  SetMaxTxPower(23); // maximum transmission power in dBm

  m_use_efficient_ra = false;
  m_efficient_ra_method = 1;

  if (UPLINK == true) {
    Simulator::Init()->Schedule(0.000, &UeLtePhy::SetTxSignalForReferenceSymbols, this);
  }

  Simulator::Init()->Schedule(0.000, &UeLtePhy::ResetNoiseInterference, this);
}

UeLtePhy::~UeLtePhy() {
  Destroy();
}

void
UeLtePhy::DoSetBandwidthManager() {
  double powerPUSCHTx_dBm = -1000;
  double PL = -1000;

  BandwidthManager *s = GetBandwidthManager();
  std::vector<double> channels = s->GetUlSubChannels();

  TransmittedSignal *txSignal = new TransmittedSignal();

  /* Set the power of the PRBs to a lower value */
  std::vector<double> values;
  std::vector<double>::iterator it;
  for (it = channels.begin(); it != channels.end(); it++) {
    values.push_back(-1000);
  }
  /* --- */

  if (m_channelsForTx.size() > 0) {

    /* Power Control -- Open Loop */
    double P0_PUSCH = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetP0NominalPusch();
    double alpha_PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetAlphaPL();

    PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) GetDevice())->GetReferenceSignalReceivedPower();

    powerPUSCHTx_dBm = P0_PUSCH + (10 * log10(m_channelsForTx.size())) + (alpha_PL * PL);

    if (GetTxPower() <= powerPUSCHTx_dBm) {
      powerPUSCHTx_dBm = GetTxPower();
    }
    //powerPUSCHTx_dBm = GetTxPower();

    double powerTxPerPRB_dBm = powerPUSCHTx_dBm - (10 * log10(m_channelsForTx.size()));
    //double powerTxPerPRB_dBm = powerPUSCHTx_dBm - (10 * log10(channels.size()));
    /* --- */

    std::cout << "PRBs TX UE " << GetDevice()->GetIDNetworkNode() << " " << m_channelsForTx.size() << " " << powerPUSCHTx_dBm << " " << powerTxPerPRB_dBm << std::endl;

    //for (std::vector<int>::iterator it = m_channelsForTx.begin(); it != m_channelsForTx.end(); it++) {
    for (int it = 0; it < m_channelsForTx.size(); it++) {
      //int channel = (*it);
      int channel = m_channelsForTx.at(it);

      //std::cout << "PUSCH TX UE " << GetDevice()->GetIDNetworkNode() << " " << it << " " << channel << std::endl;

      //values.at(channel) = powerTx_dBm - (10 * log10(m_channelsForTx.size()));
      values.at(channel) = powerTxPerPRB_dBm;
    }

    //std::cout << "PUSCH TX UE " << GetDevice()->GetIDNetworkNode() << " PRBs " << m_channelsForTx.size() << " PL " << PL << " POWER-BAND " << powerPUSCHTx_dBm << " POWER-PRB " << powerTxPerPRB_dBm << std::endl;

    txSignal->SetValues(values);
    //SetTxSignal(txSignal);
    m_tt.push(txSignal);
  }
}

void
UeLtePhy::StartTx(PacketBurst *p) {
#ifdef TEST_DEVICE_ON_CHANNEL
  std::cout << "Node " << GetDevice()->GetIDNetworkNode() << " starts phy tx" << std::endl;
#endif

#ifdef ENERGY_CONSUMPTION_DEBUG
  std::cout << "[Counting Time TX] from " << GetDevice()->GetIDNetworkNode() << " at " << Simulator::Init()->Now() << std::endl;
#endif

  double powerPUSCHTx_dBm = -1000;
  double PL = -1000;

  int nPRBsUsed = 0;

  TransmittedSignal *t = m_tt.front();
  SetTxSignal(t);
  m_tt.pop();

  std::vector<double> rxSignalValues;
  std::vector<double>::iterator it;

  rxSignalValues = t->Getvalues();

  nPRBsUsed = p->GetUsedPRBs().size();

  /* Power Control - Open Loop */
  double P0_PUSCH = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetP0NominalPusch();
  double alpha_PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetAlphaPL();

  PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) GetDevice())->GetReferenceSignalReceivedPower();

  powerPUSCHTx_dBm = P0_PUSCH + (10 * log10(nPRBsUsed)) + (alpha_PL * PL);

  if (GetTxPower() <= powerPUSCHTx_dBm) {
    powerPUSCHTx_dBm = GetTxPower();
  }

  double powerTxPerPRB_dBm = powerPUSCHTx_dBm - (10 * log10(nPRBsUsed));

  std::cout << "PUSCH TX UE " << GetDevice()->GetIDNetworkNode() << " PRBs " << nPRBsUsed << " PL " << PL << " POWER " << powerPUSCHTx_dBm << " POWER PER PRB " << powerTxPerPRB_dBm << std::endl;

  ((UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity())->SetPowerHeadroom(GetTxPower() - powerPUSCHTx_dBm);
  ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->PowerHeadroomReport(GetDevice()->GetIDNetworkNode(), GetTxPower() - powerPUSCHTx_dBm);
  /* --- */

  if (p != NULL) {
    //std::cout << "Test UE send " << p->GetNPackets() << " " << GetTxSignal()->Getvalues().size() << " " << nPRBsUsed << std::endl;
  } else {
    //std::cout << "Test UE send 0 " << GetTxSignal()->Getvalues().size() << " " << nPRBsUsed << std::endl;
  }

  if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
    ((UserEquipment*) GetDevice())->GetConsumptionModel()->SetCurrentTxPower(powerPUSCHTx_dBm);
    ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_TX, DRXManager::PUSCH_TX);
  }
  if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
    ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateUL(DRXManager::PUSCH_TX);
  }

  GetUlChannel()->StartTx(p, GetTxSignal(), GetDevice());
}

void
UeLtePhy::StartRx(PacketBurst *p, TransmittedSignal *txSignal) {
#ifdef TEST_DEVICE_ON_CHANNEL
  std::cout << "Node " << GetDevice()->GetIDNetworkNode() << " starts phy rx" << std::endl;
#endif

  if (p != NULL) {

    std::vector<double> rxSignalValues;
    std::vector<double>::iterator it;

    rxSignalValues = txSignal->Getvalues();

    int chId = 0;
    double power_dBm;
    double power_W;

    for (it = rxSignalValues.begin(); it != rxSignalValues.end(); it++) {
      if ((*it) > -500.0) {
        power_dBm = (*it);

        power_W = pow(10.0, ((power_dBm - 30) / 10.0)); // in natural unit
        m_noiseInterference[chId] += power_W;
      }
      //std::cout << "Device = " << GetDevice()->GetIDNetworkNode() << "; PRB = " << chId << "; Power = " << power_W << "; NoiseInterference = " << m_noiseInterference[chId] << std::endl;
      chId++;
    }

    Simulator::Init()->Schedule(0.00025, &UeLtePhy::EndRx, this, p, txSignal);
  }
}

void
UeLtePhy::EndRx(PacketBurst *p, TransmittedSignal *txSignal) {
#ifdef TEST_DEVICE_ON_CHANNEL
  std::cout << "Node " << GetDevice()->GetIDNetworkNode() << " ends phy rx" << std::endl;
#endif

  if (p != NULL) {

    std::vector<double> measuredSinr;
    std::vector<int> channelsForRx;

    std::vector<double> rxSignalValues;
    std::vector<double>::iterator it;

    rxSignalValues = txSignal->Getvalues();

    int chId = 0;
    double power_dBm;
    double powerPDSCHRx_W = 0;

    for (it = rxSignalValues.begin(); it != rxSignalValues.end(); it++) {
      if ((*it) > -500.0) {
        power_dBm = (*it);
        channelsForRx.push_back(chId);

        double power_W = pow(10.0, ((power_dBm - 30) / 10.0)); // in natural unit
        double sinr_W = power_W / (m_noiseInterference[chId] - power_W); // in natural unit
        double sinr_dB = 10 * log10(sinr_W); // in dB unit

        measuredSinr.push_back(sinr_dB);

        powerPDSCHRx_W += power_W;
      }

      chId++;
    }

    bool phyError;

    if (GetErrorModel() != NULL && m_channelsForRx.size() > 0) {
      std::vector<int> cqi_;
      for (int i = 0; i < m_mcsIndexForRx.size(); i++) {
        AMCModule *amc = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
        int cqi = amc->GetCQIFromMCS(m_mcsIndexForRx.at(i));
        cqi_.push_back(cqi);
      }
      phyError = GetErrorModel()->CheckForPhysicalError(m_channelsForRx, cqi_, measuredSinr);

    } else {
      phyError = false;
    }

    if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->SetCurrentRxPower((10 * log10(powerPDSCHRx_W)) + 30);
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->SetCurrentRxRate(p->GetSize());
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_RX, DRXManager::PDSCH_RX);
    }
    if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
      ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateUL(DRXManager::PDSCH_RX);
    }

    if (!phyError && p->GetNPackets() > 0) {
      //std::cout << "Rec Data" << std::endl;
      Simulator::Init()->Schedule(0.00075, &NetworkNode::ReceivePacketBurst, GetDevice(), p->Copy());
    } else {
      //std::cout << "**** YES PHY ERROR (node " << GetDevice()->GetIDNetworkNode() << ") ****" << std::endl;
    }
  }

  m_channelsForRx.clear();
  m_channelsForTx.clear();
  m_mcsIndexForRx.clear();
  m_mcsIndexForTx.clear();

  delete txSignal;
  delete p;
}

void
UeLtePhy::CreateCqiFeedbacks(std::vector<double> sinr) {
  if (DOWNLINK == true) {
    UserEquipment *thisNode = (UserEquipment*) GetDevice();
    if (thisNode->GetCqiManager()->NeedToSendFeedbacks()) {
      thisNode->GetCqiManager()->CreateCqiFeedbacks(sinr);
    }
  }
}

void
UeLtePhy::SendIdealControlMessage(IdealControlMessage *msg) {
  TransmittedSignal *txSignal = new TransmittedSignal();

#ifdef TEST_CQI_FEEDBACKS
  std::cout << "SendIdealControlMessage (PHY) from  " << msg->GetSourceDevice()->GetIDNetworkNode() << " to " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
#endif

  if (msg->GetMessageType() == IdealControlMessage::RA_PREAMBLE) {

    /* Power Control - Open Loop */
    double preambleInitialReceivedTargetPower = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetPreambleInitialReceivedTargetPower();
    double powerRampingStep = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetPowerRampingStep();
    double PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) GetDevice())->GetReferenceSignalReceivedPower();

    double preambleReceivedTargetPower = preambleInitialReceivedTargetPower + ((((UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity())->GetPreambleTransmissionCounter() - 1) * powerRampingStep);
    double powerPRACHTx_dBm = preambleReceivedTargetPower + PL;

    if(m_use_efficient_ra == true){
      if(m_efficient_ra_method == 1){
        std::cout << Simulator::Init()->Now()
                  << " [EFRA] Method_1: Checking_PTX UE " << GetDevice()->GetIDNetworkNode()
                  << " POWER " << powerPRACHTx_dBm;
        if(powerPRACHTx_dBm < 11.4){
          std::cout << " >>> Enhancing_Power ";
          powerPRACHTx_dBm = 11.4;
        } else {
          std::cout << " >>> Keeping_Power ";
        }
      } else if(m_efficient_ra_method == 2){
        std::cout << Simulator::Init()->Now()
                  << " [EFRA] Method_2: Checking_PTX UE " << GetDevice()->GetIDNetworkNode()
                  << " POWER " << powerPRACHTx_dBm;
        if(powerPRACHTx_dBm <= 0.2){
          std::cout << " >>> Enhancing_Power " << powerPRACHTx_dBm << " --> 0.2" << "";
          powerPRACHTx_dBm = 0.2;
        } else if(powerPRACHTx_dBm < 11.4){
          std::cout << " >>> Enhancing_Power " << powerPRACHTx_dBm << " --> 11.4" << "";
          powerPRACHTx_dBm = 11.4;
        } else {
        std::cout << " >>> Keeping_Power ";
        }
      }
      std::cout << std::endl;
    }

    if (GetTxPower() < powerPRACHTx_dBm) {
      powerPRACHTx_dBm = GetTxPower();
    }
    /* --- */

    std::cout << "PRACH TX UE " << GetDevice()->GetIDNetworkNode() << " PL = " << PL << ", Preamble Counter = " << ((UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity())->GetPreambleTransmissionCounter() << ", POWER = " << powerPRACHTx_dBm << std::endl;

    if (((UserEquipment *) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment *) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_TX, DRXManager::PRACH_TX);
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->SetCurrentTxPower(powerPRACHTx_dBm);
    }
    if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
      ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateUL(DRXManager::PRACH_TX);
    }

    std::vector<double> values;
    values.push_back(powerPRACHTx_dBm);
    txSignal->SetValues(values);

    //std::cout << "PRACH TX UE " << GetDevice()->GetIDNetworkNode() << " DISTANCE " << msg->GetSourceDevice()->GetMobilityModel()->GetAbsolutePosition()->GetDistance(msg->GetDestinationDevice()->GetMobilityModel()->GetAbsolutePosition()) << " COUNTER " << ((UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity())->GetPreambleTransmissionCounter() << " POWER " << powerPRACHTx_dBm << std::endl;
  } else if (msg->GetMessageType() == IdealControlMessage::SCHEDULING_REQUEST || msg->GetMessageType() == IdealControlMessage::CONNECTION_REQUEST) {

    /* Power Control - Open Loop */
    double P0_PUSCH = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetP0NominalPusch();
    double alpha_PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetAlphaPL();
    double PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) GetDevice())->GetReferenceSignalReceivedPower();

    double powerPUSCHTx_dBm = P0_PUSCH + (10 * log10(1)) + (alpha_PL * PL);

    if (GetTxPower() <= powerPUSCHTx_dBm) {
      powerPUSCHTx_dBm = GetTxPower();
    }
    /* --- */

    double powerTxPerPRB_dBm = powerPUSCHTx_dBm - (10 * log10(1));

    if (InformationManager::Init()->harq_method != 1 ||
        !(((ConnectionRequestIdealControlMessage*) msg)->isFakeTransmission())) {

      if (((UserEquipment *) GetDevice())->GetConsumptionModel() != NULL) {
        ((UserEquipment*) GetDevice())->GetConsumptionModel()->SetCurrentTxPower(powerPUSCHTx_dBm);
        ((UserEquipment *) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_TX, DRXManager::PUSCH_TX);
      }
      if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
        ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateUL(DRXManager::PUSCH_TX);
      }
    }

    std::vector<double> values;
    values.push_back(powerTxPerPRB_dBm);
    txSignal->SetValues(values);

    //std::cout << "PUSCH TX UE " << GetDevice()->GetIDNetworkNode() << " PRBs " << "1" << " PL " << PL << " POWER " << powerPUSCHTx_dBm << std::endl;
  } else if (msg->GetMessageType() == IdealControlMessage::BUFFER_STATUS_REPORTING) {
    if (((BufferStatusReportingIdealControlMessage*) msg)->GetTriggerManager() == 1) {

      /* Power Control - Open Loop */
      double P0_PUSCH = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetP0NominalPusch();
      double alpha_PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetAlphaPL();
      double PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) GetDevice())->GetReferenceSignalReceivedPower();

      double powerPUSCHTx_dBm = P0_PUSCH + (10 * log10(1)) + (alpha_PL * PL);

      if (GetTxPower() <= powerPUSCHTx_dBm) {
        powerPUSCHTx_dBm = GetTxPower();
      }
      /* --- */

      double powerTxPerPRB_dBm = powerPUSCHTx_dBm - (10 * log10(1));

      std::vector<double> values;
      values.push_back(powerTxPerPRB_dBm);
      txSignal->SetValues(values);

      if (((UserEquipment *) GetDevice())->GetConsumptionModel() != NULL) {
        ((UserEquipment*) GetDevice())->GetConsumptionModel()->SetCurrentTxPower(powerPUSCHTx_dBm);
        ((UserEquipment *) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_TX, DRXManager::PUSCH_TX);
      }
      if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
        ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateUL(DRXManager::PUSCH_TX);
      }

      std::cout << "PUSCH TX UE " << GetDevice()->GetIDNetworkNode() << " PRBs " << "1" << " PL " << PL << " POWER-BAND " << powerPUSCHTx_dBm << " POWER-PRB " << powerTxPerPRB_dBm << std::endl;
    }
  } else if (msg->GetMessageType() == IdealControlMessage::SCHEDULING_REQUEST_PUCCH) {

    /* Power Control - Open Loop PUCCH*/
    double P0_PUCCH = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetP0NominalPucch();
    double alpha_PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetAlphaPL();
    double PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) GetDevice())->GetReferenceSignalReceivedPower();

    double powerPUCCHTx_dBm = P0_PUCCH + (10 * log10(1)) + (alpha_PL * PL);

    if (GetTxPower() <= powerPUCCHTx_dBm) {
      powerPUCCHTx_dBm = GetTxPower();
    }

    double powerTxPerPRB_dBm = powerPUCCHTx_dBm - (10 * log10(1));

    std::vector<double> values;
    values.push_back(powerTxPerPRB_dBm);
    txSignal->SetValues(values);

    if (((UserEquipment *) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->SetCurrentTxPower(powerPUCCHTx_dBm);
      ((UserEquipment *) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_TX, DRXManager::PUCCH_TX);
    }
    if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
      ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateUL(DRXManager::PUCCH_TX);
    }

    std::cout << "PUCCH TX UE " << GetDevice()->GetIDNetworkNode() << " PRBs " << "1" << " PL " << PL << " POWER " << powerPUCCHTx_dBm << std::endl;
  }

  msg->GetDestinationDevice()->GetPhy()->ReceiveIdealControlMessage(msg, GetDevice(), txSignal);
}

void
UeLtePhy::ReceiveIdealControlMessage(IdealControlMessage *msg, NetworkNode *src, TransmittedSignal *txSignal) {
  if (msg->GetMessageType() == IdealControlMessage::ALLOCATION_MAP) {
    m_channelsForRx.clear();
    m_channelsForTx.clear();
    m_mcsIndexForRx.clear();
    m_mcsIndexForTx.clear();

    PdcchMapIdealControlMessage *map = (PdcchMapIdealControlMessage*) msg;
    PdcchMapIdealControlMessage::IdealPdcchMessage *map2 = map->GetMessage();
    PdcchMapIdealControlMessage::IdealPdcchMessage::iterator it;

    /*if (Random::Init()->Uniform(0.0, 1.0) <= 0.99) {
      msg->setError(false);
    } else {
      msg->setError(true);
    }*/

    int nodeID = GetDevice()->GetIDNetworkNode();

    for (it = map2->begin(); it != map2->end(); it++) {
      if ((*it).m_ue->GetIDNetworkNode() == nodeID) {
        if ((*it).m_direction == PdcchMapIdealControlMessage::DOWNLINK) {
          m_channelsForRx.push_back((*it).m_idSubChannel);
          m_mcsIndexForRx.push_back((*it).m_mcsIndex);
        } else if ((*it).m_direction == PdcchMapIdealControlMessage::UPLINK) {
          m_channelsForTx.push_back((*it).m_idSubChannel);
          m_mcsIndexForTx.push_back((*it).m_mcsIndex);
        }
      }
    }

    if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_NO_DATA, DRXManager::PDCCH_RX);
    }

    if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
      ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateDL(DRXManager::PDCCH_RX);
    }

    if (m_channelsForTx.size() > 0) {
      DoSetBandwidthManager();

      UeMacEntity* mac = (UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
      //mac->ScheduleUplinkTransmission(m_channelsForTx.size(), m_mcsIndexForTx.at(0));
      //Simulator::Init()->Schedule(0.001, &UeMacEntity::ScheduleUplinkTransmission, mac, m_channelsForTx.size(), m_mcsIndexForTx.at(0));
      Simulator::Init()->Schedule(0.001, &UeMacEntity::ReceiveUplinkAllocationMap, mac, m_channelsForTx, m_mcsIndexForTx.at(0), msg->getMsgID());
    }

  } else if (msg->GetMessageType() == IdealControlMessage::RA_RESPONSE) {
    //std::cerr << Simulator::Init()->Now() << " UE " << GetDevice()->GetIDNetworkNode() << " received RA_Response from ENodeB " << ((ENodeB*) msg->GetSourceDevice())->GetIDNetworkNode() << std::endl;

    RAResponseIdealControlMessage *RAResponse = (RAResponseIdealControlMessage *) msg;
    //if (Random::Init()->Uniform(0.0, 1.0) <= 0.90) {
    RAResponse->setError(false);
    //} else {
    //  RAResponse->setError(true);
    //}fdfd      ff
    //((UserEquipment *) GetDevice())->GetEnergyModel()->UpdateEnergyConsumptionOnPdsch(10, 1, 1);

    UeMacEntity *mac = (UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    //mac->updateCounterMatchedRA_RNTI(RAResponse);

    Simulator::Init()->Schedule(0.001, &UeMacEntity::ReceiveRandomAccessResponseIdealControlMessage, mac, RAResponse);
  } else if (msg->GetMessageType() == IdealControlMessage::CONTENTION_RESOLUTION) {
    ContentionResolutionIdealControlMessage *contetionResolution = (ContentionResolutionIdealControlMessage *) msg;
    //if (Random::Init()->Uniform(0.0, 1.0) <= 0.99) {
    contetionResolution->setError(false);
    //} else {
    //  contetionResolution->setError(true);
    //}

    if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_NO_DATA, DRXManager::PDCCH_RX);
    }

    UeMacEntity *mac = (UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();

    Simulator::Init()->Schedule(0.001, &UeMacEntity::ReceiveContentionResolutionIdealControlMessage, mac, contetionResolution);

  } else if (msg->GetMessageType() == IdealControlMessage::HARQ) {
    HarqIdealControlMessage *Harq = (HarqIdealControlMessage *) msg;
    Harq->setError(false);
    UeMacEntity *mac = (UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    Simulator::Init()->Schedule(0.001, &UeMacEntity::ReceiveHarqIdealControlMessage, mac, Harq);

  } else if (msg->GetMessageType() == IdealControlMessage::BSR_GRANT) {
    BsrGrantIdealControlMessage *bsrGrantMessage = (BsrGrantIdealControlMessage *) msg;
    //if (Random::Init()->Uniform(0.0, 1.0) <= 0.99) {
    bsrGrantMessage->setError(false);
    //} else {
    //  contetionResolution->setError(true);
    //}

    //if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
    //  ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_NO_DATA, DRXManager::PDCCH_RX);
    //}

    UeMacEntity *mac = (UeMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    Simulator::Init()->Schedule(0.001, &UeMacEntity::ReceiveBsrGrantIdealControlMessage, mac, bsrGrantMessage);
  }
}

void
UeLtePhy::SetTxSignalForReferenceSymbols() {
  BandwidthManager *s = GetBandwidthManager();
  std::vector<double> channels = s->GetUlSubChannels();

  //double powerTx_W = pow(10.0, (GetTxPower() - 30.0) / 10.0); // in natural unit
  //double powerTx_dBm = 10 * log10(powerTx_W / channels.size()) + 30; // in dBm

  /* Power Control - Open Loop */
  double P0_PUSCH = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetP0NominalPusch();
  double alpha_PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetAlphaPL();
  double PL = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) GetDevice())->GetReferenceSignalReceivedPower();

  double powerSRSTx_dBm = P0_PUSCH + (10 * log10(channels.size())) + (alpha_PL * PL);

  if (GetTxPower() <= powerSRSTx_dBm) {
    powerSRSTx_dBm = GetTxPower();
  }

  if (USE_POWER_CONTROL_SRS == false) {
    powerSRSTx_dBm = GetTxPower();
  }

  double powerTxPerPRB_dBm = powerSRSTx_dBm - (10 * log10(channels.size()));
  /* --- */

  TransmittedSignal *txSignal = new TransmittedSignal();
  std::vector<double> values;
  std::vector<double>::iterator it;
  for (it = channels.begin(); it != channels.end(); it++) {
    values.push_back(powerTxPerPRB_dBm);
  }
  txSignal->SetValues(values);
  m_txSignalForRerferenceSymbols = txSignal;

  if (((UserEquipment *) GetDevice())->GetConsumptionModel() != NULL) {
    ((UserEquipment*) GetDevice())->GetConsumptionModel()->SetCurrentTxPower(powerSRSTx_dBm);
    ((UserEquipment *) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_TX, DRXManager::PUSCH_TX);
  }

  if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
    ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateUL(DRXManager::PUSCH_TX);
  }

#ifdef UL_SRS_DEBUG
  std::cout << Simulator::Init()->Now() << " SRS TX UE " << GetDevice()->GetIDNetworkNode() << " PRBs " << channels.size() << " PL[dBm] " << PL << " Tx Power[dBm] " << powerSRSTx_dBm << " Tx Power_PRB[dBm] " << powerTxPerPRB_dBm << std::endl;
#endif

  SendReferenceSymbols();

  Simulator::Init()->Schedule(0.080, &UeLtePhy::SetTxSignalForReferenceSymbols, this);
}

TransmittedSignal *
UeLtePhy::GetTxSignalForReferenceSymbols() {
  return m_txSignalForRerferenceSymbols;
}

void
UeLtePhy::SendReferenceSymbols() {
  for (std::vector<NetworkNode*>::iterator it = GetUlChannel()->GetDevices()->begin(); it != GetUlChannel()->GetDevices()->end(); it++) {
    NetworkNode *dst = (*it);

    if (GetDevice()->GetIDNetworkNode() == dst->GetIDNetworkNode()) {
      continue;
    }

    dst->GetPhy()->ReceiveSoundingReferenceSignal(GetDevice(), GetTxSignalForReferenceSymbols()->Copy());
  }

  delete GetTxSignalForReferenceSymbols();
}

void
UeLtePhy::ReceiveReferenceSymbols(NetworkNode *n, TransmittedSignal *s) {
  UserEquipment *ue = (UserEquipment*) GetDevice();
  TransmittedSignal *rxSignal;

  if (GetDlChannel()->GetPropagationLossModel() != NULL) {
    rxSignal = GetDlChannel()->GetPropagationLossModel()->AddLossModel(n, GetDevice(), s); // in dBm
  } else {
    rxSignal = s->Copy();
  }

  delete s;

  AMCModule *amc = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
  std::vector<double> dlQuality;
  std::vector<double> rxSignalValues = rxSignal->Getvalues();

  double noiseInterference = CreateNoisePower(1); // in natural
  double power_W_sum = 0;
  //double noise_sum = 0;

  for (std::vector<double>::iterator it = rxSignalValues.begin(); it != rxSignalValues.end(); it++) {
    double power_dBm;
    if ((*it) != 0) {
      power_dBm = (*it);
    } else {
      power_dBm = 0;
    }

    double power_W = pow(10.0, ((power_dBm - 30) / 10.0)); // in natural unit
    double sinr_W = power_W / noiseInterference; // in natural
    double sinr_dB = 10 * log10(sinr_W); // in dB

    power_W_sum += power_W;
    //noise_sum += noiseInterference;

    //double spectralEfficiency = log2(1.0 + (sinr_W / ((-log(5.0 * 0.00005)) / 1.5)));

    //std::cerr << Simulator::Init()->Now() << " DL UE " << ue->GetIDNetworkNode() << " SINR in W = " << sinr_W << " SINR in dB = " << sinr_dB << " SINR = " << power_dB - (10 * log10(noiseInterference)) << " Efficiency = " << s << std::endl;
    dlQuality.push_back(sinr_dB);
  }

  double rsrp = power_W_sum / rxSignalValues.size(); // in W

  double sinr_W = rsrp / noiseInterference; // in natural
  double sinr_dB = 10 * log10(sinr_W); // in dB

  //double effSinr = sinr_dB;
  double effSinr = GetEesmEffectiveSinr(dlQuality);

  //double effectiveSinr = GetEesmEffectiveSinr(dlQuality);
  //int cqi = amc->GetCQIFromSinr(effectiveSinr);

  /* PDCCH Aggregation Level derived from RSRP */
  int al = amc->GetPdcchAggregationLevelFromRsrp((10 * log10(rsrp)) + 30);
  //int al = amc->GetPdcchAggregationLevelFromSinr(effSinr);

#ifdef DL_RS_DEBUG
  //double s1 = log2(1.0 + (pow(10.0, (effectiveSinr / 10.0)) / ((-log(5.0 * 0.00005)) / 1.5)));
  //int cqi2 = amc->GetCQIFromEfficiency(s1);
  std::cout << "RS RX UE " << ue->GetIDNetworkNode() << " Distance[m] " << ue->GetMobilityModel()->GetAbsolutePosition()->GetDistance(n->GetMobilityModel()->GetAbsolutePosition()) << " RSRP[dBm] " << (10 * log10(rsrp)) + 30 << " PL[dBm] " << (((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetReferenceSignalPower() - (10 * log10(power_W_sum / rxSignalValues.size()) + 30)) << " Noise[dB] " << (10 * log10(noiseInterference) + 30) << " Effective SINR[dB] " << effSinr << " AL " << al << std::endl;
#endif

  ((UserEquipment*) GetDevice())->SetReferenceSignalReceivedPower((10 * log10(rsrp)) + 30); // in dBm
  ((UserEquipment*) GetDevice())->SetReferenceSignalSinr(effSinr); // in dB

  //CQI report
  CreateCqiFeedbacks(dlQuality);

  delete rxSignal;
}

void
UeLtePhy::ResetNoiseInterference() {
  double noise = CreateNoisePower(12);
  for (int i = 0; i < 100; i++) {
    m_noiseInterference[i] = noise;
  }

  Simulator::Init()->Schedule(0.001, &UeLtePhy::ResetNoiseInterference, this);
}

void
UeLtePhy::ReceiveSoundingReferenceSignal(NetworkNode *n, TransmittedSignal *s) {
  TransmittedSignal *rxSignal;

  if (GetUlChannel()->GetPropagationLossModel() != NULL) {
    rxSignal = GetUlChannel()->GetPropagationLossModel()->AddLossModel(n, GetDevice(), s); // in dBm
  } else {
    rxSignal = s->Copy();
  }

  delete s;

  AMCModule *amc = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
  std::vector<double> ulQuality;
  std::vector<double> rxSignalValues = rxSignal->Getvalues();

  double noiseInterference = CreateNoisePower(12); // in natural

  for (std::vector<double>::iterator it = rxSignalValues.begin(); it != rxSignalValues.end(); it++) {
    double power_dBm;
    if ((*it) != 0) {
      power_dBm = (*it);
    } else {
      power_dBm = 0;
    }

    double power_W = pow(10.0, ((power_dBm - 30) / 10.0)); // in natural unit
    double sinr_W = power_W / noiseInterference; // in natural
    double sinr_dB = 10 * log10(sinr_W); // in dB

    //std::cout << Simulator::Init()->Now() << " UE " << n->GetIDNetworkNode() << " SINR in W = " << sinr_W << " SINR in dB = " << sinr_dB << std::endl;
    ulQuality.push_back(sinr_dB);
  }

#ifdef UL_SRS_DEBUG
  double effectiveSinr = GetUplinkEffectiveSinr(ulQuality); // in dB
  int mcs = amc->GetImcsFromSinr(effectiveSinr);
  std::cout << "SRS RX UE " << GetDevice()->GetIDNetworkNode() << " " << GetDevice()->GetMobilityModel()->GetAbsolutePosition()->GetDistance(n->GetMobilityModel()->GetAbsolutePosition()) << " " << effectiveSinr << " " << mcs << std::endl;
#endif

  /*bool r = false;
  int i = 0;
  ENodeB::UserEquipmentRecord *user = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetUserEquipmentRecord(n->GetIDNetworkNode());
  for (std::vector<double>::iterator it = user->GetUplinkChannelStatusIndicator().begin(); it != user->GetUplinkChannelStatusIndicator().end(); it++) {
    if (i < ulQuality.size()) {
      if (ulQuality.at(i++) > (*it)) {
        r = true;
        break;
      }
    }
  }

  if (r) {
    user->SetUplinkChannelStatusIndicator(ulQuality);
  }*/

  ((UserEquipment*) GetDevice())->UpdateUplinkChannelEstimation((UserEquipment*) n, ulQuality);

  delete rxSignal;
}
