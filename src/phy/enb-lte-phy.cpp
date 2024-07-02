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

#include "enb-lte-phy.h"
#include "ue-lte-phy.h"
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
#include "../channel/propagation-model/propagation-loss-model.h"
#include "../utility/eesm-effective-sinr.h"
#include "../componentManagers/FrameManager.h"

#include "../flows/application/WEB.h"
#include "../utility/spectral-efficiency.h"

/*
 * Noise is computed as follows:
 *  - noise_figure = 5 dB
 *  - n0 = -174 dBm
 *  - subchannel_bandwidth = 180 kHz (in natural unit)
 *
 *  noise[dBm] = noise_figure[dBm] + n0[dBm] + (10 * log10(180000)) = -88.95
 */
#define NOISE -148.95
#define INTERFERENCE 0

EnbLtePhy::EnbLtePhy() {
  SetDevice(NULL);
  SetDlChannel(NULL);
  SetUlChannel(NULL);
  SetBandwidthManager(NULL);
  SetTxSignal(NULL);
  SetErrorModel(NULL);
  SetInterference(NULL);
  SetMaxTxPower(46); //dBm

  //ResetNoiseInterference();

  Simulator::Init()->Schedule(0.000, &EnbLtePhy::SetTxSignalForReferenceSymbols, this);
  Simulator::Init()->Schedule(0.000, &EnbLtePhy::ResetNoiseInterference, this);

  //double t = 0.;
  //while (t < Simulator::Init()->GetTimeToStop()) {
  //Simulator::Init()->Schedule(t, &EnbLtePhy::ResetNoiseInterference, this);
  //t += 0.001;
  //}
}

EnbLtePhy::~EnbLtePhy() {
  Destroy();
}

void
EnbLtePhy::DoSetBandwidthManager() {
  BandwidthManager *s = GetBandwidthManager();
  TransmittedSignal *txSignal = new TransmittedSignal();

  /* based in the specification 3GPP TS 36.942 */
  if ((s->GetDlBandwidth() == 1.4) || (s->GetDlBandwidth() == 3) || (s->GetDlBandwidth() == 5)) {
    SetMaxTxPower(43); //dBm
  } else {
    SetMaxTxPower(46); //dBm
  }

  std::vector<double> channels = s->GetDlSubChannels();
  std::vector<double> values;
  std::vector<double>::iterator it;

  double powerTx = pow(10., (GetTxPower() - 30) / 10); // in natural unit

  double txPower = 10 * log10(powerTx / channels.size()); //in dB

  for (it = channels.begin(); it != channels.end(); it++) {
    values.push_back(txPower);
  }

  txSignal->SetValues(values);

  SetTxSignal(txSignal);
}

void
EnbLtePhy::StartTx(PacketBurst* p) {
  std::cout << "Node " << GetDevice()->GetIDNetworkNode() << " starts phy tx" << std::endl;
  GetDlChannel()->StartTx(p, GetTxSignal(), GetDevice());
}

void
EnbLtePhy::StartRx(PacketBurst *p, TransmittedSignal *txSignal) {
  //#ifdef TEST_DEVICE_ON_CHANNEL
  std::cout << "Node " << GetDevice()->GetIDNetworkNode() << " starts phy rx" << std::endl;
  //#endif

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

    Simulator::Init()->Schedule(0.0005, &EnbLtePhy::EndRx, this, p, txSignal);
  }
}

void
EnbLtePhy::EndRx(PacketBurst *p, TransmittedSignal *txSignal) {
  //#ifdef TEST_DEVICE_ON_CHANNEL
  std::cout << "Node " << GetDevice()->GetIDNetworkNode() << " ends phy rx" << std::endl;
  //#endif

  if (p != NULL) {

    std::vector<double> measuredSinr;
    std::vector<int> channelsForRx;

    std::vector<double> rxSignalValues;
    std::vector<double>::iterator it;

    rxSignalValues = txSignal->Getvalues();

    int chId = 0;
    double power_dBm;
    double m = 0;

    for (it = rxSignalValues.begin(); it != rxSignalValues.end(); it++) {
      if ((*it) > -1000.0) {
        power_dBm = (*it);
        channelsForRx.push_back(chId);

        double power_W = pow(10.0, ((power_dBm - 30) / 10.0)); // in natural unit
        double sinr_W = power_W / CreateNoisePower(12); //(m_noiseInterference[chId] - power_W); // in natural unit
        //double sinr_W = power_W / m_noiseInterference[chId]; // in natural unit
        double sinr_dB = 10 * log10(sinr_W); // in dB unit

        measuredSinr.push_back(sinr_dB);

        m += power_W;

        std::cout << "UE " << p->m_sourceDevice << " PRB " << chId << " Power[W] " << power_W << " SINR[dB] " << sinr_dB << " Noise " << m_noiseInterference[chId] << std::endl;
      }

      chId++;
    }

    std::cout << "Received Data on eNB at " << FrameManager::Init()->GetTTICounter() << " with power equal to " << (10 * log10(m / (double) channelsForRx.size()) + 30) << std::endl;

    bool phyError = false;

    AMCModule *amc = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
    double SINRThreshold = amc->GetSinrFromImcs(p->GetMcsUsed());
    double effectiveSinr = GetUplinkEffectiveSinr(measuredSinr); // in dB
    std::cout << "UE " << p->m_sourceDevice << " PRBs " << measuredSinr.size() << " MCS " << p->GetMcsUsed() << " SINR " << effectiveSinr << " Threshold " << SINRThreshold << std::endl;

    if (effectiveSinr < (SINRThreshold - 1.0)) {
      //phyError = true;
    }

    if ((m / (double) channelsForRx.size()) < pow(10.0, ((((ENodeB *) GetDevice())->GetReferenceSensitivityPowerLevel() - 30) / 10.0))) {
      //phyError = true;
    }

    if ((10 * log10(m / (double) channelsForRx.size()) + 30) < ((ENodeB *) GetDevice())->GetP0NominalPusch()) {
      //phyError = true;
      //std::cout << "**** YES PHY ERROR (node " << GetDevice()->GetIDNetworkNode() << ") " << (10 * log10(m / (double) channelsForRx.size()) + 30) << " " << ((ENodeB *) GetDevice())->GetP0NominalPusch() << " ****" << std::endl;
    }

    /*if (GetErrorModel() != NULL && m_channelsForRx.size() > 0) {
      std::vector<int> cqi_;
      for (int i = 0; i < m_mcsIndexForRx.size(); i++) {
        AMCModule *amc = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
        int cqi = amc->GetCQIFromMCS(m_mcsIndexForRx.at(i));
        cqi_.push_back(cqi);
      }
      phyError = GetErrorModel()->CheckForPhysicalError(m_channelsForRx, cqi_, measuredSinr);

    } else {
      phyError = false;
    }*/

    int TTI = trunc(Simulator::Init()->Now() * double(1000.0));

    if (!phyError) {
      ENodeB::UserEquipmentRecord *s = ((ENodeB *) GetDevice())->GetUserEquipmentRecord(p->m_sourceDevice);
      delete s->m_harqBuffer[TTI % 8];
      s->m_harqBuffer[TTI % 8] = NULL;

      HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(1, false);
      HARQ->SetSourceDevice(GetDevice());
      HARQ->SetDestinationDevice(s->GetUE());
      HARQ->SetHarqProcessId(TTI % 8);

      //std::cout << p->GetSize() << " " << TTI << " " << (TTI % 8) << std::endl;

      Simulator::Init()->Schedule(0.0005, &EnbLtePhy::SendIdealControlMessage, this, HARQ);

      if (p->m_BSR > -1) {
        Simulator::Init()->Schedule(0.0005, &EnbMacEntity::ReceiveBufferStatusReportingMessage, (EnbMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity(), p->m_sourceDevice, p->m_BSR);
      }

      if (p->GetNPackets() > 0) {
        Simulator::Init()->Schedule(0.0005, &NetworkNode::ReceivePacketBurst, GetDevice(), p->Copy());
      }
    } else {
      std::cout << "ENode " << GetDevice()->GetIDNetworkNode() << " error in reception" << std::endl;

      ENodeB::UserEquipmentRecord *s = ((ENodeB *) GetDevice())->GetUserEquipmentRecord(p->m_sourceDevice);

      //s->m_harqBuffer[TTI % 8] = p->Copy();

      //HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(2, false);
      //HARQ->SetSourceDevice(GetDevice());
      //HARQ->SetDestinationDevice(s->GetUE());
      //HARQ->SetHarqProcessId(TTI % 8);

      delete s->m_harqBuffer[TTI % 8];
      s->m_harqBuffer[TTI % 8] = NULL;

      HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(1, false);
      HARQ->SetSourceDevice(GetDevice());
      HARQ->SetDestinationDevice(s->GetUE());
      HARQ->SetHarqProcessId(TTI % 8);

      Simulator::Init()->Schedule(0.0005, &EnbLtePhy::SendIdealControlMessage, this, HARQ);
    }
  }

  delete txSignal;
  delete p;
}

void
EnbLtePhy::SendIdealControlMessage(IdealControlMessage *msg) {
  TransmittedSignal *txSignal = new TransmittedSignal();

  if (msg->GetMessageType() == IdealControlMessage::ALLOCATION_MAP) {
    ENodeB *enb = (ENodeB*) GetDevice();
    ENodeB::UserEquipmentRecords *registeredUe = enb->GetUserEquipmentRecords();
    ENodeB::UserEquipmentRecords::iterator it;

    for (it = registeredUe->begin(); it != registeredUe->end(); it++) {
      (*it)->GetUE()->GetPhy()->ReceiveIdealControlMessage(msg, GetDevice(), txSignal);
    }
  } else if (msg->GetMessageType() == IdealControlMessage::RA_RESPONSE) {
    ENodeB *enb = (ENodeB*) GetDevice();
    ENodeB::UserEquipmentRecords *registeredUe = enb->GetUserEquipmentRecords();
    ENodeB::UserEquipmentRecords::iterator iter;

    for (iter = registeredUe->begin(); iter != registeredUe->end(); iter++) {
      RAResponseIdealControlMessage *newMsg = ((RAResponseIdealControlMessage*) msg)->Copy();
      newMsg->SetDestinationDevice((*iter)->GetUE()->GetPhy()->GetDevice());

      (*iter)->GetUE()->GetPhy()->ReceiveIdealControlMessage(newMsg, GetDevice(), txSignal);
    }

    delete msg;

  } else if (msg->GetMessageType() == IdealControlMessage::HARQ) {
    if (((HarqIdealControlMessage *) msg)->getHARQType() == 1) {
      std::cerr << Simulator::Init()->Now() << " PHY@ENodeB " << GetDevice()->GetIDNetworkNode() << " sent ACK to UE " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
    } else {
      std::cerr << Simulator::Init()->Now() << " PHY@ENodeB " << GetDevice()->GetIDNetworkNode() << " sent NACK to UE " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
    }
    msg->GetDestinationDevice()->GetPhy()->ReceiveIdealControlMessage(msg, GetDevice(), txSignal);

  } else {
    msg->GetDestinationDevice()->GetPhy()->ReceiveIdealControlMessage(msg, GetDevice(), txSignal);
  }
}

void
EnbLtePhy::ReceiveIdealControlMessage(IdealControlMessage *msg, NetworkNode *src, TransmittedSignal *txSignal) {
  if (msg->GetMessageType() == IdealControlMessage::CQI_FEEDBACKS) {
    CqiIdealControlMessage *cqiMsg = (CqiIdealControlMessage*) msg;
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " received CQI_Feedback from UE " << ((UserEquipment*) msg->GetSourceDevice())->GetIDNetworkNode() << std::endl;
    EnbMacEntity *mac = (EnbMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    mac->ReceiveCqiIdealControlMessage(cqiMsg);

  } else if (msg->GetMessageType() == IdealControlMessage::BUFFER_STATUS_REPORTING) {
    BufferStatusReportingIdealControlMessage *srMsg = (BufferStatusReportingIdealControlMessage*) msg;
    //if (Random::Init()->Uniform(0.0, 1.0) <= 0.99) {
    srMsg->setError(false);
    //} else {
    //  srMsg->setError(true);
    //}
    EnbMacEntity *mac = (EnbMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    Simulator::Init()->Schedule(0.001, &EnbMacEntity::ReceiveBufferStatusReportingIdealControlMessage, mac, srMsg);

  } else if (msg->GetMessageType() == IdealControlMessage::RA_PREAMBLE) {
    TransmittedSignal *rxSignal;

    if (GetUlChannel()->GetPropagationLossModel() != NULL) {
      rxSignal = GetUlChannel()->GetPropagationLossModel()->AddLossModel(src, GetDevice(), txSignal); // in dBm
    } else {
      rxSignal = txSignal->Copy();
    }

    delete txSignal;

    std::vector<double> rxSignalValues = rxSignal->Getvalues();

    double powerPRACHRx_dBm = rxSignalValues.at(0);

    delete rxSignal;

    std::cout << "PRACH RX UE " << src->GetIDNetworkNode() << " DISTANCE " << msg->GetSourceDevice()->GetMobilityModel()->GetAbsolutePosition()->GetDistance(msg->GetDestinationDevice()->GetMobilityModel()->GetAbsolutePosition()) << " POWER " << powerPRACHRx_dBm << std::endl;

    RAPreambleIdealControlMessage *RAPreamble = (RAPreambleIdealControlMessage *) msg;

    //if (powerPRACHRx_dBm >= ((ENodeB*) GetDevice())->GetPreambleInitialReceivedTargetPower()) {
    //  RAPreamble->setError(false);
    //} else {
    //  std::cout << Simulator::Init()->Now()
    //            << " PRACH RX UE " << src->GetIDNetworkNode() << " ERROR "
    //            << " PRx " <<  powerPRACHRx_dBm
    //            << " Target " << ((ENodeB*) GetDevice())->GetPreambleInitialReceivedTargetPower() << std::endl;
    //  RAPreamble->setError(true);
    //}

    //if (Random::Init()->Uniform(0.0, 1.0) <= (1 - (1 / (pow(M_E, RAPreamble->getAttempts()))))) {
      //RAPreamble->setError(false);
    //} else {
      //RAPreamble->setError(true);
    //}

    EnbMacEntity *mac = (EnbMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    Simulator::Init()->Schedule(0.001, &EnbMacEntity::ReceiveRandomAccessPreambleMessage, mac, RAPreamble);

  } else if (msg->GetMessageType() == IdealControlMessage::CONNECTION_REQUEST) {
    ConnectionRequestIdealControlMessage *connectionRequest = (ConnectionRequestIdealControlMessage *) msg;
    if (Random::Init()->Uniform(0.0, 1.0) <= 0.90) {
      connectionRequest->setError(false);
    } else {
      connectionRequest->setError(true);
    }
    EnbMacEntity *mac = (EnbMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    Simulator::Init()->Schedule(0.001, &EnbMacEntity::ReceiveConnectionRequestIdealControlMessage, mac, connectionRequest);

  } else if (msg->GetMessageType() == IdealControlMessage::SCHEDULING_REQUEST) {
    SchedulingRequestIdealControlMessage *schedulingRequest = (SchedulingRequestIdealControlMessage *) msg;
    if (Random::Init()->Uniform(0.0, 1.0) <= 0.90) {
      schedulingRequest->setError(false);
    } else {
      schedulingRequest->setError(true);
    }
    EnbMacEntity *mac = (EnbMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    Simulator::Init()->Schedule(0.001, &EnbMacEntity::ReceiveSchedulingRequestIdealControlMessage, mac, schedulingRequest);

  } else if (msg->GetMessageType() == IdealControlMessage::SCHEDULING_REQUEST_PUCCH) {
    SchedulingRequestPUCCHIdealControlMessage *schedulingRequestMessage = (SchedulingRequestPUCCHIdealControlMessage *) msg;
    //if (Random::Init()->Uniform(0.0, 1.0) <= 0.90) {
    schedulingRequestMessage->setError(false);
    //} else {
    //schedulingRequestMessage->setError(true);
    //}
    EnbMacEntity *mac = (EnbMacEntity*) GetDevice()->GetProtocolStack()->GetMacEntity();
    Simulator::Init()->Schedule(0.001, &EnbMacEntity::ReceiveSchedulingRequestPUCCHIdealControlMessage, mac, schedulingRequestMessage);
  }
}

void
EnbLtePhy::ReceiveReferenceSymbols(NetworkNode *n, TransmittedSignal *s) {
  ENodeB::UserEquipmentRecord *user = ((ENodeB*) GetDevice())->GetUserEquipmentRecord(n->GetIDNetworkNode());
  TransmittedSignal *rxSignal;

  if (GetUlChannel()->GetPropagationLossModel() != NULL) {
    rxSignal = GetUlChannel()->GetPropagationLossModel()->AddLossModel(n, GetDevice(), s); // in dBm
  } else {
    rxSignal = s->Copy();
  }

  delete s;

  AMCModule *amc = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
  std::vector<double> measuredSinr;
  std::vector<double> rxSignalValues = rxSignal->Getvalues();

  //double noise_interference = 10.0 * log10(pow(10.0, NOISE / 10)); // dB
  //double noise_interference = NOISE;
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

    std::cout << Simulator::Init()->Now() << " UE " << n->GetIDNetworkNode() << " SINR in W = " << sinr_W << " SINR in dB = " << sinr_dB << std::endl;
    measuredSinr.push_back(sinr_dB);
  }

#ifdef UL_SRS_DEBUG
  //double effectiveSinr = GetEesmEffectiveSinr(measuredSinr); // in dB
  double effectiveSinr = GetUplinkEffectiveSinr(measuredSinr); // in dB
  //double ni = log2(1 + (pow(10.0, (effectiveSinr / 10.0)) / ((-log(5.0 * 0.00005)) / 1.5)));
  double ni = GetSpectralEfficiency(effectiveSinr);
  //int mcs = amc->GetMCSFromCQI(amc->GetCQIFromSinr(effectiveSinr));
  //int mcs = amc->GetMCSIndexFromEfficiency(s1);
  std::cout << "UL_SRS RX UE " << n->GetIDNetworkNode() << " " << GetDevice()->GetMobilityModel()->GetAbsolutePosition()->GetDistance(n->GetMobilityModel()->GetAbsolutePosition()) << " " << effectiveSinr << " " << ni << std::endl;
#endif

  user->SetUplinkChannelStatusIndicator(measuredSinr);

  delete rxSignal;
}

void
EnbLtePhy::SetTxSignalForReferenceSymbols() {
  BandwidthManager *s = GetBandwidthManager();
  std::vector<double> channels = s->GetDlSubChannels();

  double powerTx_W = pow(10.0, (GetTxPower() - 30.0) / 10.0); // in natural unit
  double powerTx_dBm = 10 * log10(powerTx_W / (channels.size() * 12)) + 30; // in dBm

  ((ENodeB*) GetDevice())->SetReferenceSignalPower(powerTx_dBm); // set the reference signal power

#ifdef DL_RS_DEBUG
  std::cout << "DL_RS TX " << powerTx_dBm << std::endl;
#endif

  TransmittedSignal *txSignal = new TransmittedSignal();
  std::vector<double> values;
  std::vector<double>::iterator it;

  for (it = channels.begin(); it != channels.end(); it++) {
    values.push_back(powerTx_dBm);
  }

  txSignal->SetValues(values);
  m_txSignalForRerferenceSymbols = txSignal;

  SendReferenceSymbols();

  //Simulator::Init()->Schedule(0.100, &EnbLtePhy::SetTxSignalForReferenceSymbols, this);

  if (UPLINK == true) {
    Simulator::Init()->Schedule(0.080, &EnbLtePhy::SetTxSignalForReferenceSymbols, this);
  } else {
    Simulator::Init()->Schedule(0.500, &EnbLtePhy::SetTxSignalForReferenceSymbols, this);
  }
}

TransmittedSignal*
EnbLtePhy::GetTxSignalForReferenceSymbols() {
  return m_txSignalForRerferenceSymbols;
}

void
EnbLtePhy::SendReferenceSymbols() {
  ENodeB *enb = (ENodeB*) GetDevice();

  ENodeB::UserEquipmentRecords *registeredUe = enb->GetUserEquipmentRecords();
  ENodeB::UserEquipmentRecords::iterator iter;

  for (iter = registeredUe->begin(); iter != registeredUe->end(); iter++) {
    TransmittedSignal *txSignal = GetTxSignalForReferenceSymbols()->Copy();
    UeLtePhy *uePhy = (UeLtePhy*) (*iter)->GetUE()->GetPhy();
    uePhy->ReceiveReferenceSymbols(enb, txSignal);
  }

  delete GetTxSignalForReferenceSymbols();
}

void
EnbLtePhy::ResetNoiseInterference() {
  //std::cout << "Reset Noise Interference" << std::endl;

  double noise = CreateNoisePower(12);
  for (int i = 0; i < 100; i++) {
    m_noiseInterference[i] = noise;
  }

  Simulator::Init()->Schedule(0.001, &EnbLtePhy::ResetNoiseInterference, this);
}

void
EnbLtePhy::ReceiveSoundingReferenceSignal(NetworkNode *n, TransmittedSignal *s) {
  ENodeB::UserEquipmentRecord *user = ((ENodeB*) GetDevice())->GetUserEquipmentRecord(n->GetIDNetworkNode());
  TransmittedSignal *rxSignal;

  if (GetUlChannel()->GetPropagationLossModel() != NULL) {
    rxSignal = GetUlChannel()->GetPropagationLossModel()->AddLossModel(n, GetDevice(), s); // in dBm
  } else {
    rxSignal = s->Copy();
  }

  delete s;

  AMCModule *amc = GetDevice()->GetProtocolStack()->GetMacEntity()->GetAmcModule();
  std::vector<double> measuredSinr;
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

    std::cout << Simulator::Init()->Now() << " SRS RX UE " << n->GetIDNetworkNode() << " SINR in W = " << sinr_W << " SINR in dB = " << sinr_dB << std::endl;
    measuredSinr.push_back(sinr_dB);
  }

#ifdef UL_SRS_DEBUG
  //double effectiveSinr = GetEesmEffectiveSinr(measuredSinr); // in dB
  double effectiveSinr = GetUplinkEffectiveSinr(measuredSinr); // in dB
  int mcs = amc->GetImcsFromSinr(effectiveSinr);
  std::cout << Simulator::Init()->Now() << " SRS RX UE " << GetDevice()->GetIDNetworkNode() << " Distance[m] " << GetDevice()->GetMobilityModel()->GetAbsolutePosition()->GetDistance(n->GetMobilityModel()->GetAbsolutePosition()) << " Effective SINR[dBm] " << effectiveSinr << " MCS " << mcs << std::endl;
#endif

  user->SetUplinkChannelStatusIndicator(measuredSinr);

  delete rxSignal;
}
