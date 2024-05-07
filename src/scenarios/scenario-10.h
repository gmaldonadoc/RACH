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
 * Author: Tiago Pedroso da Cruz de Andrade <tiagoandrade@lrc.ic.unicamp.br>
 */

#include <cstring>
#include <cmath>

#include "../load-parameters.h"
#include "../channel/LteChannel.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../networkTopology/Cell.h"
#include "../core/eventScheduler/simulator.h"
#include "../flows/application/VoIP.h"
#include "../flows/application/TraceBased.h"
#include "../flows/application/CBR.h"
#include "../flows/application/TimeDriven.h"
#include "../flows/application/WEB.h"
#include "../flows/QoS/QoSParameters.h"
#include "../componentManagers/FrameManager.h"
#include "../device/ENodeB.h"
#include "../utility/UsersDistribution.h"
#include "../componentManagers/InformationManager.h"
#include "../protocolStack/mac/pdcch-scheduler/pdcch-scheduler.h"
#include "../protocolStack/mac/ue-mac-entity.h"
#include "../arrival/BetaArrival.h"
#include "../energyConsumption/ue-energy-model.h"

static void Scenario_10(int seed, int raMethod, double radius, int ulBW, int dlBW, int ulTDScheduler, int ulFDScheduler, int nUE_HTC_CBR, int prbs, int distance) {
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager* networkManager = NetworkManager::Init();
  FlowsManager* flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);
  BetaArrival *betaArrival = BetaArrival::Init();

  /* Default PRACH Configuration */
  informationManager->ra_Method = 1;
  informationManager->maxHarq_Msg3Tx = 5;
  informationManager->mac_ContentionResolutionTimer = 48;
  informationManager->numberOfRA_Preambles = 52;
  informationManager->preambleTransMax = 10;
  informationManager->ra_PRACH_MaskIndex = 6;

  /* To Conventional Scheme */
  informationManager->backoffIndicatorGeneral = 2; // 20ms

  /* To ACB scheme */
  informationManager->ac_BarringFactor = 0.9;
  informationManager->ac_BarringTime = 4;

  /* To RRS scheme */
  informationManager->numberOfRA_Preambles_HTC = 20;

  /* To EAB scheme */
  informationManager->eab_MonitoringPeriod = 75;

  /* To Specific Backoff scheme */
  informationManager->backoffIndicatorHTC = 2; // 20ms
  informationManager->backoffIndicatorMTC = 9; // 240ms

  /* Aggregation level of PDCCH -> 0 - 1 CCE; 1 - 2 CCEs; 2 - 4 CCEs; 3 - 8 CCEs; -1 - Dynamic */
  informationManager->m_pdcch_format = -1;

  informationManager->qtdPRBsTest = prbs;

  /* Create Channels */
  LteChannel *dlCh = new LteChannel();
  LteChannel *ulCh = new LteChannel();

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(ulBW, dlBW, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(1, radius, 0.100, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(2, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  enb->SetPreambleInitialReceivedTargetPower(-110.0);
  enb->SetPowerRampingStep(4.0);

  /**/
  enb->SetP0NominalPusch(-103.0);
  enb->SetAlphaPL(1.0);

  /**/
  enb->SetPdcchScheduler();
  enb->GetPdcchScheduler()->SetPDCCHModeType(1);
  enb->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(4);
  enb->GetPdcchScheduler()->SetLimitUEsToFD(true);
  enb->GetPdcchScheduler()->SetnUEsToDL(1);

  /**/
  enb->SetDLScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);

  /**/
  enb->SetULScheduler();

  /**/
  switch (ulTDScheduler) {
    case 0: // None
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_NONE);
      break;
    case 1: // Proportional Fair
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_FME);
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetnUsersFD(1);
      break;
    case 2: // Maximum Throughput
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT);
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetnUsersFD(1);
      break;
    case 3: // Z-Base QoS
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_ZBQoS);
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetnUsersFD(1);
      break;
  }

  /**/
  switch (ulFDScheduler) {
    case 0: // None
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_NONE);
      break;
    case 1: // FME Proportional Fair
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_FME);
      break;
    case 2: // Maximum Throughput
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT);
      break;
    case 3: // Round Robin
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_ROUNDROBIN);
      break;
    case 4:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_LIOUMPAS_A1);
      break;
    case 5:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_LIOUMPAS_A2);
      break;
  }

  int nUEs = nUE_HTC_CBR;
  int posX_ue;
  int posY_ue;

  int applicationID = 0;
  int srcPort;
  int dstPort;
  double startTime;
  double stopTime = 10.000;

  int i;
  int idUe = 3;

  /* user equipments are distributed uniformly into a cell */
  vector<CartesianCoordinates*> *positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nUEs);

  for (i = 0; i < nUE_HTC_CBR; i++) {
    //posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    //posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    posX_ue = distance;
    posY_ue = 0;

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    // set device type to HTC
    ue->SetDeviceType(0);

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(8, 8, 300, false, 6);

    /* used to active SR */
    ue->SchedulingRequestConfiguration(false, 1);

    ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    // register UE to the eNodeB
    enb->RegisterUserEquipment(ue);

    // define the channel realization
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    /*MacroCellRuralAreaChannelRealization *c_dl = new MacroCellRuralAreaChannelRealization(enb, ue);
    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    MacroCellRuralAreaChannelRealization *c_ul = new MacroCellRuralAreaChannelRealization(ue, enb);
    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul)*/

    srcPort = 0;
    dstPort = 100;

    startTime = idUe % 20;

    CBR *CBRApp = new CBR();
    CBRApp->SetApplicationID(applicationID);
    CBRApp->SetSource(ue);
    CBRApp->SetDestination(enb);
    CBRApp->SetSourcePort(srcPort);
    CBRApp->SetDestinationPort(dstPort);
    CBRApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApp->SetStartTime(startTime / (double) 1000);
    CBRApp->SetStopTime(stopTime);

    CBRApp->SetInterval(0.016);
    CBRApp->SetSize(500);

    QoSParameters *qos = new QoSParameters(8);
    qos->SetGBR(0.0);

    CBRApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    applicationID++;
    idUe++;
  }

  positions->clear();
  delete positions;

  /* activate the SR over PUCCH */
  enb->SchedulingRequestConfiguration(3);

  simulator->SetStop(stopTime + 0.250);
  simulator->Run();
}