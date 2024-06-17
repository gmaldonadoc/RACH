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
#include "../flows/application/WEB.h"
#include "../flows/application/GameDownlink.h"
#include "../flows/QoS/QoSParameters.h"
#include "../componentManagers/FrameManager.h"
#include "../device/ENodeB.h"
#include "../utility/UsersDistribution.h"
#include "../componentManagers/InformationManager.h"
#include "../protocolStack/mac/pdcch-scheduler/pdcch-scheduler.h"
#include "../protocolStack/mac/ue-mac-entity.h"
#include "../arrival/BetaArrival.h"
#include "../energyConsumption/ue-energy-model.h"
#include "../energyConsumption/ue-consumption-model.h"
#include "../drx/drx-manager.h"
#include "../protocolStack/mac/rrm/rrm-entity.h"

static void Scenario_13(int seed, int raMethod, double radius, int ulBW, int dlBW, int pdcchScheduler, int maxMSGs4ToSchedule, int dlTDScheduler, int nFlowsToFDScheduler, int dlFDScheduler, int semiSch, int nUE_HTC_VoIP, int nUE_HTC_Video_GBR, int nUE_HTC_Video_nGBR, int nUE_HTC_CBR, int nUE_HTC_Game) {
  int nUEs = nUE_HTC_VoIP + nUE_HTC_Video_GBR + nUE_HTC_Video_nGBR + nUE_HTC_CBR + nUE_HTC_Game;
  int posX_ue;
  int posY_ue;

  int applicationID = 0;
  int srcPort;
  int dstPort;
  double startTime;
  double stopTime = 5.000;

  double stopSimulation = stopTime + 5.0;

  double direction;
  int speed;

  int i;
  int idUe = 3;

  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager* networkManager = NetworkManager::Init();
  FlowsManager* flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);
  BetaArrival *betaArrival = BetaArrival::Init();

  /* PRACH Configuration */
  informationManager->ra_Method = raMethod;
  informationManager->maxHarq_Msg3Tx = 5;
  informationManager->mac_ContentionResolutionTimer = 48;
  informationManager->numberOfRA_Preambles = 52;
  informationManager->preambleTransMax = 10;
  informationManager->ra_PRACH_MaskIndex = 6;

  /* To Conventional Scheme */
  informationManager->backoffIndicatorGeneral = 2; // 20ms

  /* To Specific Backoff scheme */
  informationManager->backoffIndicatorHTC = 2;
  informationManager->backoffIndicatorMTC = 12;

  /* To ACB scheme */
  informationManager->ac_BarringFactor = 0.90;
  informationManager->ac_BarringTime = 4;

  /* To RRS scheme */
  informationManager->numberOfRA_Preambles_HTC = 12;

  /* Aggregation level of PDCCH -> 0 - 1 CCE; 1 - 2 CCEs; 2 - 4 CCEs; 3 - 8 CCEs; -1 - Dynamic */
  informationManager->m_pdcch_format = -1;

  /* Create Channels */
  LteChannel *ulCh = new LteChannel();
  ulCh->SetChannelId(1);
  LteChannel *dlCh = new LteChannel();
  dlCh->SetChannelId(2);

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(ulBW, dlBW, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(1, radius, 0.090, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(2, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  enb->SetPreambleInitialReceivedTargetPower(-104.0);
  enb->SetPowerRampingStep(4.0);
  enb->SetP0NominalPusch(-58.0);
  enb->SetAlphaPL(0.6);

  /**/
  enb->SetRrmEntity();
  enb->GetRrmEntity()->SetPdcchScheduler();
  enb->GetRrmEntity()->GetPdcchScheduler()->SetPDCCHModeType(pdcchScheduler);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(maxMSGs4ToSchedule);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetLimitUEsToFD(true);

  /**/
  enb->GetRrmEntity()->SetDLScheduler();

  /**/
  enb->GetRrmEntity()->SetULScheduler();
  ((UplinkPacketScheduler *) enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_FME);
  ((UplinkPacketScheduler *) enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetnUsersFD(nFlowsToFDScheduler);
  ((UplinkPacketScheduler *) enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_FME);

  /**/
  switch (dlTDScheduler) {
    case 0: // None
      ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkTDScheduler(ENodeB::DLScheduler_TYPE_NONE);
      break;
    case 1: // Proportional Fair
      ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkTDScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
      ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetnFlowsFD(nFlowsToFDScheduler);
      break;
    case 2: // Maximum Throughput
      ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkTDScheduler(ENodeB::DLScheduler_TYPE_MAXIMUM_THROUGHPUT);
      ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetnFlowsFD(nFlowsToFDScheduler);
      break;
  }

  /**/
  switch (dlFDScheduler) {
    case 0: // None
      ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkFDScheduler(ENodeB::DLScheduler_TYPE_NONE);
      break;
    case 1: // FME Proportional Fair
      ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkFDScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
      break;
    case 2: // Maximum Throughput
      ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkFDScheduler(ENodeB::DLScheduler_TYPE_MAXIMUM_THROUGHPUT);
      break;
  }

  /* user equipments are distributed uniformly into a cell */
  //vector<CartesianCoordinates*> *positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nUEs);
  vector<CartesianCoordinates*> *positions = GetUniformUsersDistribution(cell->GetIdCell(), nUEs);

  for (i = 0; i < nUE_HTC_VoIP; i++) {
    direction = GetRandomVariable(360.0) * ((2.0 * 3.14) / 360.0);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, direction, cell, enb, false, Mobility::RANDOM_DIRECTION);

    /* set device type to HTC */
    ue->SetDeviceType(0);

    /* Configure DRX Manager */
    //ue->SetDRXManager();

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("MILETUS");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    /* set the access class */
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);

    /* */
    ue->SchedulingRequestConfiguration(false, 1);

    ue->GetPhy()->SetDlChannel(ue->GetTargetNode()->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(ue->GetTargetNode()->GetPhy()->GetUlChannel());

    /* CQI */
    FullbandCqiManager *cqiManager = new FullbandCqiManager();
    cqiManager->SetCqiReportingMode(CqiManager::PERIODIC);
    cqiManager->SetReportingInterval(10);
    cqiManager->SetDevice(ue);
    ue->SetCqiManager(cqiManager);

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    /* register UE to the eNodeB */
    enb->RegisterUserEquipment(ue);

    /* define the channel realization -- DL */
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);

    /* define the channel realization -- UL */
    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    srcPort = 0;
    dstPort = 100;

    startTime = 5 + rndGenerator->Uniform(0, 40);

    VoIP *VoIPapp = new VoIP();
    VoIPapp->SetApplicationID(applicationID);
    VoIPapp->SetSource(enb);
    VoIPapp->SetDestination(ue);
    VoIPapp->SetSourcePort(srcPort);
    VoIPapp->SetDestinationPort(dstPort);
    VoIPapp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    VoIPapp->SetStartTime(startTime / (double) 1000);
    VoIPapp->SetStopTime(stopTime);

    QoSParameters *qos = new QoSParameters(1);
    qos->SetGBR(12.2);

    VoIPapp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(enb->GetIDNetworkNode(), ue->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    VoIPapp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    /* configure the semi-persistent scheduler */
    ue->SetSemiSchEnable(semiSch);
    if (semiSch) {
      enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 20);
    } else {
      enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);
    }

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_HTC_Video_GBR; i++) {
    direction = GetRandomVariable(360.) * ((2. * 3.14) / 360.);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 3, direction, cell, enb, false, Mobility::RANDOM_DIRECTION);

    /* set device type to HTC */
    ue->SetDeviceType(0);

    /* Configure DRX Manager */
    //ue->SetDRXManager();

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("MILETUS");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    /* set the access class */
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    /* used by QoS-Dracon random-access scheme */
    ue->GetProtocolStack()->GetMacEntity()->SetQCI(2, 4, 150, true, 4);

    /* used to active SR -- false because the downlink not use SR*/
    ue->SchedulingRequestConfiguration(false, 1);

    ue->GetPhy()->SetDlChannel(ue->GetTargetNode()->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(ue->GetTargetNode()->GetPhy()->GetUlChannel());

    /* CQI */
    FullbandCqiManager *cqiManager = new FullbandCqiManager();
    cqiManager->SetCqiReportingMode(CqiManager::PERIODIC);
    cqiManager->SetReportingInterval(10);
    cqiManager->SetDevice(ue);
    ue->SetCqiManager(cqiManager);

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    /* register UE to the eNodeB */
    enb->RegisterUserEquipment(ue);

    // define the channel realization
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);

    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    srcPort = 0;
    dstPort = 100;

    startTime = 5 + rndGenerator->Uniform(0, 40);

    string video_trace("foreman_H264_");
    string _file(path + "src/flows/application/Trace/" + video_trace + "128k.dat");

    TraceBased *videoApp = new TraceBased();
    videoApp->SetApplicationID(applicationID);
    videoApp->SetSource(enb);
    videoApp->SetDestination(ue);
    videoApp->SetSourcePort(srcPort);
    videoApp->SetDestinationPort(dstPort);
    videoApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetStartTime(startTime / (double) 1000);
    videoApp->SetStopTime(stopTime);

    videoApp->SetTraceFile(_file);

    QoSParameters *qos = new QoSParameters(2);

    qos->SetGBR(128);

    videoApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(enb->GetIDNetworkNode(), ue->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_HTC_Video_nGBR; i++) {
    direction = GetRandomVariable(360.) * ((2. * 3.14) / 360.);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 3, direction, cell, enb, false, Mobility::RANDOM_DIRECTION);

    /* set device type to HTC */
    ue->SetDeviceType(0);

    /* Configure DRX Manager */
    ue->SetDRXManager();

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("MILETUS");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    // used by QoS-Dracon random-access scheme
    ue->GetProtocolStack()->GetMacEntity()->SetQCI(6, 6, 300, false, 4);

    // used to active SR
    ue->SchedulingRequestConfiguration(false, 1);

    ue->GetPhy()->SetDlChannel(ue->GetTargetNode()->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(ue->GetTargetNode()->GetPhy()->GetUlChannel());

    /* CQI */
    FullbandCqiManager *cqiManager = new FullbandCqiManager();
    cqiManager->SetCqiReportingMode(CqiManager::PERIODIC);
    cqiManager->SetReportingInterval(1.000);
    cqiManager->SetDevice(ue);
    ue->SetCqiManager(cqiManager);

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

    srcPort = 0;
    dstPort = 100;

    startTime = 5 + rndGenerator->Uniform(0, 40);

    string video_trace("foreman_H264_");
    string _file(path + "src/flows/application/Trace/" + video_trace + "242k.dat");

    TraceBased *videoApp = new TraceBased();
    videoApp->SetApplicationID(applicationID);
    videoApp->SetSource(enb);
    videoApp->SetDestination(ue);
    videoApp->SetSourcePort(srcPort);
    videoApp->SetDestinationPort(dstPort);
    videoApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetStartTime(startTime / (double) 1000);
    videoApp->SetStopTime(stopTime);

    videoApp->SetTraceFile(_file);

    QoSParameters *qos = new QoSParameters(6);

    qos->SetGBR(242);

    videoApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(enb->GetIDNetworkNode(), ue->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_HTC_CBR; i++) {
    direction = GetRandomVariable(360.) * ((2. * 3.14) / 360.);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 3, direction, cell, enb, false, Mobility::RANDOM_DIRECTION);

    /* set device type to HTC */
    ue->SetDeviceType(0);

    /* Configure DRX Manager */
    //ue->SetDRXManager();

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("MILETUS");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(8, 8, 300, false, 6);

    // used to active SR
    ue->SchedulingRequestConfiguration(false, 1);

    ue->GetPhy()->SetDlChannel(ue->GetTargetNode()->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(ue->GetTargetNode()->GetPhy()->GetUlChannel());

    /* CQI */
    FullbandCqiManager *cqiManager = new FullbandCqiManager();
    cqiManager->SetCqiReportingMode(CqiManager::PERIODIC);
    cqiManager->SetReportingInterval(10);
    cqiManager->SetDevice(ue);
    ue->SetCqiManager(cqiManager);

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

    srcPort = 0;
    dstPort = 100;

    startTime = 5 + rndGenerator->Uniform(0, 40);

    CBR *CBRApp = new CBR();
    CBRApp->SetApplicationID(applicationID);
    CBRApp->SetSource(enb);
    CBRApp->SetDestination(ue);
    CBRApp->SetSourcePort(srcPort);
    CBRApp->SetDestinationPort(dstPort);
    CBRApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApp->SetStartTime(startTime / (double) 100);
    CBRApp->SetStopTime(stopTime);

    CBRApp->SetInterval(0.008);
    //CBRApp->SetInterval(0.032);
    CBRApp->SetSize(250);

    QoSParameters *qos = new QoSParameters(8);
    qos->SetGBR(0.0);

    CBRApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(enb->GetIDNetworkNode(), ue->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    applicationID++;
    idUe++;
  }
  
  for (i = 0; i < nUE_HTC_Game; i++) {
    direction = GetRandomVariable(360.) * ((2. * 3.14) / 360.);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 3, direction, cell, enb, false, Mobility::RANDOM_DIRECTION);

    /* set device type to HTC */
    ue->SetDeviceType(0);

    /* Configure DRX Manager */
    //ue->SetDRXManager();

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("3GPP");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    /* set the access class */
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(8, 8, 300, false, 6);

    /* used to active SR */
    ue->SchedulingRequestConfiguration(false, 1);

    ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    /* CQI */
    FullbandCqiManager *cqiManager = new FullbandCqiManager();
    cqiManager->SetCqiReportingMode(CqiManager::PERIODIC);
    cqiManager->SetReportingInterval(1000);
    cqiManager->SetDevice(ue);
    ue->SetCqiManager(cqiManager);

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    /* register UE to the eNodeB */
    enb->RegisterUserEquipment(ue);

    // define the channel realization
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    
    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    srcPort = 0;
    dstPort = 100;

    startTime = 5 + rndGenerator->Uniform(0, 40);

    GameDownlink *GameApp = new GameDownlink();
    GameApp->SetApplicationID(applicationID);
    GameApp->SetSource(enb);
    GameApp->SetDestination(ue);
    GameApp->SetSourcePort(srcPort);
    GameApp->SetDestinationPort(dstPort);
    GameApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    GameApp->SetStartTime(startTime / (double) 1000);
    GameApp->SetStopTime(stopTime);

    QoSParameters *qos = new QoSParameters(7);
    qos->SetGBR(0.0);

    GameApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(enb->GetIDNetworkNode(), ue->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    GameApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    applicationID++;
    idUe++;
  }

  positions->clear();
  delete positions;

  /* activate the SR over PUCCH */
  enb->SchedulingRequestConfiguration(18);

  /*
   * Initialize DRX management cycle of all UEs.
   * Important: It must go after the declaration of all UEs and before running the simulation
   */
  enb->InitDRXCycle();

  /*
   * Initialize Energy Consumption management cycle of all UEs.
   * Important: It must go after the declaration of all UEs and before running the simulation and after DRX initiation
   */
  enb->InitConsumptionEnergyCycle();

  /*
   * Initialize Frame management cycle.
   * Important: It must go after the declaration of DRX and Energy initialization and before Running the simulation
   */
  frameManager->Start();

  simulator->SetStop(stopSimulation);

  simulator->Run();

  /**/
  UserEquipment* uepointer;
  vector<UserEquipment*>::iterator iter;
  double consumoTotal = 0.0, consumoue = 0.0;
  int timerRx = 0, timerRxTx = 0, timerTx = 0, timerNoData = 0, timerLightSleep = 0, timerDeepSleep = 0, timerLightActive = 0, idue = 0;
  int counter = 0;
  //std::cerr << Simulator::Init()->Now() << std::endl;

  std::cerr << "--------------------------" << std::endl;
  for (iter = networkManager->GetUserEquipmentContainer()->begin(); iter != networkManager->GetUserEquipmentContainer()->end(); iter++) {
    counter++;
    uepointer = *iter;
    consumoue = uepointer->GetConsumptionModel()->getConsumption();
    consumoTotal += consumoue;
    timerRx = uepointer->GetConsumptionModel()->gettActiveRx();
    timerRxTx = uepointer->GetConsumptionModel()->gettActiveRxTx();
    timerTx = uepointer->GetConsumptionModel()->gettActiveTx();
    timerNoData = uepointer->GetConsumptionModel()->gettActiveNoData();
    timerLightSleep = uepointer->GetConsumptionModel()->gettLightSleep();
    timerDeepSleep = uepointer->GetConsumptionModel()->gettDeepSleep();
    timerLightActive = uepointer->GetConsumptionModel()->gettLightActive();
    idue = uepointer->GetIDNetworkNode();
    std::cerr << "UE #" << idue
            << " --- Energia Consumida [J]: " << consumoue
            << " - Tempo Tx [ms]: " << timerTx
            << " - Tempo Rx [ms]: " << timerRx
            << " - Tempo RxTx [ms]: " << timerRxTx
            << " - Tempo NoData [ms]: " << timerNoData
            << " - Tempo LightSleep [ms]: " << timerLightSleep
            << " - Tempo DeepSleep [ms]: " << timerDeepSleep
            << " - Tempo Wakeup [ms]: " << uepointer->GetConsumptionModel()->gettSleepToActive()
            << " - Tempo Powerdown [ms]: " << uepointer->GetConsumptionModel()->gettActiveToSleep()
            << " - Bateria: " << uepointer->GetBatteryModel()->GetCurrentCapacity()
            << "mA " << uepointer->GetBatteryModel()->GetBatteryCharge() << "%"
            << std::endl;
  }
  std::cerr << "--------------------------" << std::endl;

  std::cerr << "Energia Consumida total [J]: " << consumoTotal << std::endl;
  /**/
}