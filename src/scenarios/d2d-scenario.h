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
#include "../channel/propagation-model/d2d-channel-realization.h"

static void d2d_scenario() {

  int i;
  int seed = 1;
  int raMethod = 1;

  // spectrum data
  int ulBw = 10; // in MHz
  int dlBw = 10; // in MHz
  int ulOffset = 0;
  int dlOffset = 0;

  // cell data
  int cellId = 1;
  double cellRadius = 0.5; // in km
  int cellPosX = 0;
  int cellPosY = 0;
  int cellMinDist = 0.100;

  // eNB data
  int enbId = 5;
  int enbPosX = 0;
  int enbPosY = 0;

  // UE data
  int nUEs = 2;
  int ue1Id = 10;
  int ue2Id = 11;
  int ue3Id = 12;
  int ue4Id = 13;

  int ue1PosX, ue1PosY;
  int ue2PosX, ue2PosY;
  int ue3PosX, ue3PosY;
  int ue4PosX, ue4PosY;

  int ueSpeed = 0;
  double ueSpeedDir;

  int semiSch = 0; // only VoIP traffic?

  // CBR app initial data
  int cbrApp1Id = 20;
  int cbrApp2Id = 21;
  int srcPort = 200;
  int dstPort = 201;
  double startTime = 0;
  double stopTime = 20.000;

  // simulation time values
  double stopSimulation = stopTime + 1.0;

  // initialize main components
  std::cout << "# initializing main components..." << std::endl;
  Simulator *simulator = Simulator::Init();
  FrameManager *frameMgr = FrameManager::Init();
  NetworkManager* networkMgr = NetworkManager::Init();
  FlowsManager* flowsMgr = FlowsManager::Init();
  InformationManager *infoMgr = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);

  // setup infoMgr
  std::cout << "# setting up infoMgr ..." << std::endl;

  /* PRACH configuration  */
  infoMgr->ra_Method = raMethod;
  infoMgr->maxHarq_Msg3Tx = 5;
  infoMgr->mac_ContentionResolutionTimer = 48;
  infoMgr->numberOfRA_Preambles = 52;
  infoMgr->preambleTransMax = 10;
  infoMgr->ra_PRACH_MaskIndex = 6;

  /* To Conventional Scheme */
  infoMgr->backoffIndicatorGeneral = 2; // 20ms

  /* To Specific Backoff scheme */
  infoMgr->backoffIndicatorHTC = 2;
  infoMgr->backoffIndicatorMTC = 12;

  /* To ACB scheme */
  infoMgr->ac_BarringFactor = 0.90;
  infoMgr->ac_BarringTime = 4;

  /* To RRS scheme */
  infoMgr->numberOfRA_Preambles_HTC = 12;

  /* Aggregation level of PDCCH -> 0 - 1 CCE; 1 - 2 CCEs; 2 - 4 CCEs; 3 - 8 CCEs; -1 - Dynamic */
  infoMgr->m_pdcch_format = -1;

  // create infrastructure
  std::cout << "# creating infrastructure ..." << std::endl;

  // create uplink and downlink channels
  LteChannel *ulChan = new LteChannel();
  LteChannel *dlChan = new LteChannel();

  // create bandwidth spectrum
  BandwidthManager *spectrum = new BandwidthManager(ulBw, dlBw, ulOffset, dlOffset);

  // create cell
  // params: idCell, radius, minDistance, pos_X, pos_Y
  Cell *cell = networkMgr->CreateCell(cellId, cellRadius, cellMinDist, cellPosX, cellPosY);

  // generate an uniform distribution for the UEs into the cell
  vector<CartesianCoordinates*> *positionsUEs = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nUEs);

  // setup eNB
  std::cout << "# setting up eNB ..." << std::endl;

  // create eNB
  ENodeB *enb = networkMgr->CreateEnodeb(enbId, cell, enbPosX, enbPosY, dlChan, ulChan, spectrum);

  /**/
  enb->SetPreambleInitialReceivedTargetPower(-100.0);
  enb->SetPowerRampingStep(4.0);
  enb->SetP0NominalPusch(-110.0);
  enb->SetAlphaPL(1.0);

  /**/
  enb->SetRrmEntity();
  enb->GetRrmEntity()->SetPdcchScheduler();
  enb->GetRrmEntity()->GetPdcchScheduler()->SetPDCCHModeType(1);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(5);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetLimitUEsToFD(true);

  /**/
  enb->GetRrmEntity()->SetDLScheduler();
  ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkTDScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetnFlowsFD(10);
  ((DownlinkPacketScheduler *) enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkFDScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);

  /**/
  enb->GetRrmEntity()->SetULScheduler();
  ((UplinkPacketScheduler *) enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_FME);
  ((UplinkPacketScheduler *) enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetnUsersFD(10);
  ((UplinkPacketScheduler *) enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_FME);

  // generate positions and directions
  //ue1PosX = positionsUEs->at(ue1Id - 10)->GetCoordinateX();
  //ue1PosY = positionsUEs->at(ue1Id - 10)->GetCoordinateY();
  //ue2PosX = positionsUEs->at(ue2Id - 10)->GetCoordinateX();
  //ue2PosY = positionsUEs->at(ue2Id - 10)->GetCoordinateY();
  //ue3PosX = positionsUEs->at(ue3Id - 10)->GetCoordinateX();
  //ue3PosY = positionsUEs->at(ue3Id - 10)->GetCoordinateY();
  //ue4PosX = positionsUEs->at(ue4Id - 10)->GetCoordinateX();
  //ue4PosY = positionsUEs->at(ue4Id - 10)->GetCoordinateY();
  ue1PosX = 500;
  ue1PosY = 0;
  ue2PosX = 450;
  ue2PosY = 0;
  ue3PosX = 150;
  ue3PosY = 0;
  ue4PosX = 100;
  ue4PosY = 0;
  ueSpeedDir = GetRandomVariable(360.0) * ((2.0 * 3.14) / 360.0);

  // create UEs
  std::cout << "# creating UEs ..." << std::endl;
  UserEquipment *ue1 = new UserEquipment(ue1Id, ue1PosX, ue1PosY, ueSpeed, ueSpeedDir, cell, enb, false, Mobility::RANDOM_DIRECTION);
  UserEquipment *ue2 = new UserEquipment(ue2Id, ue2PosX, ue2PosY, ueSpeed, ueSpeedDir, cell, enb, false, Mobility::RANDOM_DIRECTION);
  UserEquipment *ue3 = new UserEquipment(ue3Id, ue3PosX, ue3PosY, ueSpeed, ueSpeedDir, cell, enb, false, Mobility::RANDOM_DIRECTION);
  UserEquipment *ue4 = new UserEquipment(ue4Id, ue4PosX, ue4PosY, ueSpeed, ueSpeedDir, cell, enb, false, Mobility::RANDOM_DIRECTION);

  // set device type to HTC
  ue1->SetDeviceType(0);
  ue2->SetDeviceType(0);
  ue3->SetDeviceType(0);
  ue4->SetDeviceType(0);

  /**/
  ue1->SetD2DAble(true);
  ue2->SetD2DAble(true);
  ue3->SetD2DAble(true);
  ue4->SetD2DAble(true);

  /* Configure Energy Consumption */
  ue1->SetConsumptionModel("MILETUS");
  ue2->SetConsumptionModel("MILETUS");
  ue3->SetConsumptionModel("MILETUS");
  ue4->SetConsumptionModel("MILETUS");
  /* ----- */

  /* Configure Battery Model */
  ue1->SetBatteryModel();
  ue2->SetBatteryModel();
  ue3->SetBatteryModel();
  ue4->SetBatteryModel();
  /* ----- */

  // set the access class
  ue1->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));
  ue2->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));
  ue3->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));
  ue4->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

  ue1->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);
  ue2->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);
  ue3->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);
  ue4->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);

  ue1->SchedulingRequestConfiguration(false, 1);
  ue2->SchedulingRequestConfiguration(false, 1);
  ue3->SchedulingRequestConfiguration(false, 1);
  ue4->SchedulingRequestConfiguration(false, 1);

  ue1->GetPhy()->SetDlChannel(ue1->GetTargetNode()->GetPhy()->GetDlChannel());
  ue2->GetPhy()->SetDlChannel(ue2->GetTargetNode()->GetPhy()->GetDlChannel());
  ue3->GetPhy()->SetDlChannel(ue3->GetTargetNode()->GetPhy()->GetDlChannel());
  ue4->GetPhy()->SetDlChannel(ue4->GetTargetNode()->GetPhy()->GetDlChannel());
  ue1->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());
  ue2->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());
  ue3->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());
  ue4->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

  networkMgr->GetUserEquipmentContainer()->push_back(ue1);
  networkMgr->GetUserEquipmentContainer()->push_back(ue2);
  networkMgr->GetUserEquipmentContainer()->push_back(ue3);
  networkMgr->GetUserEquipmentContainer()->push_back(ue4);

  /* CQI */
  FullbandCqiManager *cqiManager1 = new FullbandCqiManager();
  cqiManager1->SetCqiReportingMode(CqiManager::PERIODIC);
  cqiManager1->SetReportingInterval(1000);
  cqiManager1->SetDevice(ue1);
  ue1->SetCqiManager(cqiManager1);

  FullbandCqiManager *cqiManager2 = new FullbandCqiManager();
  cqiManager2->SetCqiReportingMode(CqiManager::PERIODIC);
  cqiManager2->SetReportingInterval(1000);
  cqiManager2->SetDevice(ue2);
  ue2->SetCqiManager(cqiManager2);

  FullbandCqiManager *cqiManager3 = new FullbandCqiManager();
  cqiManager3->SetCqiReportingMode(CqiManager::PERIODIC);
  cqiManager3->SetReportingInterval(1000);
  cqiManager3->SetDevice(ue3);
  ue3->SetCqiManager(cqiManager3);

  FullbandCqiManager *cqiManager4 = new FullbandCqiManager();
  cqiManager4->SetCqiReportingMode(CqiManager::PERIODIC);
  cqiManager4->SetReportingInterval(1000);
  cqiManager4->SetDevice(ue4);
  ue4->SetCqiManager(cqiManager4);

  // register UEs into the eNodeB
  enb->RegisterUserEquipment(ue1);
  enb->RegisterUserEquipment(ue2);
  enb->RegisterUserEquipment(ue3);
  enb->RegisterUserEquipment(ue4);

  // define the DL channel realization
  MacroCellUrbanAreaChannelRealization *c1_dl = new MacroCellUrbanAreaChannelRealization(enb, ue1);
  MacroCellUrbanAreaChannelRealization *c2_dl = new MacroCellUrbanAreaChannelRealization(enb, ue2);
  MacroCellUrbanAreaChannelRealization *c3_dl = new MacroCellUrbanAreaChannelRealization(enb, ue3);
  MacroCellUrbanAreaChannelRealization *c4_dl = new MacroCellUrbanAreaChannelRealization(enb, ue4);

  c1_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  c2_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  c3_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  c4_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);

  enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c1_dl);
  enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c2_dl);
  enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c3_dl);
  enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c4_dl);

  // define the UL channel realization
  MacroCellUrbanAreaChannelRealization *c1_ul = new MacroCellUrbanAreaChannelRealization(ue1, enb);
  MacroCellUrbanAreaChannelRealization *c2_ul = new MacroCellUrbanAreaChannelRealization(ue2, enb);
  MacroCellUrbanAreaChannelRealization *c3_ul = new MacroCellUrbanAreaChannelRealization(ue3, enb);
  MacroCellUrbanAreaChannelRealization *c4_ul = new MacroCellUrbanAreaChannelRealization(ue4, enb);

  c1_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  c2_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  c3_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  c4_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);

  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c1_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c2_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c3_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c4_ul);

  // define the D2D channel realization

  D2DChannelRealization *d2d_c12_ul = new D2DChannelRealization(ue1, ue2);
  D2DChannelRealization *d2d_c13_ul = new D2DChannelRealization(ue1, ue3);
  D2DChannelRealization *d2d_c14_ul = new D2DChannelRealization(ue1, ue4);
  D2DChannelRealization *d2d_c21_ul = new D2DChannelRealization(ue2, ue1);
  D2DChannelRealization *d2d_c23_ul = new D2DChannelRealization(ue2, ue3);
  D2DChannelRealization *d2d_c24_ul = new D2DChannelRealization(ue2, ue4);
  D2DChannelRealization *d2d_c31_ul = new D2DChannelRealization(ue3, ue1);
  D2DChannelRealization *d2d_c32_ul = new D2DChannelRealization(ue3, ue2);
  D2DChannelRealization *d2d_c34_ul = new D2DChannelRealization(ue3, ue4);
  D2DChannelRealization *d2d_c41_ul = new D2DChannelRealization(ue4, ue1);
  D2DChannelRealization *d2d_c42_ul = new D2DChannelRealization(ue4, ue2);
  D2DChannelRealization *d2d_c43_ul = new D2DChannelRealization(ue4, ue3);

  d2d_c12_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c13_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c14_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c21_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c23_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c24_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c31_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c32_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c34_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c41_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c42_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  d2d_c43_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);

  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c12_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c13_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c14_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c21_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c23_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c24_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c31_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c32_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c34_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c41_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c42_ul);
  enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(d2d_c43_ul);

  // create CBR applications
  CBR *CBRApp1 = new CBR();
  CBRApp1->SetApplicationID(cbrApp1Id);
  CBRApp1->SetSource(ue1);
  CBRApp1->SetDestination(ue2);
  CBRApp1->SetSourcePort(srcPort);
  CBRApp1->SetDestinationPort(dstPort);
  CBRApp1->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
  CBRApp1->SetStartTime(startTime / (double) 100);
  CBRApp1->SetStopTime(stopTime);
  CBRApp1->SetInterval(0.016);
  CBRApp1->SetSize(1000);

  CBR *CBRApp2 = new CBR();
  CBRApp2->SetApplicationID(cbrApp2Id);
  CBRApp2->SetSource(ue3);
  CBRApp2->SetDestination(ue4);
  CBRApp2->SetSourcePort(srcPort);
  CBRApp2->SetDestinationPort(dstPort);
  CBRApp2->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
  CBRApp2->SetStartTime(startTime / (double) 100);
  CBRApp2->SetStopTime(stopTime);
  CBRApp2->SetInterval(0.016);
  CBRApp2->SetSize(1000);

  QoSParameters *qos1 = new QoSParameters(8);
  QoSParameters *qos2 = new QoSParameters(8);
  qos1->SetGBR(0.0);
  qos2->SetGBR(0.0);

  CBRApp1->SetQoSParameters(qos1);
  CBRApp2->SetQoSParameters(qos2);

  ClassifierParameters *cp1 = new ClassifierParameters(ue1->GetIDNetworkNode(), ue2->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
  ClassifierParameters *cp2 = new ClassifierParameters(ue3->GetIDNetworkNode(), ue4->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
  CBRApp1->SetClassifierParameters(cp1);
  CBRApp2->SetClassifierParameters(cp2);

  enb->SetQoSParametersforUERecord(ue1->GetIDNetworkNode(), qos1);
  enb->SetQoSParametersforUERecord(ue3->GetIDNetworkNode(), qos2);

  /* configure the semi-persistent scheduler */
  ue1->SetSemiSchEnable(semiSch);
  ue2->SetSemiSchEnable(semiSch);
  ue3->SetSemiSchEnable(semiSch);
  ue4->SetSemiSchEnable(semiSch);
  if (semiSch) {
    enb->SetSemiSchConfigUERecord(ue1->GetIDNetworkNode(), 0, 20);
    enb->SetSemiSchConfigUERecord(ue2->GetIDNetworkNode(), 0, 20);
    enb->SetSemiSchConfigUERecord(ue3->GetIDNetworkNode(), 0, 20);
    enb->SetSemiSchConfigUERecord(ue4->GetIDNetworkNode(), 0, 20);
  } else {
    enb->SetSemiSchConfigUERecord(ue1->GetIDNetworkNode(), 0, 0);
    enb->SetSemiSchConfigUERecord(ue2->GetIDNetworkNode(), 0, 0);
    enb->SetSemiSchConfigUERecord(ue3->GetIDNetworkNode(), 0, 0);
    enb->SetSemiSchConfigUERecord(ue4->GetIDNetworkNode(), 0, 0);
  }

  positionsUEs->clear();
  delete positionsUEs;

  /* activate the SR over PUCCH */
  enb->SchedulingRequestConfiguration(18);

  /*
   * Initialize DRX cycle of all UE attached to that eNB.
   * Important: It must go after the declaration of all UE and before Running the simulation
   */
  //enb->InitDRXCycle();

  /*
   * Initialize Energy Consumption cycle of all UE attached to that eNB.
   * Important: It must go after the declaration of all UE and before Running the simulation and after DRX init
   */
  enb->InitConsumptionEnergyCycle();

  /*
   * Initialize Frame Structure cycle of eNB.
   * Important: It must go after the declaration of DRX initialization and before Running the simulation
   */
  frameMgr->Start();

  simulator->SetStop(stopSimulation);

  std::cout << "# starting simulation ..." << std::endl;
  simulator->Run();

  /**/
  UserEquipment* uepointer;
  vector<UserEquipment*>::iterator iter;
  double consumoTotal = 0.0, consumoue = 0.0;
  int timerRx = 0, timerRxTx = 0, timerTx = 0, timerNoData = 0, timerLightSleep = 0, timerDeepSleep = 0, timerLightActive = 0, ue_id = 0;
  int counter = 0;
  std::cout << Simulator::Init()->Now() << std::endl;

  std::cout << "--------------------------" << std::endl;

  for (iter = networkMgr->GetUserEquipmentContainer()->begin(); iter != networkMgr->GetUserEquipmentContainer()->end(); iter++) {
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
    ue_id = uepointer->GetIDNetworkNode();
    std::cout << "UE #" << ue_id
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
  std::cout << "Energia Consumida total [J]: " << consumoTotal << std::endl;

  std::cout << "--------------------------" << std::endl;

  /**/
}
