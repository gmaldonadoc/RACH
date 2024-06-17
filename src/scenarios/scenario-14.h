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
#include "../energyConsumption/ue-consumption-model.h"
#include "../drx/drx-manager.h"
#include "../protocolStack/mac/rrm/rrm-entity.h"

static void Scenario_14(int seed, double radius, int ulBW, int dlBW, int nUEs, int nAPPs)
{
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager *networkManager = NetworkManager::Init();
  FlowsManager *flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);
  BetaArrival *betaArrival = BetaArrival::Init();

  /* PRACH Configuration */
  informationManager->ra_Method = 1;
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
  LteChannel *dlCh = new LteChannel();
  dlCh->SetChannelId(1);
  LteChannel *ulCh = new LteChannel();
  ulCh->SetChannelId(2);

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(ulBW, dlBW, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(1, radius, 0.100, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(2, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  enb->SetPreambleInitialReceivedTargetPower(-110.0);
  enb->SetPowerRampingStep(4.0);
  enb->SetP0NominalPusch(-103.0);
  enb->SetAlphaPL(1.0);

  /**/
  enb->SetRrmEntity();
  enb->GetRrmEntity()->SetPdcchScheduler();
  enb->GetRrmEntity()->GetPdcchScheduler()->SetPDCCHModeType(1);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(5);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetLimitUEsToFD(true);

  /**/
  enb->GetRrmEntity()->SetDLScheduler();
  ((DownlinkPacketScheduler *)enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkTDScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
  ((DownlinkPacketScheduler *)enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetnFlowsFD(10);
  ((DownlinkPacketScheduler *)enb->GetRrmEntity()->GetDownlinkPacketScheduler())->SetDownlinkFDScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);

  /**/
  enb->GetRrmEntity()->SetULScheduler();
  ((UplinkPacketScheduler *)enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_FME);
  ((UplinkPacketScheduler *)enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetnUsersFD(10);
  ((UplinkPacketScheduler *)enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_FME);

  int posX_ue;
  int posY_ue;

  int applicationID = 0;
  int srcPort;
  int dstPort;
  double startTime;
  double stopTime = 10.000;

  double stopSimulation = stopTime + 5.0;

  int i;
  int idUe = 3;

  /* user equipments are distributed uniformly into a cell */
  vector<CartesianCoordinates *> *positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), (nUEs * 2));

  std::vector<UserEquipment *> *attachedTxUEs = new std::vector<UserEquipment *>();
  for (i = 0; i < nUEs; i++)
  {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    // posX_ue = 120;
    // posY_ue = 0;

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    std::cout << "Position UE " << ue->GetIDNetworkNode() << " --- X = " << posX_ue << ", Y = " << posY_ue << ", Abs = " << ue->GetMobilityModel()->GetAbsolutePosition()->GetDistance(enb->GetMobilityModel()->GetAbsolutePosition()) << std::endl;

    /* set device type to HTC */
    ue->SetDeviceType(0);

    /* Configure DRX Manager */
    // ue->SetDRXManager();

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("3GPP");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);

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

    attachedTxUEs->push_back(ue);

    idUe++;
  }

  std::vector<UserEquipment *> *attachedRxUEs = new std::vector<UserEquipment *>();
  for (i = 0; i < nUEs; i++)
  {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    // posX_ue = 240;
    // posY_ue = 0;

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    std::cout << "Position UE " << ue->GetIDNetworkNode() << " --- X = " << posX_ue << ", Y = " << posY_ue << ", Abs = " << ue->GetMobilityModel()->GetAbsolutePosition()->GetDistance(enb->GetMobilityModel()->GetAbsolutePosition()) << std::endl;

    /* set device type to HTC */
    ue->SetDeviceType(0);

    /* Configure DRX Manager */
    // ue->SetDRXManager();

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("3GPP");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);

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

    attachedRxUEs->push_back(ue);

    idUe++;
  }

  for (i = 0; i < nAPPs; i++)
  {
    srcPort = 101;
    dstPort = 102;

    startTime = floor(rndGenerator->Uniform(0.0, 1.0) * 1000);

    CBR *CBRApp = new CBR();
    CBRApp->SetApplicationID(applicationID);
    CBRApp->SetSource(attachedTxUEs->at(i));
    CBRApp->SetDestination(attachedRxUEs->at(i));
    CBRApp->SetSourcePort(srcPort);
    CBRApp->SetDestinationPort(dstPort);
    CBRApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApp->SetStartTime(startTime / (double)1000);
    CBRApp->SetStopTime(stopTime);

    // CBRApp->SetInterval(0.008);
    CBRApp->SetInterval(0.032);
    CBRApp->SetSize(1000);

    QoSParameters *qos = new QoSParameters(8);
    qos->SetGBR(0.0);

    CBRApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(attachedTxUEs->at(i)->GetIDNetworkNode(), attachedRxUEs->at(i)->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(attachedTxUEs->at(i)->GetIDNetworkNode(), qos);

    attachedTxUEs->at(i)->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(attachedTxUEs->at(i)->GetIDNetworkNode(), 0, 0);

    applicationID++;
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
  UserEquipment *uepointer;
  vector<UserEquipment *>::iterator iter;
  double consumoTotal = 0.0, consumoue = 0.0;
  int timerRx = 0, timerRxTx = 0, timerTx = 0, timerNoData = 0, timerLightSleep = 0, timerDeepSleep = 0, timerLightActive = 0, idue = 0;
  int counter = 0;
  // std::cerr << Simulator::Init()->Now() << std::endl;

  std::cerr << "--------------------------" << std::endl;
  for (iter = networkManager->GetUserEquipmentContainer()->begin(); iter != networkManager->GetUserEquipmentContainer()->end(); iter++)
  {
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

  // for (int i = 0; i < networkManager->GetMGatewayContainer->size(); i++)
  // {
  // }
  /**/
}