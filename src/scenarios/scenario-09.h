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
#include "../flows/application/GameUplink.h"
#include "../flows/application/TimeDriven.h"
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
#include "../channel/propagation-model/macrocell-sub-urban-area-channel-realization.h"

static void Scenario_09(int seed, double radius, int ulBW, int dlBW) {
  int posX_ue, posY_ue;

  int applicationID = 0;
  int srcPort;
  int dstPort;
  double startTime;
  double stopTime = 1.000;

  double stopSimulation = stopTime;

  double direction;
  int speed;

  int idUe = 3;

  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager* networkManager = NetworkManager::Init();
  FlowsManager* flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);
  BetaArrival *betaArrival = BetaArrival::Init();

  simulator->SetStop(stopSimulation);

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
  LteChannel *ulCh = new LteChannel();

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(ulBW, dlBW, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(1, radius, 0.050, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(2, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  enb->SetPreambleInitialReceivedTargetPower(-104.0);
  enb->SetPowerRampingStep(4.0);
  enb->SetP0NominalPusch(-86.0);
  enb->SetAlphaPL(0.8);

  /**/
  enb->SetRrmEntity();
  enb->GetRrmEntity()->SetPdcchScheduler();
  enb->GetRrmEntity()->GetPdcchScheduler()->SetPDCCHModeType(1);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(4);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetLimitUEsToFD(true);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetnUEsToDL(10);

  /**/
  enb->GetRrmEntity()->SetDLScheduler();

  /**/
  enb->GetRrmEntity()->SetULScheduler();

  /**/
  ((UplinkPacketScheduler *) enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_NONE);

  /**/
  ((UplinkPacketScheduler *) enb->GetRrmEntity()->GetUplinkPacketScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_NONE);

  int nUEs = ((radius - 0.045) * 1000) / 5;

  for (int i = 0; i < nUEs; i++) {
    posX_ue = (i * 5) + 50;
    posY_ue = 0;

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, direction, cell, enb, false, Mobility::CONSTANT_POSITION);

    /* set device type to HTC */
    ue->SetDeviceType(0);

    /* Configure DRX Manager */
    //ue->SetDRXManager();

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("3GPP");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);

    /* used to active SR -- true is to SR active */
    ue->SchedulingRequestConfiguration(false, 5);

    ue->GetPhy()->SetDlChannel(ue->GetTargetNode()->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    /* CQI */
    FullbandCqiManager *cqiManager = new FullbandCqiManager();
    cqiManager->SetCqiReportingMode(CqiManager::PERIODIC);
    cqiManager->SetReportingInterval(1000);
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
    
    //MacroCellSubUrbanAreaChannelRealization *c_dl = new MacroCellSubUrbanAreaChannelRealization(enb, ue);
    //c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    //enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    //MacroCellSubUrbanAreaChannelRealization *c_ul = new MacroCellSubUrbanAreaChannelRealization(ue, enb);
    //c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    //enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    //srcPort = 0;
    //dstPort = 100;

    //startTime = 5 + rndGenerator->Uniform(0, 40);

    //VoIP *VoIPapp = new VoIP();
    //VoIPapp->SetApplicationID(applicationID);
    //VoIPapp->SetSource(ue);
    //VoIPapp->SetDestination(enb);
    //VoIPapp->SetSourcePort(srcPort);
    //VoIPapp->SetDestinationPort(dstPort);
    //VoIPapp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    //VoIPapp->SetStartTime(startTime / (double) 1000);
    //VoIPapp->SetStopTime(stopTime);

    //QoSParameters *qos = new QoSParameters(1);
    //qos->SetGBR(12.2);

    //VoIPapp->SetQoSParameters(qos);

    //ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    //VoIPapp->SetClassifierParameters(cp);

    //enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    /* configure the semi-persistent scheduler */
    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    //applicationID++;
    idUe++;
  }
  
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