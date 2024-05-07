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

static void Scenario_06(int seed, int raMethod, double radius, int ulBW, int dlBW, int pdcchScheduler, int limitUEsToUlScheduler, int maxMSGs4ToSchedule, int ulTDScheduler, int nUsersToFDScheduler, int ulFDScheduler, int semiSch, int nUE_HTC_VoIP, int nUE_HTC_Video, int nUE_HTC_CBR, int nUE_MTC_TimeDriven, int nUE_MTC_Emergency, int nUE_MTC_Video) {
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager* networkManager = NetworkManager::Init();
  FlowsManager* flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);
  BetaArrival *betaArrival = BetaArrival::Init();

  /* RACH Configurarion*/
  informationManager->ra_Method = raMethod;
  informationManager->maxHarq_Msg3Tx = 5;
  informationManager->mac_ContentionResolutionTimer = 48;
  informationManager->numberOfRA_Preambles = 52;
  informationManager->preambleTransMax = 10;
  informationManager->ra_PRACH_MaskIndex = 6;

  /* To Especific Backoff scheme */
  //informationManager->backoffIndicatorGeneral = backoffIndex_general;
  //informationManager->backoffIndicatorHTC = backoffIndex_HTC;
  //informationManager->backoffIndicatorMTC = backoffIndex_MTC;

  /* To ACB scheme */
  informationManager->ac_BarringFactor = 0.95;
  informationManager->ac_BarringTime = 4;

  /* To RRS scheme */
  informationManager->numberOfRA_Preambles_HTC = 20;

  /* Aggregation level of PDCCH - 2 CCEs per DCI */
  informationManager->m_pdcch_format = -1;

  /* Create Channels */
  LteChannel *dlCh = new LteChannel();
  LteChannel *ulCh = new LteChannel();

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(ulBW, dlBW, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(1, radius, 0.040, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(2, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  enb->SetPdcchScheduler();
  enb->GetPdcchScheduler()->SetPDCCHModeType(pdcchScheduler);
  enb->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(maxMSGs4ToSchedule);
  enb->GetPdcchScheduler()->SetLimitUEsToFD(limitUEsToUlScheduler);

  /**/
  enb->SetDLScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);

  /**/
  enb->SetULScheduler(ENodeB::ULScheduler_TYPE_FME);

  /**/
  switch (ulTDScheduler) {
    case 1:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_FME);
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetnUsersFD(nUsersToFDScheduler);
      break;
    case 2:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT);
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetnUsersFD(nUsersToFDScheduler);
      break;
    case 3:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_ZBQoS);
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetnUsersFD(nUsersToFDScheduler);
      break;
    case 4:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_ROUNDROBIN);
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetnUsersFD(nUsersToFDScheduler);
      break;
  }

  /**/
  switch (ulFDScheduler) {
    case 1:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_FME);
      break;
    case 2:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_MAXIMUM_THROUGHPUT);
      break;
    case 3:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_ZBQoS);
      break;
    case 4:
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_ROUNDROBIN);
      break;
  }

  int nUEs = nUE_HTC_VoIP + nUE_HTC_Video + nUE_HTC_CBR + nUE_MTC_TimeDriven + nUE_MTC_Emergency + nUE_MTC_Video;
  int posX_ue;
  int posY_ue;

  int applicationID = 0;
  int srcPort;
  int dstPort;
  double startTime;
  double stopTime = 10.000;

  double speedDirection;

  int i;
  int idUe = 3;

  /* user equipments are distributed uniformly into a cell */
  vector<CartesianCoordinates*> *positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nUEs);

  for (i = 0; i < nUE_HTC_VoIP; i++) {
    speedDirection = GetRandomVariable(360.0) * ((2.0 * 3.14) / 360.0);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, speedDirection, cell, enb, false, Mobility::RANDOM_DIRECTION);

    // set device type to HTC
    ue->SetDeviceType(0);

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);

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

    srcPort = 0;
    dstPort = 100;

    startTime = idUe % 20;

    VoIP *VoIPapp = new VoIP();
    VoIPapp->SetApplicationID(applicationID);
    VoIPapp->SetSource(ue);
    VoIPapp->SetDestination(enb);
    VoIPapp->SetSourcePort(srcPort);
    VoIPapp->SetDestinationPort(dstPort);
    VoIPapp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    VoIPapp->SetStartTime(startTime / (double) 1000);
    VoIPapp->SetStopTime(stopTime);

    QoSParameters *qos = new QoSParameters(1);
    qos->SetGBR(8.4);

    VoIPapp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    VoIPapp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    /* configure the semi-persistent scheduler */
    ue->SetSemiSchEnable(semiSch);
    if (semiSch) {
      //enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), startTime, 20);
      enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 20);
      //Simulator::Init()->Schedule((startTime / 1000) + 0.004, &UeMacEntity::ScheduleUplinkTransmissionSemiSch, (UeMacEntity *) (ue->GetProtocolStack()->GetMacEntity()));
    } else {
      enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);
    }

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_HTC_Video; i++) {
    speedDirection = GetRandomVariable(360.) * ((2. * 3.14) / 360.);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 3, speedDirection, cell, enb, false, Mobility::RANDOM_DIRECTION);

    // set device type to HTC
    ue->SetDeviceType(0);

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    // used by QoS-Dracon random-access scheme
    ue->GetProtocolStack()->GetMacEntity()->SetQCI(2, 4, 150, true, 4);

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

    srcPort = 0;
    dstPort = 100;

    startTime = idUe % 20;

    string video_trace("foreman_H264_");
    string _file(path + "src/flows/application/Trace/" + video_trace + "128k.dat");

    TraceBased *videoApp = new TraceBased();
    videoApp->SetApplicationID(applicationID);
    videoApp->SetSource(ue);
    videoApp->SetDestination(enb);
    videoApp->SetSourcePort(srcPort);
    videoApp->SetDestinationPort(dstPort);
    videoApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetStartTime(startTime / (double) 1000);
    videoApp->SetStopTime(stopTime);

    videoApp->SetTraceFile(_file);

    QoSParameters *qos = new QoSParameters(2);

    qos->SetGBR(128);

    videoApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_HTC_CBR; i++) {
    speedDirection = GetRandomVariable(360.) * ((2. * 3.14) / 360.);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 3, speedDirection, cell, enb, false, Mobility::RANDOM_DIRECTION);

    // set device type to HTC
    ue->SetDeviceType(0);

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(8, 8, 300, false, 6);

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

  if (nUE_MTC_TimeDriven > 0) {
    string _file(path + "src/arrival/probability-6-10.txt");
    vector<TimeArrival*> *MTCArrivals = betaArrival->buildArrival(6, 10, nUE_MTC_TimeDriven, _file);

    int itSlot;

    for (itSlot = 0; itSlot < MTCArrivals->size(); itSlot++) {

      int i;

      for (i = 0; i < MTCArrivals->at(itSlot)->getNumberArrivals(); i++) {
        posX_ue = positions->at(idUe - 3)->GetCoordinateX();
        posY_ue = positions->at(idUe - 3)->GetCoordinateY();

        UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

        /* set device type to MTC */
        ue->SetDeviceType(1);

        /* set the access class */
        ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

        ue->GetProtocolStack()->GetMacEntity()->SetQCI(9, 9, 300, false, 6);

        ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
        ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

        networkManager->GetUserEquipmentContainer()->push_back(ue);

        /* register UE to the eNodeB */
        enb->RegisterUserEquipment(ue);

        /* define the channel realization */
        MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
        c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
        enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
        MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
        c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
        enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

        srcPort = 0;
        dstPort = 100;

        startTime = MTCArrivals->at(itSlot)->getTimeArrivals();

        TimeDriven *TimeDrivenApp = new TimeDriven();
        TimeDrivenApp->SetApplicationID(applicationID);
        TimeDrivenApp->SetSource(ue);
        TimeDrivenApp->SetDestination(enb);
        TimeDrivenApp->SetSourcePort(srcPort);
        TimeDrivenApp->SetDestinationPort(dstPort);
        TimeDrivenApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
        TimeDrivenApp->SetStartTime(startTime);
        TimeDrivenApp->SetStopTime(stopTime);

        TimeDrivenApp->SetInterval(1.000);
        TimeDrivenApp->SetSize(125);

        QoSParameters *qos = new QoSParameters(9);
        qos->SetGBR(0.0);

        TimeDrivenApp->SetQoSParameters(qos);

        ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
        TimeDrivenApp->SetClassifierParameters(cp);

        enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

        ue->SetSemiSchEnable(false);
        enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

        applicationID++;
        idUe++;
      }
    }
    MTCArrivals->clear();
    delete MTCArrivals;
  }

  for (i = 0; i < nUE_MTC_Emergency; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    /* set device type to MTC */
    ue->SetDeviceType(1);

    /* set the access class */
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);

    ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    /* register UE to the eNodeB */
    enb->RegisterUserEquipment(ue);

    /* define the channel realization */
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    srcPort = 0;
    dstPort = 100;

    TimeDriven *TimeDrivenApp = new TimeDriven();
    TimeDrivenApp->SetApplicationID(applicationID);
    TimeDrivenApp->SetSource(ue);
    TimeDrivenApp->SetDestination(enb);
    TimeDrivenApp->SetSourcePort(srcPort);
    TimeDrivenApp->SetDestinationPort(dstPort);
    TimeDrivenApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    TimeDrivenApp->SetStartTime(stopTime / 2);
    TimeDrivenApp->SetStopTime(stopTime);

    TimeDrivenApp->SetInterval(20.000);
    TimeDrivenApp->SetSize(8);

    QoSParameters *qos = new QoSParameters(1);
    qos->SetGBR(0.0);

    TimeDrivenApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    TimeDrivenApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_MTC_Video; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    // set device type to HTC
    ue->SetDeviceType(0);

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    // used by QoS-Dracon random-access scheme
    ue->GetProtocolStack()->GetMacEntity()->SetQCI(2, 4, 150, true, 4);

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

    srcPort = 0;
    dstPort = 100;

    startTime = 0.005;

    string video_trace("foreman_H264_");
    string _file(path + "src/flows/application/Trace/" + video_trace + "242k.dat");

    TraceBased *videoApp = new TraceBased();
    videoApp->SetApplicationID(applicationID);
    videoApp->SetSource(ue);
    videoApp->SetDestination(enb);
    videoApp->SetSourcePort(srcPort);
    videoApp->SetDestinationPort(dstPort);
    videoApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetStartTime(startTime / 1000);
    videoApp->SetStopTime(stopTime);

    videoApp->SetTraceFile(_file);

    QoSParameters *qos = new QoSParameters(2);

    qos->SetGBR(242);

    videoApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetClassifierParameters(cp);

    enb->SetQoSParametersforUERecord(ue->GetIDNetworkNode(), qos);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

    applicationID++;
    idUe++;
  }

  positions->clear();
  delete positions;

  simulator->SetStop(stopTime + 0.250);
  simulator->Run();
}