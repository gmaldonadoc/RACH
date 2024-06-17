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
 * Author: Tiago
 */

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

#include <cstring>

static void Simple(int seed, double radius, int ulScheduler, int nUsersToFDScheduler, int nUE_HTC_VoIP, int nUE_HTC_Video, int nUE_HTC_CBR, int nUE_MTC_TimeDriven) {
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager* networkManager = NetworkManager::Init();
  FlowsManager* flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);

  informationManager->ra_Method = 1;
  //informationManager->mac_ContentionResolutionTimer = 32;
  //informationManager->numberOfRA_Preambles = nbTotalPreambles;
  //informationManager->preambleTransMax = preambleTransMax;
  //informationManager->ra_PRACH_MaskIndex = RACHIndex;

  //informationManager->backoffIndicatorGeneral = backoffIndex_general;
  //informationManager->backoffIndicatorHTC = backoffIndex_HTC;
  //informationManager->backoffIndicatorMTC = backoffIndex_MTC;

  informationManager->ac_BarringFactor = 0.5;
  informationManager->ac_BarringTime = 1;

  //informationManager->numberOfRA_Preambles_HTC = nbHTCPreambles;

  informationManager->m_pdcch_format = 1;

  /* Create Channels */
  LteChannel *dlCh = new LteChannel();
  LteChannel *ulCh = new LteChannel();

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(5, 5, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(1, radius, 0.040, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(2, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  enb->SetPdcchScheduler();
  enb->GetPdcchScheduler()->SetLimitUEsToFD(false);
  enb->GetPdcchScheduler()->SetPDCCHModeType(2);

  /**/
  enb->SetDLScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);

  /**/
  switch (ulScheduler) {
    case 1:
      enb->SetULScheduler(ENodeB::ULScheduler_TYPE_FME);
      break;
    case 2:
      enb->SetULScheduler(ENodeB::ULScheduler_TYPE_ZBQoS);
      ((UplinkPacketScheduler *) enb->GetULScheduler())->SetnUsersFD(nUsersToFDScheduler);
      break;
  }

  int nUEs = nUE_HTC_VoIP + nUE_HTC_Video + nUE_HTC_CBR + nUE_MTC_TimeDriven;
  int idUe = 3;
  int posX_ue; //m
  int posY_ue; //m

  int applicationID = 0;
  int srcPort;
  int dstPort;
  double startTime; //s
  //double stopTime; //s

  int i;

  //user equipments are distributed uniformly into a cell
  vector<CartesianCoordinates*> *positions = GetUniformUsersDistribution(cell->GetIdCell(), nUEs);

  for (i = 0; i < nUE_HTC_VoIP; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    ue->SetDeviceType(0); // set device type to HTC

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

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
    //startTime = rndGenerator->Uniform(0.000, 0.050); //s
    startTime = 0.010;
    //duration = 9.050; //s

    VoIP *VoIPapp = new VoIP();
    VoIPapp->SetApplicationID(applicationID);
    VoIPapp->SetSource(ue);
    VoIPapp->SetDestination(enb);
    VoIPapp->SetSourcePort(srcPort);
    VoIPapp->SetDestinationPort(dstPort);
    VoIPapp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    VoIPapp->SetStartTime(startTime);
    VoIPapp->SetStopTime(20);

    QoSParameters *qos = new QoSParameters(1);
    VoIPapp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    VoIPapp->SetClassifierParameters(cp);

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_HTC_Video; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    ue->SetDeviceType(0); // set device type to HTC

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

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
    //startTime = rndGenerator->Uniform(0.000, 0.050); //s
    startTime = 0.020;

    string video_trace("foreman_H264_");
    string _file(path + "src/flows/application/Trace/" + video_trace + "128k.dat");

    TraceBased *videoApp = new TraceBased();
    videoApp->SetApplicationID(applicationID);
    videoApp->SetSource(ue);
    videoApp->SetDestination(enb);
    videoApp->SetSourcePort(srcPort);
    videoApp->SetDestinationPort(dstPort);
    videoApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetStartTime(startTime);
    videoApp->SetStopTime(20);

    videoApp->SetTraceFile(_file);

    QoSParameters *qos = new QoSParameters(2);
    videoApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    videoApp->SetClassifierParameters(cp);

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_HTC_CBR; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    // set device type to HTC
    ue->SetDeviceType(0);

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

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


    //QoSParameters *qos = new QoSParameters(6);
    srcPort = 0;
    dstPort = 100;
    //startTime = rndGenerator->Uniform(0.000, 0.050); //s
    startTime = 0.030;
    //duration = 9.050; //s

    CBR *CBRApp = new CBR();
    CBRApp->SetApplicationID(applicationID);
    CBRApp->SetSource(ue);
    CBRApp->SetDestination(enb);
    CBRApp->SetSourcePort(srcPort);
    CBRApp->SetDestinationPort(dstPort);
    CBRApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApp->SetStartTime(startTime);
    CBRApp->SetStopTime(20);

    CBRApp->SetInterval(0.008);
    CBRApp->SetSize(1000);

    QoSParameters *qos = new QoSParameters(6);
    CBRApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApp->SetClassifierParameters(cp);

    applicationID++;
    idUe++;
  }

  for (i = 0; i < nUE_MTC_TimeDriven; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    // set device type to MTC
    ue->SetDeviceType(1);

    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

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

    //QoSParameters *qos = new QoSParameters(9);
    srcPort = 0;
    dstPort = 100;
    startTime = 0.050; //s

    TimeDriven *TimeDrivenApp = new TimeDriven();
    TimeDrivenApp->SetApplicationID(applicationID);
    TimeDrivenApp->SetSource(ue);
    TimeDrivenApp->SetDestination(enb);
    TimeDrivenApp->SetSourcePort(srcPort);
    TimeDrivenApp->SetDestinationPort(dstPort);
    TimeDrivenApp->SetTransportProtocol(TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    TimeDrivenApp->SetStartTime(startTime);
    TimeDrivenApp->SetStopTime(10);

    TimeDrivenApp->SetInterval(1.000);
    TimeDrivenApp->SetSize(32);

    QoSParameters *qos = new QoSParameters(9);
    TimeDrivenApp->SetQoSParameters(qos);

    ClassifierParameters *cp = new ClassifierParameters(ue->GetIDNetworkNode(), enb->GetIDNetworkNode(), srcPort, dstPort, TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    TimeDrivenApp->SetClassifierParameters(cp);

    applicationID++;
    idUe++;
  }

  //positions->clear();
  //delete positions;

  simulator->SetStop(20.000);
  simulator->Run();
}