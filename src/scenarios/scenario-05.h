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

static void Scenario_05(int seed, int raMethod, double radius, int ulBW, int dlBW, int nUE_HTC_VoIP, int nUE_HTC_Video, int nUE_HTC_CBR, int nUE_MTC_TimeDriven) {
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
  informationManager->backoffIndicatorGeneral = 2;

  /* To Specific Backoff scheme */
  //informationManager->backoffIndicatorGeneral = backoffIndex_general;
  //informationManager->backoffIndicatorHTC = backoffIndex_HTC;
  //informationManager->backoffIndicatorMTC = backoffIndex_MTC;

  /* To ACB scheme */
  informationManager->ac_BarringFactor = 0.5;
  informationManager->ac_BarringTime = 1;

  /* To RRS scheme */
  informationManager->numberOfRA_Preambles_HTC = 20;

  /* To EAB scheme */
  informationManager->eab_MonitoringPeriod = 80;

  /* Aggregation level of PDCCH - 4 CCEs per DCI */
  informationManager->m_pdcch_format = 2;

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
  enb->GetPdcchScheduler()->SetPDCCHModeType(1);
  enb->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(4);
  enb->GetPdcchScheduler()->SetLimitUEsToFD(true);

  /**/
  enb->SetDLScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);

  /**/
  enb->SetULScheduler(ENodeB::ULScheduler_TYPE_FME);

  /**/
  ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_FME);

  /**/
  ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_FME);

  int nUEs = nUE_HTC_VoIP + nUE_HTC_Video + nUE_HTC_CBR + nUE_MTC_TimeDriven;
  int posX_ue;
  int posY_ue;

  double startTime;
  double speedDirection;

  int i;
  int idUe = 3;

  /* user equipments are distributed uniformly into a cell */
  vector<CartesianCoordinates*> *positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nUEs);

  for (i = 0; i < nUE_HTC_VoIP; i++) {
    speedDirection = GetRandomVariable(360.0) * ((2.0 * 3.14) / 360.0);

    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 3, speedDirection, cell, enb, false, Mobility::RANDOM_DIRECTION);

    // set device type to HTC
    ue->SetDeviceType(0);

    // set the access class
    //ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));
    ue->SetDeviceAccessClass((idUe - 3) % 10);

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

    //startTime = idUe % 20;

    //ue->StartRandomAccessProcedure(startTime / 1000);
    ue->StartRandomAccessProcedure(0.005);

    ue->SetSemiSchEnable(false);
    enb->SetSemiSchConfigUERecord(ue->GetIDNetworkNode(), 0, 0);

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
    ue->SetDeviceAccessClass((idUe - 3) % 10);

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

    startTime = idUe % 20;

    //ue->StartRandomAccessProcedure(startTime / 1000);
    ue->StartRandomAccessProcedure(0.005);

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
    ue->SetDeviceAccessClass((idUe - 3) % 10);

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

    startTime = idUe % 20;

    //ue->StartRandomAccessProcedure(startTime / 1000);
    ue->StartRandomAccessProcedure(0.005);

    idUe++;
  }

  //if (nUE_MTC_TimeDriven > 0) {
  //string _file(path + "src/arrival/probability-6-10.txt");
  //vector<TimeArrival*> *MTCArrivals = betaArrival->buildArrival(6, 10, nUE_MTC_TimeDriven, _file);

  //int itSlot;

  //for (itSlot = 0; itSlot < MTCArrivals->size(); itSlot++) {

  //int i;

  for (i = 0; i < nUE_MTC_TimeDriven; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    /* set device type to MTC */
    ue->SetDeviceType(1);

    /* set the access class */
    ue->SetDeviceAccessClass((idUe - 3) % 10);

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

    //startTime = MTCArrivals->at(itSlot)->getTimeArrivals();

    //ue->StartRandomAccessProcedure(startTime);
    ue->StartRandomAccessProcedure(0.005);

    idUe++;
  }

  positions->clear();
  delete positions;

  simulator->Run();
}