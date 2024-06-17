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

static void Scenario_02(int seed, int raMethod, double radius, int ulBW, int dlBW, double acbBarringFactor, double acbBarringTime, int nHTC, int nMTC) {
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager* networkManager = NetworkManager::Init();
  FlowsManager* flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);

  /* RACH Configurarion*/
  informationManager->ra_Method = raMethod;
  informationManager->maxHarq_Msg3Tx = 5;
  informationManager->mac_ContentionResolutionTimer = 48;
  informationManager->numberOfRA_Preambles = 52;
  informationManager->preambleTransMax = 10;
  informationManager->ra_PRACH_MaskIndex = 6;
  informationManager->backoffIndicatorGeneral = 2;

  /* To Especific Backoff scheme */
  //informationManager->backoffIndicatorHTC = backoffIndex_HTC;
  //informationManager->backoffIndicatorMTC = backoffIndex_MTC;

  /* To ACB scheme */
  informationManager->ac_BarringFactor = acbBarringFactor;
  informationManager->ac_BarringTime = acbBarringTime;

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
  Cell *cell = networkManager->CreateCell(1, radius, 0.050, 0, 0);

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

  int posX_ue;
  int posY_ue;

  int i;
  int idUe = 3;
  int nUEs = nHTC + nMTC;

  /* user equipments are distributed uniformly into a cell */
  vector<CartesianCoordinates*> *positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nUEs);

  for (i = 0; i < nHTC; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    // set device type to HTC or MTC
    ue->SetDeviceType(0);

    // set the access class
    ue->SetDeviceAccessClass((idUe - 3) % 10);

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(8, 8, 300, false, 6);

    ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    // register UE to the eNodeB
    enb->RegisterUserEquipment(ue);

    ue->SetSemiSchEnable(false);

    ue->SchedulingRequestConfiguration(false, 0);

    // define the channel realization
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    ue->StartRandomAccessProcedure(0.005);

    idUe++;
  }

  for (i = 0; i < nMTC; i++) {
    posX_ue = positions->at(idUe - 3)->GetCoordinateX();
    posY_ue = positions->at(idUe - 3)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUe, posX_ue, posY_ue, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    // set device type to HTC or MTC
    ue->SetDeviceType(1);

    // set the access class
    ue->SetDeviceAccessClass((idUe - 3) % 10);

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(9, 9, 300, false, 6);

    ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    // register UE to the eNodeB
    enb->RegisterUserEquipment(ue);

    ue->SetSemiSchEnable(false);

    ue->SchedulingRequestConfiguration(false, 0);

    // define the channel realization
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    ue->StartRandomAccessProcedure(0.005);

    idUe++;
  }

  positions->clear();
  delete positions;

  simulator->Run();
}