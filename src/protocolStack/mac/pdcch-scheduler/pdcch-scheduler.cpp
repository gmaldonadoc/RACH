/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015
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

#include <algorithm>

#include "pdcch-scheduler.h"

#include "../../../phy/lte-phy.h"
#include "../../../device/NetworkNode.h"
#include "../../../componentManagers/InformationManager.h"
#include "../../../core/idealMessages/ideal-control-messages.h"
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../flows/radio-bearer.h"
#include "../../../device/ENodeB.h"
#include "../../../device/UserEquipment.h"
#include "../../../componentManagers/FrameManager.h"
#include "../../../load-parameters.h"
#include "../../../flows/application/WEB.h"
#include "../AMCModule.h"
#include "../../../drx/drx-manager.h"

PdcchScheduler::PdcchScheduler() {
  m_limitUEsToFD = false;
  m_mac = NULL;
  m_modeType = 1;

  m_DCIsToSchedule = NULL;

  m_GBRDCIsToSchedule = NULL;
  m_NGBRDCIsToSchedule = NULL;

  m_MSGs4ToSchedule = NULL;

  m_HighPriorityMSGs2ToSchedule = NULL;
  m_LowPriorityMSGs2ToSchedule = NULL;

  m_maxGrantsPerRAR = 3; // # grants for each RAR
  m_maxMSGs4ToSchedule = 4;
  m_maxBSRGrantsToSchedule = 4;

  UEsToDL = NULL;
}

PdcchScheduler::~PdcchScheduler() {
  m_mac = NULL;
}

void
PdcchScheduler::SetMacEntity(MacEntity* mac) {
  m_mac = mac;
}

MacEntity*
PdcchScheduler::GetMacEntity(void) {
  return m_mac;
}

void
PdcchScheduler::StartSchedule(void) {
#ifdef PDCCH_DEBUG
  std::cerr << Simulator::Init()->Now() << " PDCCH - Start Scheduler" << std::endl;
#endif

  ((EnbMacEntity*) m_mac)->CheckForDelayedMSGs2OfRandomAccess();
  ((EnbMacEntity*) m_mac)->CheckForDelayedMSGs4OfRandomAccess();

  m_nUlUEsToSchedule = 0;
  m_nDlUEsToSchedule = 0;
  m_nMsg2ToSchedule = 0;
  m_nMsg4ToSchedule = 0;
  m_nBsrGrantsToSchedule = 0;

  m_nUlReservedCces = 0;
  m_nDlReservedCces = 0;
  m_nMsg2ReservedCces = 0;
  m_nMsg4ReservedCces = 0;
  m_nBsrGrantsReservedCces = 0;

  uDl = 0;
  m_current_pdcch_cces = m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces();

  for (int i = 0; i < 84; i++) {
    PDCCHResources[i] = -2;
    for (int j = 0; j < 200; j++) {
      PDCCHCcesPerUser[j][i] = 0;
    }
  }

  if ((UPLINK == true) && (DOWNLINK == false)) {
    GenerateUserEquipmentDL(nUEsToDL);
  }
}

void
PdcchScheduler::Schedule(void) {
  DoSchedule();
}

void
PdcchScheduler::DoSchedule(void) {
#ifdef PDCCH_DEBUG
  std::cerr << Simulator::Init()->Now() << " PDCCH - Scheduling" << std::endl;
#endif

  if (USE_PDCCH_MANAGER == true) {
    if (m_modeType == 1) {
      // RACH-Priorized (RAP) Algorithm
      RAP_Scheduling();

    } else if (m_modeType == 2) {
      // Lifetime-Aware (LTA) Algorithm
      //LTA_Scheduling();
      MinAL_Algorithm();
    } else if (m_modeType == 3) {
      // GBR-Priorized Lifetime-Aware (GBR-LTA) Algorithm
      GBRLTA_Scheduling();
    } else if (m_modeType == 4) {
      NMScheduling1();
    } else if (m_modeType == 5) {
      // Preamble Priority-Aware Control Resource Allocation (PPA) Algorithm
      PPA_Scheduling();
    } else if (m_modeType == 6) {
      // Enhanced Preamble Priority-Aware Control Resource Allocation (ePPA) Algorithm
      PPA_Scheduling();
    }
  }
}

void
PdcchScheduler::StopSchedule() {
  DoStopSchedule();
}

void
PdcchScheduler::DoStopSchedule() {
  //std::cerr << Simulator::Init()->Now() << " PDCCH TTI " << FrameManager::Init()->GetTTICounter() << " Max-CCEs " << m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces() << " Used-CCEs " << m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces() - m_current_pdcch_cces << " Available-CCEs " << m_current_pdcch_cces << " Used-To-Msg2 " << m_nMsg2ReservedCces << " Used-To-Msg4 " << m_nMsg4ReservedCces << " Used-To-UL " << m_nUlReservedCces << " Used-To-DL " << m_nDlReservedCces << " Used-To-Bsr " << m_nBsrGrantsReservedCces << std::endl;

#ifdef PDCCH_DEBUG
  std::cerr << "[DEBUG] PDCCH Scheduler"
          << "\n\t MSG2 = " << m_nMsg2ToSchedule
          << "\n\t MSG4 = " << m_nMsg4ToSchedule
          << "\n\t BSR = " << m_nBsrGrantsToSchedule
          << "\n\t UL = " << m_nUlUEsToSchedule
          << "\n\t DL = " << m_nDlUEsToSchedule;

  if (((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUplinkTDScheduler() != NULL) {
    UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();
    std::cerr << "\n\t Users Scheduled to UL:";

    if (usersToSchedule) {
      for (int u = 0; u < usersToSchedule->size(); u++) {
        if (usersToSchedule->at(u)->m_allowToSchedule == true) {
          std::cerr << " " << usersToSchedule->at(u)->m_userToSchedule->GetIDNetworkNode();
        }
      }
    }
  }

  std::cerr << std::endl;
#endif

  /**/
  ((EnbMacEntity*) m_mac)->CleanRandomAccessMessagensContainer();
  ((EnbMacEntity*) m_mac)->CleanRandomAccessMessagesQueue();

  /**/
  DeleteDCIsToSchedule();
  DeleteMSGs2ToSchedule();
  DeleteMSGs4ToSchedule();
}

int
PdcchScheduler::GetCurrentCCEsToPDCCH() {
  return m_current_pdcch_cces;
}

int
PdcchScheduler::GetMaxUEsToScheduleInUL() {
  return m_nUlUEsToSchedule;
}

double PdcchScheduler::ComputeRandomAccessMsg2TDSchedulingMetric(EnbMacEntity::MsgToSchedule * msg2) {
  double metric = ((Simulator::Init()->Now() - msg2->m_timeArrived) * 1000) / (double) (InformationManager::Init()->ra_ResponseWindowSize - 1);
  return metric;
}

double
PdcchScheduler::ComputeRandomAccessMsg4TDSchedulingMetric(EnbMacEntity::MsgToSchedule * msg4) {
  double metric = ((Simulator::Init()->Now() - msg4->m_timeArrived) * 1000) / (double) InformationManager::Init()->mac_ContentionResolutionTimer;
  return metric;
}

void
PdcchScheduler::Msg2NormalPriorityScheduling() {
#ifdef PDCCH_DEBUG
  std::cerr << Simulator::Init()->Now() << "PDCCH - Scheduling Msg2" << std::endl;
#endif

  int qtd = 0, pdcchFormat;
  bool canSch;

  pdcchFormat = 3;

  if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {

    std::random_shuffle(((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->begin(), ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->end());

    EnbMacEntity::MSGsToSchedule::iterator iter;

    RAResponseIdealControlMessage *RandomAccessResponse = NULL;

    for (iter = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->end(); iter++) {
      if ((*iter) != NULL) {
        if (!(*iter)->m_scheduled) {

          /* --- [DRX] --- */
          canSch = true;
          DRXManager *drxMan = ((UserEquipment *) (*iter)->m_destination)->GetDRXManager();
          if (drxMan != NULL) {
            DRXManager::State drxState = drxMan->getStatus();
            if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
              if (drxMan->GetExpectedOffTime() > 1)//If GetExpectedOffTime() é igual a 1, no pŕoximo TTI o UE escutará o PDCCH, assim ele poderá ser escalonado para o próximo TTI
                canSch = false;
            } else if (drxState == DRXManager::DEEP) {
              canSch = false;
            }
          }
          /* --- */

          if (canSch) {
            if (Simulator::Init()->Now() - (*iter)->m_timeArrived < InformationManager::Init()->ra_ResponseWindowSize) {

              // Make the PDCCH Allocation on the Common Search Space
              if (PDCCHResourcesAllocation(-1) == false) {
                std::cout << "MSG2 false" << std::endl;
                return;
              }

              RandomAccessResponse = new RAResponseIdealControlMessage((*iter)->m_RARNTI, InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
              RandomAccessResponse->SetSourceDevice(m_mac->GetDevice());

              m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

              m_nMsg2ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
              m_nMsg2ToSchedule++;

              Simulator::Init()->Schedule(0.000, &EnbMacEntity::SendRandomAccessResponseIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessResponse);

              break;
            }
          }

        }
      }
    }

    if (RandomAccessResponse != NULL) {
      for (iter = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->end(); iter++) {
        if ((*iter) != NULL) {
          if ((!(*iter)->m_scheduled) && ((*iter)->m_RARNTI == RandomAccessResponse->getRARNTI()) && (qtd < m_maxGrantsPerRAR)) {
            if (Simulator::Init()->Now() - (*iter)->m_timeArrived < InformationManager::Init()->ra_ResponseWindowSize) {
              /* --- [DRX] --- */
              canSch = true;
              DRXManager *drxMan = ((UserEquipment *) (*iter)->m_destination)->GetDRXManager();
              if (drxMan != NULL) {
                DRXManager::State drxState = drxMan->getStatus();
                if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
                  if (drxMan->GetExpectedOffTime() > 1)//If GetExpectedOffTime() é igual a 1, no pŕoximo TTI o UE escutará o PDCCH, assim ele poderá ser escalonado para o próximo TTI
                    canSch = false;
                } else if (drxState == DRXManager::DEEP) {
                  canSch = false;
                }
              }
              /* --- */
              if (canSch) {
                (*iter)->m_scheduled = true;
                RandomAccessResponse->AddNewRecord((*iter)->m_preamble);
                ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission(1);
                qtd++;
              }
            } else {
              (*iter)->m_scheduled = true;
            }
          }
        }
      }
    }
  }
}

void
PdcchScheduler::Msg4NormalPriorityScheduling() {
  int qtd = 0, pdcchFormat;
  bool canSch = true;

  std::random_shuffle(((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->begin(), ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->end());

  EnbMacEntity::MSGsToSchedule::iterator iter;

  for (iter = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->end(); iter++) {
    /* --- [DRX] --- */
    DRXManager *drxMan = ((UserEquipment *) (*iter)->m_destination)->GetDRXManager();
    if (drxMan != NULL) {
      DRXManager::State drxState = drxMan->getStatus();
      if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
        if (drxMan->GetExpectedOffTime() > 1)//If GetExpectedOffTime() é igual a 1, no pŕoximo TTI o UE escutará o PDCCH, assim ele poderá ser escalonado para o próximo TTI
          canSch = false;
      } else if (drxState == DRXManager::DEEP) {
        canSch = false;
      }
    }
    /* --- */

    if (canSch) {
      if (UPLINK == true) {
        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation((*iter)->m_destination->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }
      } else {
        pdcchFormat = 2;
      }

      if ((m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) && (qtd < m_maxMSGs4ToSchedule)) {
        if (PDCCHResourcesAllocation((*iter)->m_destination->GetIDNetworkNode())) {
          qtd++;
          m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
          (*iter)->m_scheduled = true;

          m_nMsg4ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          m_nMsg4ToSchedule++;

          Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyContentionResolutionIdealControlMessage, ((EnbMacEntity*) m_mac), (*iter)->m_destination);
        }
      }
    }
  }
}

void
PdcchScheduler::BsrGrantNormalPriorityScheduling() {
  int qtd = 0, pdcchFormat;
  bool canSch = true;

  std::random_shuffle(((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->begin(), ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->end());

  EnbMacEntity::MSGsToSchedule::iterator iter;

  for (iter = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->end(); iter++) {

    /* --- [DRX] --- */
    DRXManager *drxMan = ((UserEquipment *) (*iter)->m_destination)->GetDRXManager();
    if (drxMan != NULL) {
      DRXManager::State drxState = drxMan->getStatus();
      if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
        if (drxMan->GetExpectedOffTime() > 1)//If GetExpectedOffTime() é igual a 1, no pŕoximo TTI o UE escutará o PDCCH, assim ele poderá ser escalonado para o próximo TTI
          canSch = false;
      } else if (drxState == DRXManager::DEEP) {
        canSch = false;
      }
    }
    /* --- */

    if (canSch) {
      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation((*iter)->m_destination->GetIDNetworkNode());
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }

      if ((m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) && (qtd < m_maxBSRGrantsToSchedule)) {
        if (PDCCHResourcesAllocation((*iter)->m_destination->GetIDNetworkNode())) {
          qtd++;
          //m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
          m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
          (*iter)->m_scheduled = true;

          m_nBsrGrantsReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          m_nBsrGrantsToSchedule++;

          //Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyBsrGrantIdealControlMessage, ((EnbMacEntity*) m_mac), (*iter)->m_destination);

          ((EnbMacEntity*) m_mac)->AssemblyBsrGrantIdealControlMessage((*iter)->m_destination);
        }
      }
    }
  }
}

double
PdcchScheduler::ComputeULUserTDSchedulingMetric(UplinkPacketScheduler::UserToSchedule * user) {
  double metric = user->real_HoL / user->m_maximumDelay;
  return metric;
}

double
PdcchScheduler::ComputeDLFlowTDSchedulingMetric(DownlinkPacketScheduler::FlowToSchedule *flow) {
  double metric = flow->m_realHoL / flow->m_maximumDelay;
  return metric;
}

void
PdcchScheduler::LTA_Scheduling() {
  int pdcchFormat, qtdMsg4 = 0, m_UEsScheduled_temp = 0, m_UEsScheduled = 0;
  bool msg2Allocated = false;

  uDl = 0;

  SelectDCIsToSchedule();

  OrderDCIsByMetric(GetDCIsToSchedule());

  DCIsToSchedule *DCIsToSchedule = GetDCIsToSchedule();

  UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

  DownlinkPacketScheduler::FlowsToSchedule *flowsToSchedule = ((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetFlowsToSchedule();

  /*-----*/
  /*int maxCCEs, RNTI, cceIndex;

  //std::cout << "DCI to Schedule = " << DCIsToSchedule->size() << std::endl;

  for (int d = 0; d < DCIsToSchedule->size(); d++) {
    //std::cout << "Vector Position = " << DCIsToSchedule->at(d)->m_vectorPosition << std::endl;
    if (DCIsToSchedule->at(d)->m_type == 0) {
      RNTI = usersToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(d)->m_type == 1) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);
      RNTI = -1;
    } else if (DCIsToSchedule->at(d)->m_type == 2) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);
      RNTI = RandomAccessMsg4->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(d)->m_type == 3) {
      EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);
      RNTI = BSRGrant->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(d)->m_type == 4) {
      RNTI = flowsToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode();
    }

    if (RNTI == -1) {
      pdcchFormat = 3;
    } else {
      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(RNTI);
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }
    }

    maxCCEs = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

    for (int m = 0; m < InformationManager::Init()->GetPDCCHNumberOfCandidates_UESpecific(pdcchFormat); m++) {
      for (int i = 0; i < maxCCEs; i++) {
        cceIndex = Calc_CCE_Index(m, i, RNTI);
        PDCCHCcesPerUser[d][cceIndex] = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
      }
    }
  }

#ifdef PDCCH_DEBUG
  std::cerr << "[DEBUG]" << std::endl;
  for (int i = 0; i < DCIsToSchedule->size(); i++) {
    if (DCIsToSchedule->at(i)->m_type == 0) {
      RNTI = usersToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 1) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = -1;
    } else if (DCIsToSchedule->at(i)->m_type == 2) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = RandomAccessMsg4->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 3) {
      EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = BSRGrant->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 4) {
      RNTI = flowsToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode();
    }

    if (RNTI == -1) {
      pdcchFormat = 3;
    } else {
      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(RNTI);
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }
    }

    std::cerr << "\t " << RNTI
            << "\t[" << InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)
            << ", " << InformationManager::Init()->GetPDCCHNumberOfCandidates_UESpecific(pdcchFormat)
            << "]:";
    for (int j = 0; j < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces(); j++) {
      std::cerr << " " << PDCCHCcesPerUser[i][j];
    }
    std::cerr << std::endl;
  }
#endif

  int PDCCHResources_temp[84], total, where;

  for (int i = 0; i < 84; i++) {
    PDCCHResources_temp[i] = -2;
  }

  for (int i = 0; i < DCIsToSchedule->size(); i++) {
    if (DCIsToSchedule->at(i)->m_type == 0) {
      RNTI = usersToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 1) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = -1;
    } else if (DCIsToSchedule->at(i)->m_type == 2) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = RandomAccessMsg4->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 3) {
      EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = BSRGrant->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 4) {
      RNTI = flowsToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode();
    }

    if (RNTI == -1) {
      pdcchFormat = 3;
    } else {
      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(RNTI);
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }
    }

    int h = 1;

    //std::cout << "PDCCH UE " << i << std::endl;

    where = -1;
    total = 100000;

    for (int j = 0; j < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces(); j += h) {
      h = 1;
      if (PDCCHCcesPerUser[i][j] != 0) {
        int soma = 0;
        int first = j;

        for (int k = 0; k < InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat); k++) {
          if (PDCCHResources_temp[j + k] == -2) {
            for (int p = i + 1; p < DCIsToSchedule->size(); p++) {
              soma += PDCCHCcesPerUser[p][j + k];
            }

            h = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          } else {
            soma = 1000000;
            h = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            break;
          }
        }

        if (soma < total) {
          where = first;
          total = soma;
        }
      }
    }

    //std::cout << "where " << where << std::endl;
    if (where != -1) {
      m_UEsScheduled_temp++;
      for (int k = 0; k < InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat); k++) {
        PDCCHResources_temp[where + k] = RNTI;
      }
    }
  }

#ifdef PDCCH_DEBUG
  std::cerr << "[DEBUG]" << std::endl;
  std::cerr << "\tPDCCH TEMP [";
  for (int j = 0; j < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces(); j++) {
    std::cerr << " " << PDCCHResources_temp[j];
  }
  std::cerr << " ]" << std::endl;
  std::cerr << "\t# UEs Scheduled in TEMP = " << m_UEsScheduled_temp << std::endl;
#endif*/

  /*----*/

  /*for (int d = 0; d < DCIsToSchedule->size(); d++) {
    if (m_current_pdcch_cces > 0) {

      if (DCIsToSchedule->at(d)->m_type == 0) {

        if (false) {
          if (InformationManager::Init()->m_pdcch_format == -1) {
            pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
          } else {
            pdcchFormat = InformationManager::Init()->m_pdcch_format;
          }

          if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
            // escalona DL
            if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
              m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

              m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
              m_nDlUEsToSchedule++;

              m_UEsScheduled++;
            } else {
#ifdef PDCCH_DEBUG
              std::cout << "DL false" << std::endl;
#endif
            }
          }
        }

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(usersToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona UL
          if (PDCCHResourcesAllocation(usersToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode())) {
            usersToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_allowToSchedule = true;
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;

            m_UEsScheduled++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "UL false" << std::endl;
#endif
          }
        }

      } else if (DCIsToSchedule->at(d)->m_type == 1) {
        if ((m_current_pdcch_cces >= 8) && (!msg2Allocated)) {

          if (PDCCHResourcesAllocation(-1) == true) {
            int qtd = 0;

            EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);

            RAResponseIdealControlMessage *RandomAccessResponse = NULL;
            RandomAccessResponse = new RAResponseIdealControlMessage(RandomAccessMsg2->m_RARNTI, InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
            RandomAccessResponse->SetSourceDevice(m_mac->GetDevice());

            m_current_pdcch_cces -= 8; // RAR message always uses 8 CCEs

            Simulator::Init()->Schedule(0.000, &EnbMacEntity::SendRandomAccessResponseIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessResponse);

            if (RandomAccessResponse != NULL) {
              EnbMacEntity::MSGsToSchedule::iterator iter;
              for (iter = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->end(); iter++) {
                if ((*iter) != NULL) {
                  if ((!(*iter)->m_scheduled) && ((*iter)->m_RARNTI == RandomAccessResponse->getRARNTI()) && (qtd < m_maxGrantsPerRAR)) {
                    (*iter)->m_scheduled = true;
                    RandomAccessResponse->AddNewRecord((*iter)->m_preamble);
                    ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
                    qtd++;
                  }
                }
              }
            }

            msg2Allocated = true;

            m_nMsg2ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(3);
            m_nMsg2ToSchedule++;

            m_UEsScheduled++;
          }
        }

      } else if (DCIsToSchedule->at(d)->m_type == 2) {
        EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(RandomAccessMsg4->m_destination->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat) && (qtdMsg4 < m_maxMSGs4ToSchedule)) {
          if (PDCCHResourcesAllocation(RandomAccessMsg4->m_destination->GetIDNetworkNode()) == true) {

            RandomAccessMsg4->m_scheduled = true;

            ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyContentionResolutionIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessMsg4->m_destination);

            qtdMsg4++;

            m_nMsg4ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nMsg4ToSchedule++;

            m_UEsScheduled++;
          }
        }
      } else if (DCIsToSchedule->at(d)->m_type == 3) {
        EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(BSRGrant->m_destination->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          if (PDCCHResourcesAllocation(BSRGrant->m_destination->GetIDNetworkNode()) == true) {

            BSRGrant->m_scheduled = true;

            ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyBsrGrantIdealControlMessage, ((EnbMacEntity*) m_mac), BSRGrant->m_destination);

            m_nBsrGrantsReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nBsrGrantsToSchedule++;

            m_UEsScheduled++;
          }
        }
      } else if (DCIsToSchedule->at(d)->m_type == 4) {
        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(flowsToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }
        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona DL
          if (PDCCHResourcesAllocation(flowsToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode())) {
            flowsToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_allowToSchedule = true;
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;

            m_UEsScheduled++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "DL false" << std::endl;
#endif
          }
        }

      }
    } else {

      break;
    }
  }*/

  //BestEffortCCEsScheduling(DCIsToSchedule);

  NewCCEsScheduling(DCIsToSchedule);
}

void
PdcchScheduler::SelectDCIsToSchedule() {
  int pdcchFormat;

  m_DCIsToSchedule = new DCIsToSchedule;

  /* Select the UL grants */
  if (((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUplinkTDScheduler() != NULL) {
    UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();
    for (int u = 0; u < usersToSchedule->size(); u++) {
      DCIToSchedule *dci = new DCIToSchedule();
      dci->m_type = 0;
      dci->m_vectorPosition = u;
      dci->m_metric = ComputeULUserTDSchedulingMetric(usersToSchedule->at(u));

      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(usersToSchedule->at(u)->m_userToSchedule->GetIDNetworkNode());
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }

      dci->m_aggregationLevel = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

      GetDCIsToSchedule()->push_back(dci);
    }
  }

  /* Select the DL grants */
  if (((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetDownlinkTDScheduler() != NULL) {
    DownlinkPacketScheduler::FlowsToSchedule *flowsToSchedule = ((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetFlowsToSchedule();
    for (int f = 0; f < flowsToSchedule->size(); f++) {
      DCIToSchedule *dci = new DCIToSchedule();
      dci->m_type = 4;
      dci->m_vectorPosition = f;
      dci->m_metric = ComputeDLFlowTDSchedulingMetric(flowsToSchedule->at(f));

      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(flowsToSchedule->at(f)->m_bearer->GetDestination()->GetIDNetworkNode());
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }

      dci->m_aggregationLevel = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

      GetDCIsToSchedule()->push_back(dci);
    }
  }

  /* Select the RAR messages */
  if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->size() > 0) {
    EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(0);
    DCIToSchedule *dci = new DCIToSchedule();
    dci->m_type = 1;
    dci->m_vectorPosition = 0;
    dci->m_metric = ComputeRandomAccessMsg2TDSchedulingMetric(RandomAccessMsg2);

    dci->m_aggregationLevel = InformationManager::Init()->GetPDCCHNumberOfCCEs(3);

    GetDCIsToSchedule()->push_back(dci);
  }

  /* Select the Contention-resolution messages */
  for (int i = 0; i < m_maxMSGs4ToSchedule; i++) {
    if (((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->size() > i) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(i);
      DCIToSchedule *dci = new DCIToSchedule();
      dci->m_type = 2;
      dci->m_vectorPosition = i;
      dci->m_metric = ComputeRandomAccessMsg4TDSchedulingMetric(RandomAccessMsg4);

      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(RandomAccessMsg4->m_destination->GetIDNetworkNode());
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }

      dci->m_aggregationLevel = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

      GetDCIsToSchedule()->push_back(dci);
    }
  }

  /* Select the BSR grants from SR */
  for (int i = 0; i < m_maxBSRGrantsToSchedule; i++) {
    if (((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->size() > i) {
      EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(i);
      DCIToSchedule *dci = new DCIToSchedule();
      dci->m_type = 3;
      dci->m_vectorPosition = i;
      dci->m_metric = ComputeBSRGrantTDSchedulingMetric(BSRGrant);

      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(BSRGrant->m_destination->GetIDNetworkNode());
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }

      dci->m_aggregationLevel = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

      GetDCIsToSchedule()->push_back(dci);
    }
  }
}

void
PdcchScheduler::SelectMSG2sToSchedule() {
  m_HighPriorityMSGs2ToSchedule = new MSGs2ToSchedule;
  m_LowPriorityMSGs2ToSchedule = new MSGs2ToSchedule;

  for (int i = 0; i < ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->size(); i++) {
    if (!((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(i)->m_scheduled) {
      if (Simulator::Init()->Now() - ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(i)->m_timeArrived < InformationManager::Init()->ra_ResponseWindowSize) {
        MSG2ToSchedule *msg2 = new MSG2ToSchedule();
        msg2->m_vectorPosition = i;

        if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(i)->m_preamble >= InformationManager::Init()->numberOfRA_Preambles) {
          //std::cout << "PDCCH - High Priority Selected" << std::endl;
          GetHighPriorityMSGs2ToSchedule()->push_back(msg2);
        } else {
          //std::cout << "PDCCH - Low Priority Selected" << std::endl;

          if (InformationManager::Init()->ra_Method == 5) {
            if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(i)->m_preamble < InformationManager::Init()->numberOfRA_Preambles_HTC) {
              //std::cout << "PDCCH - Low Priority Selected HTC" << std::endl;
              msg2->m_metric = 0;
            } else {
              //std::cout << "PDCCH - Low Priority Selected MTC" << std::endl;
              msg2->m_metric = 1;
            }
          } else {

            msg2->m_metric = 0;
          }

          GetLowPriorityMSGs2ToSchedule()->push_back(msg2);
        }
      }
    }
  }
}

PdcchScheduler::DCIsToSchedule *
PdcchScheduler::GetDCIsToSchedule(void) {
  return m_DCIsToSchedule;
}

PdcchScheduler::DCIsToSchedule *
PdcchScheduler::GetGBRDCIsToSchedule(void) {
  return m_GBRDCIsToSchedule;
}

PdcchScheduler::DCIsToSchedule *
PdcchScheduler::GetNGBRDCIsToSchedule(void) {
  return m_NGBRDCIsToSchedule;
}

PdcchScheduler::MSGs2ToSchedule *
PdcchScheduler::GetHighPriorityMSGs2ToSchedule(void) {
  return m_HighPriorityMSGs2ToSchedule;
}

PdcchScheduler::MSGs2ToSchedule *
PdcchScheduler::GetLowPriorityMSGs2ToSchedule(void) {
  return m_LowPriorityMSGs2ToSchedule;
}

PdcchScheduler::MSGs4ToSchedule *
PdcchScheduler::GetMSGs4ToSchedule(void) {
  return m_MSGs4ToSchedule;
}

void
PdcchScheduler::OrderDCIsByMetric(DCIsToSchedule *DCIsToSchedule) {
  int i, j;

  DCIToSchedule *aux;

  for (i = 1; i < DCIsToSchedule->size(); i++) {
    aux = DCIsToSchedule->at(i);
    j = i;
    while (j > 0 && DCIsToSchedule->at(j - 1)->m_metric < aux->m_metric) {
      DCIsToSchedule->at(j) = DCIsToSchedule->at(j - 1);
      j = j - 1;
    }
    DCIsToSchedule->at(j) = aux;
  }
}

void
PdcchScheduler::SetnUEsToDL(int nUEs) {
  nUEsToDL = nUEs;
}

void
PdcchScheduler::SetLimitUEsToFD(bool value) {
  m_limitUEsToFD = value;
}

bool
PdcchScheduler::GetLimitUEsToFD() {
  return m_limitUEsToFD;
}

void
PdcchScheduler::RAP_Scheduling() {
  int pdcchFormat, current_pdcchCCEs_downlink = 0, current_pdcchCCEs_uplink = 0, current_pdcchCCEs_last = 0;

  uDl = 0;

  /* Select the RAR message */
  Msg2NormalPriorityScheduling();

  /* Select the Contention-resolution message */
  Msg4NormalPriorityScheduling();

  /* Select the BSR grants (from SR) */
  BsrGrantNormalPriorityScheduling();

  while (m_current_pdcch_cces > 0) {

    current_pdcchCCEs_last = m_current_pdcch_cces;

    if (UPLINK == true && DOWNLINK == true) {
      current_pdcchCCEs_downlink = ceil(m_current_pdcch_cces / 2);
      current_pdcchCCEs_uplink = floor(m_current_pdcch_cces / 2);
    }

    if (UPLINK == true && DOWNLINK == false) {
      current_pdcchCCEs_uplink = m_current_pdcch_cces;
    }

    if (UPLINK == false && DOWNLINK == true) {
      current_pdcchCCEs_downlink = m_current_pdcch_cces;
    }

    /* Select the downlink flows */
    if (((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetDownlinkTDScheduler() != NULL) {
      DownlinkPacketScheduler::FlowsToSchedule *flowsToSchedule = ((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetFlowsToSchedule();

      for (int f = 0; f < flowsToSchedule->size(); f++) {

        if (flowsToSchedule->at(f)->m_allowToSchedule == true) {
          continue;
        }

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(flowsToSchedule->at(f)->m_bearer->GetDestination()->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (current_pdcchCCEs_downlink >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona DL
          if (PDCCHResourcesAllocation(flowsToSchedule->at(f)->m_bearer->GetDestination()->GetIDNetworkNode())) {
            flowsToSchedule->at(f)->m_allowToSchedule = true;

            current_pdcchCCEs_downlink -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nDlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "DL false" << std::endl;
#endif
          }
        }
      }
    }

    /* Select the uplink users */
    if (((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUplinkTDScheduler() != NULL) {
      UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

      for (int u = 0; u < usersToSchedule->size(); u++) {

        /*if (DOWNLINK == false) {
          if (InformationManager::Init()->m_pdcch_format == -1) {
            pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
          } else {
            pdcchFormat = InformationManager::Init()->m_pdcch_format;
          }

          if (current_pdcchCCEs_uplink >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
             escalona DL
            if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
              current_pdcchCCEs_uplink -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
              m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

              m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
              m_nDlUEsToSchedule++;
            } else {
#ifdef PDCCH_DEBUG
              std::cout << "DL false" << std::endl;
#endif
            }
          }
        }*/

        if (usersToSchedule->at(u)->m_allowToSchedule == true) {
          continue;
        }

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(usersToSchedule->at(u)->m_userToSchedule->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (current_pdcchCCEs_uplink >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona UL
          if (PDCCHResourcesAllocation(usersToSchedule->at(u)->m_userToSchedule->GetIDNetworkNode())) {
            usersToSchedule->at(u)->m_allowToSchedule = true;
            current_pdcchCCEs_uplink -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "UL false" << std::endl;
#endif
          }
        }
      }
    }

    if (m_current_pdcch_cces == current_pdcchCCEs_last) {
      break;
    }
  }
}

void
PdcchScheduler::NMScheduling1() {
  UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

  for (int u = 0; u < usersToSchedule->size(); u++) {
    if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format)) {
      if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format)) {
        // escalona DL
        m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
        m_nDlUEsToSchedule++;
      }
      if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format)) {
        // escalona UL
        usersToSchedule->at(u)->m_allowToSchedule = true;
        m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
        m_nUlUEsToSchedule++;
      }
    } else {

      break;
    }
  }
}

void
PdcchScheduler::SetPDCCHModeType(int type) {

  m_modeType = type;
}

void
PdcchScheduler::GBRLTA_Scheduling() {
  int pdcchFormat;

  uDl = 0;

  SelectDCIsToSchedule1();

  OrderDCIsByMetric(GetGBRDCIsToSchedule());
  OrderDCIsByMetric(GetNGBRDCIsToSchedule());

  DCIsToSchedule *GBRDCIsToSchedule = GetGBRDCIsToSchedule();
  DCIsToSchedule *NGBRDCIsToSchedule = GetNGBRDCIsToSchedule();

  //UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

  //BestEffortCCEsScheduling(GBRDCIsToSchedule);
  //BestEffortCCEsScheduling(NGBRDCIsToSchedule);

  NewCCEsScheduling(GBRDCIsToSchedule);
  NewCCEsScheduling(NGBRDCIsToSchedule);

  /*for (int d = 0; d < GBRDCIsToSchedule->size(); d++) {
    if (m_current_pdcch_cces > 0) {
      if (GBRDCIsToSchedule->at(d)->m_type == 0) {
        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        //pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona DL
          if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nDlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "DL false" << std::endl;
#endif
          }
        }

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(usersToSchedule->at(GBRDCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }
        //pdcchFormat = LinkAdaptation(usersToSchedule->at(GBRDCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode());
        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona UL
          if (PDCCHResourcesAllocation(usersToSchedule->at(GBRDCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode()) == true) {
            usersToSchedule->at(GBRDCIsToSchedule->at(d)->m_vectorPosition)->m_allowToSchedule = true;
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "UL false" << std::endl;
#endif
          }
        }

      } else if (GBRDCIsToSchedule->at(d)->m_type == 1) {
        if (m_current_pdcch_cces >= 8) {
          if (PDCCHResourcesAllocation(-1) == true) {

            int qtd = 0;

            EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(GBRDCIsToSchedule->at(d)->m_vectorPosition);

            RAResponseIdealControlMessage *RandomAccessResponse = NULL;
            RandomAccessResponse = new RAResponseIdealControlMessage(RandomAccessMsg2->m_RARNTI, InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
            RandomAccessResponse->SetSourceDevice(m_mac->GetDevice());

            //m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
            m_current_pdcch_cces -= 8;

            Simulator::Init()->Schedule(0.000, &EnbMacEntity::SendRandomAccessResponseIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessResponse);

            if (RandomAccessResponse != NULL) {
              EnbMacEntity::MSGsToSchedule::iterator iter;
              for (iter = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->end(); iter++) {
                if ((*iter) != NULL) {
                  if ((!(*iter)->m_scheduled) && ((*iter)->m_RARNTI == RandomAccessResponse->getRARNTI()) && (qtd < m_maxGrantsPerRAR)) {
                    (*iter)->m_scheduled = true;
                    RandomAccessResponse->AddNewRecord((*iter)->m_preamble);
                    ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
                    qtd++;
                  }
                }
              }
            }

            m_nMsg2ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(3);
            m_nMsg2ToSchedule++;
          }
        }

      } else if (GBRDCIsToSchedule->at(d)->m_type == 2) {
        EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(GBRDCIsToSchedule->at(d)->m_vectorPosition);

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(RandomAccessMsg4->m_destination->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          if (PDCCHResourcesAllocation(RandomAccessMsg4->m_destination->GetIDNetworkNode()) == true) {
            RandomAccessMsg4->m_scheduled = true;

            ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyContentionResolutionIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessMsg4->m_destination);

            m_nMsg4ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nMsg4ToSchedule++;
          }
        }
      } else if (GBRDCIsToSchedule->at(d)->m_type == 3) {
        EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(GBRDCIsToSchedule->at(d)->m_vectorPosition);

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(BSRGrant->m_destination->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(GBRDCIsToSchedule->at(d)->m_vectorPosition);
          if (PDCCHResourcesAllocation(BSRGrant->m_destination->GetIDNetworkNode()) == true) {

            BSRGrant->m_scheduled = true;

            ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyBsrGrantIdealControlMessage, ((EnbMacEntity*) m_mac), BSRGrant->m_destination);

            m_nBsrGrantsReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nBsrGrantsToSchedule++;
          }
        }
      }
    } else {
      break;
    }
  }

  for (int d = 0; d < NGBRDCIsToSchedule->size(); d++) {
    if (m_current_pdcch_cces > 0) {
      if (NGBRDCIsToSchedule->at(d)->m_type == 0) {
        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        //pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona DL
          if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nDlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "DL false" << std::endl;
#endif
          }
        }

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(usersToSchedule->at(NGBRDCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }
        //pdcchFormat = LinkAdaptation(usersToSchedule->at(NGBRDCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode());
        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona UL
          if (PDCCHResourcesAllocation(usersToSchedule->at(NGBRDCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode()) == true) {
            usersToSchedule->at(NGBRDCIsToSchedule->at(d)->m_vectorPosition)->m_allowToSchedule = true;
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "UL false" << std::endl;
#endif
          }
        }
      }
    } else {

      break;
    }
  }*/
}

void
PdcchScheduler::SelectDCIsToSchedule1() {
  m_GBRDCIsToSchedule = new DCIsToSchedule;
  m_NGBRDCIsToSchedule = new DCIsToSchedule;

  /* Select the UL grants */
  UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();
  for (int u = 0; u < usersToSchedule->size(); u++) {
    if (usersToSchedule->at(u)->m_gbr > 0) {
      DCIToSchedule *dci = new DCIToSchedule();
      dci->m_type = 0;
      dci->m_vectorPosition = u;
      dci->m_metric = ComputeULUserTDSchedulingMetric(usersToSchedule->at(u));
      GetGBRDCIsToSchedule()->push_back(dci);
    } else {
      DCIToSchedule *dci = new DCIToSchedule();
      dci->m_type = 0;
      dci->m_vectorPosition = u;
      dci->m_metric = ComputeULUserTDSchedulingMetric(usersToSchedule->at(u));
      GetNGBRDCIsToSchedule()->push_back(dci);
    }
  }

  /* Select the DL grants */
  if (((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetDownlinkTDScheduler() != NULL) {
    DownlinkPacketScheduler::FlowsToSchedule *flowsToSchedule = ((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetFlowsToSchedule();
    for (int f = 0; f < flowsToSchedule->size(); f++) {
      if (flowsToSchedule->at(f)->m_gbr > 0) {
        DCIToSchedule *dci = new DCIToSchedule();
        dci->m_type = 4;
        dci->m_vectorPosition = f;
        dci->m_metric = ComputeDLFlowTDSchedulingMetric(flowsToSchedule->at(f));
        GetGBRDCIsToSchedule()->push_back(dci);
      } else {
        DCIToSchedule *dci = new DCIToSchedule();
        dci->m_type = 4;
        dci->m_vectorPosition = f;
        dci->m_metric = ComputeDLFlowTDSchedulingMetric(flowsToSchedule->at(f));
        GetNGBRDCIsToSchedule()->push_back(dci);
      }
    }
  }

  if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->size() > 0) {
    EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(0);
    DCIToSchedule *dci = new DCIToSchedule();
    dci->m_type = 1;
    dci->m_vectorPosition = 0;
    dci->m_metric = ComputeRandomAccessMsg2TDSchedulingMetric(RandomAccessMsg2);
    GetGBRDCIsToSchedule()->push_back(dci);
  }

  for (int i = 0; i < m_maxMSGs4ToSchedule; i++) {
    if (((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->size() > i) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(i);
      DCIToSchedule *dci = new DCIToSchedule();
      dci->m_type = 2;
      dci->m_vectorPosition = i;
      dci->m_metric = ComputeRandomAccessMsg4TDSchedulingMetric(RandomAccessMsg4);
      GetGBRDCIsToSchedule()->push_back(dci);
    }
  }

  for (int i = 0; i < m_maxBSRGrantsToSchedule; i++) {
    if (((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->size() > i) {
      EnbMacEntity::MsgToSchedule *bsrGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(i);
      DCIToSchedule *dci = new DCIToSchedule();
      dci->m_type = 3;
      dci->m_vectorPosition = i;
      dci->m_metric = ComputeBSRGrantTDSchedulingMetric(bsrGrant);
      GetGBRDCIsToSchedule()->push_back(dci);
    }
  }
}

void
PdcchScheduler::DeleteDCIsToSchedule() {
  if (m_DCIsToSchedule != NULL) {
    if (m_DCIsToSchedule->size() > 0) {

      ClearDCIsToSchedule();
    }
    delete m_DCIsToSchedule;
  }

  m_DCIsToSchedule = NULL;
}

void
PdcchScheduler::ClearDCIsToSchedule() {
  DCIsToSchedule *records = GetDCIsToSchedule();
  DCIsToSchedule::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++) {

    delete *iter;
  }

  GetDCIsToSchedule()->clear();
}

void
PdcchScheduler::DeleteMSGs2ToSchedule() {
  if (m_HighPriorityMSGs2ToSchedule != NULL) {
    m_HighPriorityMSGs2ToSchedule->clear();
    delete m_HighPriorityMSGs2ToSchedule;
  }

  if (m_LowPriorityMSGs2ToSchedule != NULL) {

    m_LowPriorityMSGs2ToSchedule->clear();
    delete m_LowPriorityMSGs2ToSchedule;
  }

  m_HighPriorityMSGs2ToSchedule = NULL;
  m_LowPriorityMSGs2ToSchedule = NULL;
}

void
PdcchScheduler::ClearMSGs2ToSchedule() {
  if (GetHighPriorityMSGs2ToSchedule() != NULL) {
    GetHighPriorityMSGs2ToSchedule()->clear();
  }

  if (GetHighPriorityMSGs2ToSchedule() != NULL) {

    GetLowPriorityMSGs2ToSchedule()->clear();
  }
}

void
PdcchScheduler::DeleteMSGs4ToSchedule() {
  if (m_MSGs4ToSchedule != NULL) {
    if (m_MSGs4ToSchedule->size() > 0) {

      ClearMSGs4ToSchedule();
    }
    delete m_MSGs4ToSchedule;
  }

  m_MSGs4ToSchedule = NULL;
}

void
PdcchScheduler::ClearMSGs4ToSchedule() {
  MSGs4ToSchedule *records = GetMSGs4ToSchedule();
  MSGs4ToSchedule::iterator iter;

  for (iter = records->begin(); iter != records->end(); iter++) {

    delete *iter;
  }

  GetMSGs4ToSchedule()->clear();
}

void
PdcchScheduler::SetMaxMSGs4ToSchedule(int max) {
  if (max == 0) {
    m_maxMSGs4ToSchedule = 100;
  } else {

    m_maxMSGs4ToSchedule = max;
  }
}

void
PdcchScheduler::SchedulingWithouPDCCH() {
  UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

  //Msg2NormalScheduling();
  //Msg4NormalScheduling();

  for (int u = 0; u < usersToSchedule->size(); u++) {

    usersToSchedule->at(u)->m_allowToSchedule = true;
    m_nUlUEsToSchedule++;
  }
}

int
PdcchScheduler::Calc_Y(int k, int RNTI) {
  long int y, t;

  /* Common search space */
  if (RNTI == -1)
    return 0;

  /* Dedicate search space */
  if (k < 0) {
    y = RNTI;
  } else {

    t = Calc_Y(k - 1, RNTI);
    y = (39827 * t) % 65537;
  }

  return y;
}

int
PdcchScheduler::Calc_CCE_Index(int m, int i, int RNTI) {
  int pdcchFormat, C = -1;

  if (RNTI == -1) {
    pdcchFormat = 3;
  } else {
    if (InformationManager::Init()->m_pdcch_format == -1) {
      pdcchFormat = LinkAdaptation(RNTI);
    } else {
      pdcchFormat = InformationManager::Init()->m_pdcch_format;
    }
  }

  if (USE_PDCCH_MANAGER == true) {

    int L = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
    int k = FrameManager::Init()->GetNbSubframes();
    int N = m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces();
    int T = floor(N / L);

    C = L * ((Calc_Y(k, RNTI) + m) % T) + i;

  } else {

    for (int j = 0; j < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces(); j++) {
      if (j + i < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces()) {
        if (PDCCHResources[j + i] == -2) {
          C = j + i;

          break;
        }
      }
    }
  }

  return C;
}

bool
PdcchScheduler::PDCCHResourcesAllocation(int RNTI) {
  int pdcchFormat;
#ifdef PDCCH_DEBUG
  std::cerr << "PDCCH Allocation Start of the UE " << RNTI << std::endl;
#endif

  if (RNTI == -1) {
    for (int m = 0; m < InformationManager::Init()->GetPDCCHNumberOfCandidates_Common(3); m++) {
      if (PDCCHResourcesAllocation(m, 0, RNTI) == true) {
#ifdef PDCCH_DEBUG
        std::cerr << "PDCCH Allocation Stop of the UE " << RNTI << " = YES" << std::endl;
#endif
        return true;
      }
    }
  } else {
    if (InformationManager::Init()->m_pdcch_format == -1) {
      pdcchFormat = LinkAdaptation(RNTI);
    } else {
      pdcchFormat = InformationManager::Init()->m_pdcch_format;
    }

    for (int m = 0; m < InformationManager::Init()->GetPDCCHNumberOfCandidates_UESpecific(pdcchFormat); m++) {
#ifdef PDCCH_DEBUG
      std::cerr << "PDCCH " << m + 1 << " attempt to allocate of the " << RNTI << std::endl;
#endif
      if (PDCCHResourcesAllocation(m, 0, RNTI) == true) {
#ifdef PDCCH_DEBUG

        std::cerr << "PDCCH Allocation Stop of the UE " << RNTI << " = YES" << std::endl;
#endif
        return true;
      }
    }
  }

#ifdef PDCCH_DEBUG
  std::cerr << "PDCCH Allocation Stop of the UE " << RNTI << " = NO" << std::endl;
#endif
  return false;
}

bool
PdcchScheduler::PDCCHResourcesAllocation(int m, int i, int RNTI) {
  int cceIndex, maxCCEs;

  if (RNTI == -1) {
    maxCCEs = 8;
  } else {
    if (InformationManager::Init()->m_pdcch_format == -1) {
      maxCCEs = InformationManager::Init()->GetPDCCHNumberOfCCEs(LinkAdaptation(RNTI));
    } else {
      maxCCEs = InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
    }
  }
#ifdef PDCCH_DEBUG
  std::cout << "PDCCH CCEs for UE " << RNTI << " is " << maxCCEs << std::endl;
#endif

  if (i == maxCCEs) {
    return true;
  }

  cceIndex = Calc_CCE_Index(m, i, RNTI);

  if (cceIndex == -1) {
    return false;
  }

#ifdef PDCCH_DEBUG
  std::cout << "CCE_Index of the UE " << RNTI << " on TTI " << FrameManager::Init()->GetTTICounter() << " is " << cceIndex << std::endl;
#endif

  if (PDCCHResources[cceIndex] != -2) {
#ifdef PDCCH_DEBUG
    std::cout << "PDCCH Allocation of the UE " << RNTI << " is false" << std::endl;
#endif
    return false;
  } else {
    if (PDCCHResourcesAllocation(m, i + 1, RNTI) == true) {
      PDCCHResources[cceIndex] = RNTI;
      return true;
    } else {
      PDCCHResources[cceIndex] = -2;

      return false;
    }
  }
}

bool
PdcchScheduler::PDCCHResourceAllocation(int RNTI) {
  bool allocation = false;
  int pdcchFormat;

  if (InformationManager::Init()->m_pdcch_format == -1) {
    pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
  } else {
    pdcchFormat = InformationManager::Init()->m_pdcch_format;
  }
  //pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
  if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
    //if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format)) {
    // escalona DL
    if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
      //m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
      m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

      m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
      m_nDlUEsToSchedule++;
    } else {
#ifdef PDCCH_DEBUG
      std::cout << "DL false" << std::endl;
#endif
    }
  }

  if (InformationManager::Init()->m_pdcch_format == -1) {
    pdcchFormat = LinkAdaptation(RNTI);
  } else {
    pdcchFormat = InformationManager::Init()->m_pdcch_format;
  }
  //pdcchFormat = LinkAdaptation(RNTI);
  if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
    //if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format)) {
    // escalona UL
    allocation = PDCCHResourcesAllocation(RNTI);
    if (allocation) {
      // escalona UL
      //m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
      m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

      m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
      m_nUlUEsToSchedule++;
    } else {
#ifdef PDCCH_DEBUG

      std::cout << "UL false" << std::endl;
#endif
    }
  }

  return allocation;
}

void
PdcchScheduler::GenerateUserEquipmentDL(int qtd) {
  if (UEsToDL != NULL) {
    if (UEsToDL->size() > 0) {
      UEsToDL->clear();
    }
    delete UEsToDL;
  }

  UEsToDL = new std::vector<int>;

  int a;
  bool f;

  int nM = ((ENodeB*) GetMacEntity()->GetDevice())->GetNbOfUserEquipmentRecords();

  if (qtd >= nM) {
    for (int i = 0; i < nM; i++) {
      a = i + 5;
      UEsToDL->push_back(a);
    }

  } else {

    for (int i = 0; i < qtd && i < nM; i++) {
      f = true;

      while (f) {
        f = false;
        a = Random::Init()->Uniform(5, nM);

        if (UEsToDL != NULL) {
          for (int j = 0; j < UEsToDL->size(); j++) {
            if (a == UEsToDL->at(j)) {

              f = true;
            }
          }
        }
      }

      UEsToDL->push_back(a);
    }
  }
}

double
PdcchScheduler::ComputeBSRGrantTDSchedulingMetric(EnbMacEntity::MsgToSchedule * bsrGrant) {
  double metric = ((Simulator::Init()->Now() - bsrGrant->m_timeArrived) * 1000) / (double) 100;

  return metric;
}

int
PdcchScheduler::LinkAdaptation(int RNTI) {
  ENodeB::UserEquipmentRecord *record = ((ENodeB*) GetMacEntity()->GetDevice())->GetUserEquipmentRecord(RNTI);

  //int AL = GetMacEntity()->GetAmcModule()->GetPdcchAggregationLevelFromRsrp(record->GetUE()->GetReferenceSignalReceivedPower());

  int AL = GetMacEntity()->GetAmcModule()->GetPdcchAggregationLevelFromSinr(record->GetUE()->GetReferenceSignalSinr());

  return AL;
}

void
PdcchScheduler::PPA_Scheduling() {
  int pdcchFormat;

  uDl = 0;

  /* Select the RAR message */
  Msg2PriorityScheduling();

  /* Select the Contention-resolution message */
  Msg4NormalPriorityScheduling();

  /* Select the BSR grants (from SR) */
  BsrGrantNormalPriorityScheduling();

  if (((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUplinkTDScheduler() != NULL) {

    UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

    for (int u = 0; u < usersToSchedule->size(); u++) {

      if (false) {
        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona DL
          if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nDlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "DL false" << std::endl;
#endif
          }
        }
      }

      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(usersToSchedule->at(u)->m_userToSchedule->GetIDNetworkNode());
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }

      if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
        // escalona UL
        if (PDCCHResourcesAllocation(usersToSchedule->at(u)->m_userToSchedule->GetIDNetworkNode())) {
          usersToSchedule->at(u)->m_allowToSchedule = true;
          m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

          m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          m_nUlUEsToSchedule++;
        } else {
#ifdef PDCCH_DEBUG

          std::cout << "UL false" << std::endl;
#endif
        }
      }
    }
  }
}

void
PdcchScheduler::Msg2PriorityScheduling() {
  bool canSch;
  int qtd = 0, pdcchFormat, i, j;

  pdcchFormat = 3;

  //SelectMSG2sToSchedule();
  /**/
  m_HighPriorityMSGs2ToSchedule = new MSGs2ToSchedule;
  m_LowPriorityMSGs2ToSchedule = new MSGs2ToSchedule;

  for (i = 0; i < ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->size(); i++) {
    if (!((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(i)->m_scheduled) {
      if (Simulator::Init()->Now() - ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(i)->m_timeArrived < InformationManager::Init()->ra_ResponseWindowSize) {
        MSG2ToSchedule *msg2 = new MSG2ToSchedule();
        msg2->m_vectorPosition = i;

        if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(i)->m_preamble >= InformationManager::Init()->numberOfRA_Preambles) {
          //std::cout << "PDCCH - High Priority Selected" << std::endl;
          GetHighPriorityMSGs2ToSchedule()->push_back(msg2);
        } else {
          //std::cout << "PDCCH - Low Priority Selected" << std::endl;

          if (InformationManager::Init()->ra_Method == 5) {
            if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(i)->m_preamble < InformationManager::Init()->numberOfRA_Preambles_HTC) {
              //std::cout << "PDCCH - Low Priority Selected HTC" << std::endl;
              msg2->m_metric = 0;
            } else {
              //std::cout << "PDCCH - Low Priority Selected MTC" << std::endl;
              msg2->m_metric = 1;
            }
          } else {
            msg2->m_metric = 0;
          }

          GetLowPriorityMSGs2ToSchedule()->push_back(msg2);
        }
      }
    }
  }
  /**/

  //SortMSGs2ToSchedule(GetLowPriorityMSGs2ToSchedule());
  /**/
  MSGs2ToSchedule *MSGsToSchedule = GetLowPriorityMSGs2ToSchedule();
  MSG2ToSchedule *aux;

  for (i = 1; i < MSGsToSchedule->size(); i++) {
    aux = MSGsToSchedule->at(i);
    j = i;
    while ((j > 0) && (MSGsToSchedule->at(j - 1)->m_metric > aux->m_metric)) {
      MSGsToSchedule->at(j) = MSGsToSchedule->at(j - 1);
      j = j - 1;
    }
    MSGsToSchedule->at(j) = aux;
  }
  /**/

  MSGs2ToSchedule *highPriorityMSGs2ToSchedule = GetHighPriorityMSGs2ToSchedule();
  MSGs2ToSchedule *lowPriorityMSGs2ToSchedule = GetLowPriorityMSGs2ToSchedule();

  if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {

    RAResponseIdealControlMessage *RandomAccessResponse = NULL;

    if (highPriorityMSGs2ToSchedule->size() > 0) {

      if (RandomAccessResponse == NULL) {
        for (int i = 0; i < highPriorityMSGs2ToSchedule->size(); i++) {
          if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition) != NULL) {
            if (!((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_scheduled) {

              /* --- [DRX] --- */
              canSch = true;
              DRXManager *drxMan = ((UserEquipment *) ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_destination)->GetDRXManager();
              if (drxMan != NULL) {
                DRXManager::State drxState = drxMan->getStatus();
                if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
                  if (drxMan->GetExpectedOffTime() > 1) {
                    canSch = false;
                  }
                } else if (drxState == DRXManager::DEEP) {
                  canSch = false;
                }
              }
              /* --- */

              if (canSch) {
                if (Simulator::Init()->Now() - ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_timeArrived < InformationManager::Init()->ra_ResponseWindowSize) {

                  if (PDCCHResourcesAllocation(-1) == false) {
                    return;
                  }

                  RandomAccessResponse = new RAResponseIdealControlMessage(((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_RARNTI, InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
                  RandomAccessResponse->SetSourceDevice(m_mac->GetDevice());

                  m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

                  m_nMsg2ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
                  m_nMsg2ToSchedule++;

                  Simulator::Init()->Schedule(0.000, &EnbMacEntity::SendRandomAccessResponseIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessResponse);

                  break;
                }
              }
            }
          }
        }
      }

      if ((RandomAccessResponse != NULL) && (qtd < m_maxGrantsPerRAR)) {
        for (int i = 0; i < highPriorityMSGs2ToSchedule->size(); i++) {
          if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition) != NULL) {
            if ((!((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_scheduled) && (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_RARNTI == RandomAccessResponse->getRARNTI()) && (qtd < m_maxGrantsPerRAR)) {
              if (Simulator::Init()->Now() - ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_timeArrived < InformationManager::Init()->ra_ResponseWindowSize) {
                /* --- [DRX] --- */
                canSch = true;
                DRXManager *drxMan = ((UserEquipment *) ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_destination)->GetDRXManager();
                if (drxMan != NULL) {
                  DRXManager::State drxState = drxMan->getStatus();
                  if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
                    if (drxMan->GetExpectedOffTime() > 1) {
                      canSch = false;
                    }
                  } else if (drxState == DRXManager::DEEP) {
                    canSch = false;
                  }
                }
                /* --- */
                if (canSch) {
                  ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_scheduled = true;
                  RandomAccessResponse->AddNewRecord(((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_preamble);
                  ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission(1);
                  qtd++;
                }
              } else {
                ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(highPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_scheduled = true;
              }
            }
          }
        }
      }
    }

    if (lowPriorityMSGs2ToSchedule->size() > 0) {

      if (RandomAccessResponse == NULL) {
        for (int i = 0; i < lowPriorityMSGs2ToSchedule->size(); i++) {
          if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition) != NULL) {
            if (!((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_scheduled) {

              /* --- [DRX] --- */
              canSch = true;
              DRXManager *drxMan = ((UserEquipment *) ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_destination)->GetDRXManager();
              if (drxMan != NULL) {
                DRXManager::State drxState = drxMan->getStatus();
                if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
                  if (drxMan->GetExpectedOffTime() > 1) {
                    canSch = false;
                  }
                } else if (drxState == DRXManager::DEEP) {
                  canSch = false;
                }
              }
              /* --- */

              if (canSch) {
                if (Simulator::Init()->Now() - ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_timeArrived < InformationManager::Init()->ra_ResponseWindowSize) {

                  if (PDCCHResourcesAllocation(-1) == false) {
                    return;
                  }

                  RandomAccessResponse = new RAResponseIdealControlMessage(((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_RARNTI, InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
                  RandomAccessResponse->SetSourceDevice(m_mac->GetDevice());

                  m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

                  m_nMsg2ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
                  m_nMsg2ToSchedule++;

                  Simulator::Init()->Schedule(0.000, &EnbMacEntity::SendRandomAccessResponseIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessResponse);

                  break;
                }
              }
            }
          }
        }
      }

      if ((RandomAccessResponse != NULL) && (qtd < m_maxGrantsPerRAR)) {
        for (int i = 0; i < lowPriorityMSGs2ToSchedule->size(); i++) {
          if (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition) != NULL) {
            if ((!((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_scheduled) && (((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_RARNTI == RandomAccessResponse->getRARNTI()) && (qtd < m_maxGrantsPerRAR)) {
              if (Simulator::Init()->Now() - ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_timeArrived < InformationManager::Init()->ra_ResponseWindowSize) {
                /* --- [DRX] --- */
                canSch = true;
                DRXManager *drxMan = ((UserEquipment *) ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_destination)->GetDRXManager();
                if (drxMan != NULL) {
                  DRXManager::State drxState = drxMan->getStatus();
                  if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
                    if (drxMan->GetExpectedOffTime() > 1) {
                      canSch = false;
                    }
                  } else if (drxState == DRXManager::DEEP) {
                    canSch = false;
                  }
                }
                /* --- */
                if (canSch) {
                  ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_scheduled = true;
                  RandomAccessResponse->AddNewRecord(((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_preamble);
                  ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission(1);
                  qtd++;
                }
              } else {
                ((
                        EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(lowPriorityMSGs2ToSchedule->at(i)->m_vectorPosition)->m_scheduled = true;
              }
            }
          }
        }
      }
    }
  }
}

void
PdcchScheduler::SortMSGs2ToSchedule(MSGs2ToSchedule *MSGs2ToSchedule) {
  int i, j;

  MSG2ToSchedule *aux;

  for (i = 1; i < MSGs2ToSchedule->size(); i++) {
    aux = MSGs2ToSchedule->at(i);
    j = i;
    while ((j > 0) && (MSGs2ToSchedule->at(j - 1)->m_metric > aux->m_metric)) {

      MSGs2ToSchedule->at(j) = MSGs2ToSchedule->at(j - 1);
      j = j - 1;
    }
    MSGs2ToSchedule->at(j) = aux;
  }
}

void
PdcchScheduler::ePPA_Scheduling() {
  int pdcchFormat;

  uDl = 0;

  /* Select the RAR message */
  Msg2PriorityScheduling();

  /* Select the Contention-resolution message */
  Msg4PriorityScheduling();

  /* Select the BSR grants (from SR) */
  BsrGrantNormalPriorityScheduling();

  if (((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUplinkTDScheduler() != NULL) {

    UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

    for (int u = 0; u < usersToSchedule->size(); u++) {

      if (false) {
        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona DL
          if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nDlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "DL false" << std::endl;
#endif
          }
        }
      }

      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(usersToSchedule->at(u)->m_userToSchedule->GetIDNetworkNode());
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }

      if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
        // escalona UL
        if (PDCCHResourcesAllocation(usersToSchedule->at(u)->m_userToSchedule->GetIDNetworkNode())) {
          usersToSchedule->at(u)->m_allowToSchedule = true;
          m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

          m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          m_nUlUEsToSchedule++;
        } else {
#ifdef PDCCH_DEBUG

          std::cout << "UL false" << std::endl;
#endif
        }
      }
    }
  }
}

void
PdcchScheduler::Msg4PriorityScheduling() {
  int qtd = 0, pdcchFormat;
  bool canSch = true;
  int i = 0, preamble = -1, index = -1, j;

  EnbMacEntity::MSGsToSchedule::iterator iter;

  m_MSGs4ToSchedule = new MSGs4ToSchedule();

  //Here we create a list of messages 4 as an auxiliary list to schedule them.
  for (iter = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->end(); iter++) {
    MSG4ToSchedule *msg4 = new MSG4ToSchedule();
    msg4->m_vectorPosition = i;
    preamble = (*iter)->m_preamble;

    if (preamble >= InformationManager::Init()->numberOfRA_Preambles) {
      msg4->m_TDMetric = 0;
      m_MSGs4ToSchedule->push_back(msg4);
#ifdef PDCCH_DEBUG
      std::cout << Simulator::Init()->Now() << " PDCCH - High Priority MSG4 Added" << std::endl;
#endif
    } else {
      if (InformationManager::Init()->ra_Method == 5) {//This is RRS?
        if (preamble < InformationManager::Init()->numberOfRA_Preambles_HTC) {
          msg4->m_TDMetric = 1;
#ifdef PDCCH_DEBUG
          std::cout << Simulator::Init()->Now() << " PDCCH - Medium Priority MSG4 Added for HTC" << std::endl;
#endif
        } else {
          msg4->m_TDMetric = 2;
#ifdef PDCCH_DEBUG
          std::cout << Simulator::Init()->Now() << " PDCCH - Low Priority MSG4 Added for MTC" << std::endl;
#endif
        }
      }//Other scheme does not differentiate Low and Medium Priority MSG4
      else {
        msg4->m_TDMetric = 1;
      }
      //Add this message to the vector of msg4 to be scheduled
      m_MSGs4ToSchedule->push_back(msg4);
    }
    i++; //Increase the index of the msg in the vector
#ifdef PDCCH_DEBUG
    std::cout << "PDCCH - Preamble " << preamble << "added to MSG4 of the UE" << std::endl;
#endif
  }

  //Sort the vector of MSg4 to schedule
  MSG4ToSchedule *aux;

  for (i = 1; i < m_MSGs4ToSchedule->size(); i++) {
    aux = m_MSGs4ToSchedule->at(i);
    j = i;
    while ((j > 0) && (m_MSGs4ToSchedule->at(j - 1)->m_TDMetric > aux->m_TDMetric)) {
      m_MSGs4ToSchedule->at(j) = m_MSGs4ToSchedule->at(j - 1);
      j = j - 1;
    }
    m_MSGs4ToSchedule->at(j) = aux;
  }
#ifdef PDCCH_DEBUG
  if (m_MSGs4ToSchedule->size() > 0) {
    std::cerr << Simulator::Init()->Now() << " PDCCH - Sorting MSGs4" << std::endl;
    for (i = 0; i < m_MSGs4ToSchedule->size(); i++) {
      std::cerr << "#" << i << " pos:" << m_MSGs4ToSchedule->at(i)->m_vectorPosition << " metric:" << m_MSGs4ToSchedule->at(i)->m_TDMetric << std::endl;
    }
  }
#endif

  //Perccorrer a lista de msg4 para fazer allocacao
  for (i = 0; i < m_MSGs4ToSchedule->size(); i++) {
    aux = m_MSGs4ToSchedule->at(i);
    index = aux->m_vectorPosition;

    /* --- [DRX] --- */
    DRXManager *drxMan = (((UserEquipment *) ((((EnbMacEntity*) m_mac)->getMSGs4ToSchedule())->at(index))->m_destination))->GetDRXManager();
    if (drxMan != NULL) {
      DRXManager::State drxState = drxMan->getStatus();
      if (drxState == DRXManager::LIGHT || drxState == DRXManager::DEEP || drxState == DRXManager::WAKEUP_LIGHT || drxState == DRXManager::POWERDOWN_LIGHT) {
        if (drxMan->GetExpectedOffTime() > 1)//If GetExpectedOffTime() é igual a 1, no pŕoximo TTI o UE escutará o PDCCH, assim ele poderá ser escalonado para o próximo TTI
          canSch = false;
      } else if (drxState == DRXManager::DEEP) {
        canSch = false;
      }
    }
    /* --- */

    if (canSch) {
      if (UPLINK == true) {
        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(index)->m_destination->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }
      } else {
        pdcchFormat = 2;
      }

      if ((m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) && (qtd < m_maxMSGs4ToSchedule)) {
        if (PDCCHResourcesAllocation(((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(index)->m_destination->GetIDNetworkNode())) {
          qtd++;
          m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          //Verify how much UL resources are needed to send a BSR message
          ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
          ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(index)->m_scheduled = true;

          m_nMsg4ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          m_nMsg4ToSchedule++;

          Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyContentionResolutionIdealControlMessage, ((EnbMacEntity*) m_mac), ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(index)->m_destination);
        }
      }
    }
  }
}

void
PdcchScheduler::BestEffortCCEsScheduling(DCIsToSchedule *DCIsToSchedule) {
  int pdcchFormat;
  int qtdMsg4 = 0;
  int m_UEsScheduled = 0;
  bool msg2Allocated = false;

  UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

  DownlinkPacketScheduler::FlowsToSchedule *flowsToSchedule = ((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetFlowsToSchedule();

  for (int d = 0; d < DCIsToSchedule->size(); d++) {
    if (m_current_pdcch_cces > 0) {

      if (DCIsToSchedule->at(d)->m_type == 0) {

        if (false) {
          if (InformationManager::Init()->m_pdcch_format == -1) {
            pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
          } else {
            pdcchFormat = InformationManager::Init()->m_pdcch_format;
          }

          if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
            if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
              m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

              m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
              m_nDlUEsToSchedule++;

              m_UEsScheduled++;
            } else {
#ifdef PDCCH_DEBUG
              std::cout << "DL false" << std::endl;
#endif
            }
          }
        }

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(usersToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona UL
          if (PDCCHResourcesAllocation(usersToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode())) {
            usersToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_allowToSchedule = true;
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;

            m_UEsScheduled++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "UL false" << std::endl;
#endif
          }
        }

      } else if (DCIsToSchedule->at(d)->m_type == 1) {
        if ((m_current_pdcch_cces >= 8) && (!msg2Allocated)) {

          if (PDCCHResourcesAllocation(-1) == true) {
            int qtd = 0;

            EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);

            RAResponseIdealControlMessage *RandomAccessResponse = NULL;
            RandomAccessResponse = new RAResponseIdealControlMessage(RandomAccessMsg2->m_RARNTI, InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
            RandomAccessResponse->SetSourceDevice(m_mac->GetDevice());

            m_current_pdcch_cces -= 8; // RAR message always uses 8 CCEs

            Simulator::Init()->Schedule(0.000, &EnbMacEntity::SendRandomAccessResponseIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessResponse);

            if (RandomAccessResponse != NULL) {
              EnbMacEntity::MSGsToSchedule::iterator iter;
              for (iter = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->end(); iter++) {
                if ((*iter) != NULL) {
                  if ((!(*iter)->m_scheduled) && ((*iter)->m_RARNTI == RandomAccessResponse->getRARNTI()) && (qtd < m_maxGrantsPerRAR)) {
                    (*iter)->m_scheduled = true;
                    RandomAccessResponse->AddNewRecord((*iter)->m_preamble);
                    ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
                    qtd++;
                  }
                }
              }
            }

            msg2Allocated = true;

            m_nMsg2ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(3);
            m_nMsg2ToSchedule++;

            m_UEsScheduled++;
          }
        }

      } else if (DCIsToSchedule->at(d)->m_type == 2) {
        EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(RandomAccessMsg4->m_destination->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat) && (qtdMsg4 < m_maxMSGs4ToSchedule)) {
          if (PDCCHResourcesAllocation(RandomAccessMsg4->m_destination->GetIDNetworkNode()) == true) {

            RandomAccessMsg4->m_scheduled = true;

            ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyContentionResolutionIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessMsg4->m_destination);

            qtdMsg4++;

            m_nMsg4ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nMsg4ToSchedule++;

            m_UEsScheduled++;
          }
        }
      } else if (DCIsToSchedule->at(d)->m_type == 3) {
        EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);

        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(BSRGrant->m_destination->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          if (PDCCHResourcesAllocation(BSRGrant->m_destination->GetIDNetworkNode()) == true) {

            BSRGrant->m_scheduled = true;

            ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyBsrGrantIdealControlMessage, ((EnbMacEntity*) m_mac), BSRGrant->m_destination);

            m_nBsrGrantsReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nBsrGrantsToSchedule++;

            m_UEsScheduled++;
          }
        }
      } else if (DCIsToSchedule->at(d)->m_type == 4) {
        if (InformationManager::Init()->m_pdcch_format == -1) {
          pdcchFormat = LinkAdaptation(flowsToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode());
        } else {
          pdcchFormat = InformationManager::Init()->m_pdcch_format;
        }
        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona DL
          if (PDCCHResourcesAllocation(flowsToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode())) {
            flowsToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_allowToSchedule = true;
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;

            m_UEsScheduled++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "DL false" << std::endl;
#endif
          }
        }

      }
    } else {
      break;
    }
  }

#ifdef PDCCH_DEBUG
  std::cerr << "[DEBUG]" << std::endl;
  std::cerr << "\tPDCCH NORMAL [";
  for (int j = 0; j < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces(); j++) {
    std::cerr << " " << PDCCHResources[j];
  }
  std::cerr << " ]" << std::endl;
  std::cerr << "\t# UEs Scheduled in NORMAL = " << m_UEsScheduled << std::endl;
#endif
}

void
PdcchScheduler::NewCCEsScheduling(DCIsToSchedule *DCIsToSchedule) {
  int pdcchFormat;
  int qtdMsg4 = 0;
  int m_UEsScheduled_temp = 0;
  int maxCCEs;
  int RNTI;
  int cceIndex;
  bool msg2Allocated = false;

  UplinkPacketScheduler::UsersToSchedule *usersToSchedule = ((UplinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetUplinkPacketScheduler())->GetUsersToSchedule();

  DownlinkPacketScheduler::FlowsToSchedule *flowsToSchedule = ((DownlinkPacketScheduler*) ((EnbMacEntity*) m_mac)->GetDownlinkPacketScheduler())->GetFlowsToSchedule();

  //std::cout << "DCI to Schedule = " << DCIsToSchedule->size() << std::endl;

  for (int d = 0; d < DCIsToSchedule->size(); d++) {
    //std::cout << "Vector Position = " << DCIsToSchedule->at(d)->m_vectorPosition << std::endl;
    if (DCIsToSchedule->at(d)->m_type == 0) {
      RNTI = usersToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(d)->m_type == 1) {
      //EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);
      RNTI = -1;
    } else if (DCIsToSchedule->at(d)->m_type == 2) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);
      RNTI = RandomAccessMsg4->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(d)->m_type == 3) {
      EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(d)->m_vectorPosition);
      RNTI = BSRGrant->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(d)->m_type == 4) {
      RNTI = flowsToSchedule->at(DCIsToSchedule->at(d)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode();
    }

    if (RNTI == -1) {
      pdcchFormat = 3;
    } else {
      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(RNTI);
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }
    }

    maxCCEs = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

    for (int m = 0; m < InformationManager::Init()->GetPDCCHNumberOfCandidates_UESpecific(pdcchFormat); m++) {
      for (int i = 0; i < maxCCEs; i++) {
        cceIndex = Calc_CCE_Index(m, i, RNTI);
        PDCCHCcesPerUser[d][cceIndex] = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
      }
    }
  }

#ifdef PDCCH_DEBUG
  std::cerr << "[DEBUG]" << std::endl;
  for (int i = 0; i < DCIsToSchedule->size(); i++) {
    if (DCIsToSchedule->at(i)->m_type == 0) {
      RNTI = usersToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 1) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = -1;
    } else if (DCIsToSchedule->at(i)->m_type == 2) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = RandomAccessMsg4->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 3) {
      EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = BSRGrant->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 4) {
      RNTI = flowsToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode();
    }

    if (RNTI == -1) {
      pdcchFormat = 3;
    } else {
      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(RNTI);
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }
    }

    std::cerr << "\t " << RNTI
            << "\t[" << InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)
            << ", " << InformationManager::Init()->GetPDCCHNumberOfCandidates_UESpecific(pdcchFormat)
            << "]:";
    for (int j = 0; j < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces(); j++) {
      std::cerr << " " << PDCCHCcesPerUser[i][j];
    }
    std::cerr << std::endl;
  }
#endif

  int PDCCHResources_temp[84], total, where;

  for (int i = 0; i < 84; i++) {
    PDCCHResources_temp[i] = -2;
  }

  for (int i = 0; i < DCIsToSchedule->size(); i++) {
    if (DCIsToSchedule->at(i)->m_type == 0) {
      RNTI = usersToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_userToSchedule->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 1) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = -1;
    } else if (DCIsToSchedule->at(i)->m_type == 2) {
      EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = RandomAccessMsg4->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 3) {
      EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);
      RNTI = BSRGrant->m_destination->GetIDNetworkNode();
    } else if (DCIsToSchedule->at(i)->m_type == 4) {
      RNTI = flowsToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_bearer->GetDestination()->GetIDNetworkNode();
    }

    if (RNTI == -1) {
      pdcchFormat = 3;
    } else {
      if (InformationManager::Init()->m_pdcch_format == -1) {
        pdcchFormat = LinkAdaptation(RNTI);
      } else {
        pdcchFormat = InformationManager::Init()->m_pdcch_format;
      }
    }

    int h = 1;

    //std::cout << "PDCCH UE " << i << std::endl;

    where = -1;
    total = 100000;

    for (int j = 0; j < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces(); j += h) {
      h = 1;
      if (PDCCHCcesPerUser[i][j] != 0) {
        int soma = 0;
        int first = j;

        for (int k = 0; k < InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat); k++) {
          if (PDCCHResources_temp[j + k] == -2) {
            for (int p = i + 1; p < DCIsToSchedule->size(); p++) {
              soma += PDCCHCcesPerUser[p][j + k];
            }

            h = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
          } else {
            soma = 1000000;
            h = InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            break;
          }
        }

        if (soma < total) {
          where = first;
          total = soma;
        }
      }
    }

    //std::cout << "where " << where << std::endl;
    if (where != -1) {

      if (DCIsToSchedule->at(i)->m_type == 0) {

        if (false) {
          if (InformationManager::Init()->m_pdcch_format == -1) {
            pdcchFormat = LinkAdaptation(UEsToDL->at(uDl));
          } else {
            pdcchFormat = InformationManager::Init()->m_pdcch_format;
          }

          if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
            if (PDCCHResourcesAllocation(UEsToDL->at(uDl++))) {
              m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

              m_nDlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
              m_nDlUEsToSchedule++;
            } else {
#ifdef PDCCH_DEBUG
              std::cout << "DL false" << std::endl;
#endif
            }
          }
        }

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona UL
          if (PDCCHResourcesAllocation(RNTI)) {
            usersToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_allowToSchedule = true;
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "UL false" << std::endl;
#endif
          }
        }

      } else if (DCIsToSchedule->at(i)->m_type == 1) {
        if ((m_current_pdcch_cces >= 8) && (!msg2Allocated)) {

          if (PDCCHResourcesAllocation(RNTI) == true) {
            int qtd = 0;

            EnbMacEntity::MsgToSchedule *RandomAccessMsg2 = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);

            RAResponseIdealControlMessage *RandomAccessResponse = NULL;
            RandomAccessResponse = new RAResponseIdealControlMessage(RandomAccessMsg2->m_RARNTI, InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
            RandomAccessResponse->SetSourceDevice(m_mac->GetDevice());

            m_current_pdcch_cces -= 8; // RAR message always uses 8 CCEs

            Simulator::Init()->Schedule(0.000, &EnbMacEntity::SendRandomAccessResponseIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessResponse);

            if (RandomAccessResponse != NULL) {
              EnbMacEntity::MSGsToSchedule::iterator iter;
              for (iter = ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->begin(); iter != ((EnbMacEntity*) m_mac)->getMSGs2ToSchedule()->end(); iter++) {
                if ((*iter) != NULL) {
                  if ((!(*iter)->m_scheduled) && ((*iter)->m_RARNTI == RandomAccessResponse->getRARNTI()) && (qtd < m_maxGrantsPerRAR)) {
                    (*iter)->m_scheduled = true;
                    RandomAccessResponse->AddNewRecord((*iter)->m_preamble);
                    ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
                    qtd++;
                  }
                }
              }
            }

            msg2Allocated = true;

            m_nMsg2ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nMsg2ToSchedule++;
          }
        }

      } else if (DCIsToSchedule->at(i)->m_type == 2) {
        EnbMacEntity::MsgToSchedule *RandomAccessMsg4 = ((EnbMacEntity*) m_mac)->getMSGs4ToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat) && (qtdMsg4 < m_maxMSGs4ToSchedule)) {
          if (PDCCHResourcesAllocation(RNTI) == true) {

            RandomAccessMsg4->m_scheduled = true;

            ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyContentionResolutionIdealControlMessage, ((EnbMacEntity*) m_mac), RandomAccessMsg4->m_destination);

            qtdMsg4++;

            m_nMsg4ReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nMsg4ToSchedule++;
          }
        }
      } else if (DCIsToSchedule->at(i)->m_type == 3) {
        EnbMacEntity::MsgToSchedule *BSRGrant = ((EnbMacEntity*) m_mac)->GetBsrGrantsToSchedule()->at(DCIsToSchedule->at(i)->m_vectorPosition);

        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          if (PDCCHResourcesAllocation(RNTI) == true) {

            BSRGrant->m_scheduled = true;

            ((EnbMacEntity*) m_mac)->AllocationResourceToDataTransmission();
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyBsrGrantIdealControlMessage, ((EnbMacEntity*) m_mac), BSRGrant->m_destination);

            m_nBsrGrantsReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nBsrGrantsToSchedule++;
          }
        }
      } else if (DCIsToSchedule->at(i)->m_type == 4) {
        if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat)) {
          // escalona DL
          if (PDCCHResourcesAllocation(RNTI)) {
            flowsToSchedule->at(DCIsToSchedule->at(i)->m_vectorPosition)->m_allowToSchedule = true;
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);

            m_nUlReservedCces += InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat);
            m_nUlUEsToSchedule++;
          } else {
#ifdef PDCCH_DEBUG
            std::cout << "DL false" << std::endl;
#endif
          }
        }
      }


      m_UEsScheduled_temp++;
      for (int k = 0; k < InformationManager::Init()->GetPDCCHNumberOfCCEs(pdcchFormat); k++) {
        PDCCHResources_temp[where + k] = RNTI;
      }
    }
  }

#ifdef PDCCH_DEBUG
  std::cerr << "[DEBUG]" << std::endl;
  std::cerr << "\tPDCCH TEMP [";
  for (int j = 0; j < m_mac->GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces(); j++) {
    std::cerr << " " << PDCCHResources_temp[j];
  }
  std::cerr << " ]" << std::endl;
  std::cerr << "\t# UEs Scheduled in TEMP = " << m_UEsScheduled_temp << std::endl;
#endif
}

void
PdcchScheduler::MinAL_Algorithm() {
  SelectDCIsToSchedule();

  std::cout << "\tDCIs without sort:" << std::endl;
  for (int d = 0; d < GetDCIsToSchedule()->size(); d++) {
    std::cout << "\t\t" << d << " : " << GetDCIsToSchedule()->at(d)->m_aggregationLevel << std::endl;
  }

  OrderDCIsByAggregationLevel(GetDCIsToSchedule());

  std::cout << "\tDCIs sorted:" << std::endl;
  for (int d = 0; d < GetDCIsToSchedule()->size(); d++) {
    std::cout << "\t\t" << d << " : " << GetDCIsToSchedule()->at(d)->m_aggregationLevel << std::endl;
  }
}

void
PdcchScheduler::OrderDCIsByAggregationLevel(DCIsToSchedule *DCIsToSchedule) {
  int i, j;

  DCIToSchedule *aux;

  for (i = 1; i < DCIsToSchedule->size(); i++) {
    aux = DCIsToSchedule->at(i);
    j = i;
    while (j > 0 && DCIsToSchedule->at(j - 1)->m_aggregationLevel > aux->m_aggregationLevel) {
      DCIsToSchedule->at(j) = DCIsToSchedule->at(j - 1);
      j = j - 1;
    }
    DCIsToSchedule->at(j) = aux;
  }
}