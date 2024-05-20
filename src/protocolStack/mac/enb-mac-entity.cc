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
 * Author: Giuseppe Piro <g.piro@poliba.it>
 */

#include <limits>
#include <fstream>
#include "enb-mac-entity.h"

#include "../packet/Packet.h"
#include "../packet/packet-burst.h"
#include "AMCModule.h"
#include "../../core/idealMessages/ideal-control-messages.h"
#include "../../device/NetworkNode.h"
#include "packet-scheduler/packet-scheduler.h"
#include "pdcch-scheduler/pdcch-scheduler.h"
#include "../../device/UserEquipment.h"
#include "../../device/ENodeB.h"
#include "../../load-parameters.h"

#include "../../flows/application/WEB.h"
#include "../../phy/lte-phy.h"

#include "../../componentManagers/InformationManager.h"
#include "../../core/spectrum/bandwidth-manager.h"
#include "../../componentManagers/FrameManager.h"

EnbMacEntity::EnbMacEntity()
{
  SetAmcModule(new AMCModule());
  SetDevice(NULL);

  m_downlinkScheduler = NULL;
  m_uplinkScheduler = NULL;

  m_RAPreambleContainer = new std::vector<RAPreambleIdealControlMessage *>;
  m_ConnectionRequestContainer = new std::vector<ConnectionRequestIdealControlMessage *>;
  m_SchedulingRequestContainer = new std::vector<SchedulingRequestIdealControlMessage *>;

  m_unsuccessful_ra_counter = 0;
  m_total_ra_counter = 0;

  m_allocated_ul_rbs = 0;
  m_current_pdcch_cces = 0;

  m_allocated_ul_prbs_msg3_retransmition = 0;

  m_rar_sent = 0;

  m_preamble_transmissions = 0;

  m_information_changed = false;

  tt_pream = 0;

  /*for (int i = 0; i < 64; i++) {
    m_preamblesReceived[i] = 0;
  }*/

  if ((InformationManager::Init()->ra_Method == 2) && (InformationManager::Init()->acb_Dynamic == true))
  {
    Simulator::Init()->Schedule(0.000, &EnbMacEntity::ProcessPreamblesReceived, this);
    Simulator::Init()->Schedule((double)(InformationManager::Init()->acb_MonitoringPeriod / (double)1000), &EnbMacEntity::TimeoutMonitoringCycle, this);
  }
  else if (InformationManager::Init()->ra_Method == 4)
  {
    Simulator::Init()->Schedule((double)(InformationManager::Init()->QoSDracon_MonitoringPeriod / (double)1000), &EnbMacEntity::TimeoutMonitoringCycle, this);
  }
  else if (InformationManager::Init()->ra_Method == 6)
  {
    m_eab_index = -1;
    m_eab_on = false;
    Simulator::Init()->Schedule((double)(InformationManager::Init()->eab_MonitoringPeriod / (double)1000), &EnbMacEntity::TimeoutMonitoringCycle, this);
  }

  m_msgs2ToSchedule = new MSGsToSchedule();
  m_msgs4ToSchedule = new MSGsToSchedule();

  m_bsrGrantsToSchedule = new MSGsToSchedule();
}

EnbMacEntity::~EnbMacEntity()
{
  delete m_RAPreambleContainer;
  delete m_ConnectionRequestContainer;
  delete m_SchedulingRequestContainer;

  delete m_downlinkScheduler;
  delete m_uplinkScheduler;

  Destroy();
}

std::vector<RAPreambleIdealControlMessage *> *EnbMacEntity::GetM_RAPreambleContainer()
{
  return m_RAPreambleContainer;
}

void EnbMacEntity::SetUplinkPacketScheduler(PacketScheduler *s)
{
  m_uplinkScheduler = s;
}

void EnbMacEntity::SetDownlinkPacketScheduler(PacketScheduler *s)
{
  m_downlinkScheduler = s;
}

void EnbMacEntity::SetPdcchScheduler(PdcchScheduler *s)
{
  m_pdcchScheduler = s;
}

PacketScheduler *
EnbMacEntity::GetUplinkPacketScheduler()
{
  return m_uplinkScheduler;
}

PacketScheduler *
EnbMacEntity::GetDownlinkPacketScheduler()
{
  return m_downlinkScheduler;
}

PdcchScheduler *
EnbMacEntity::GetPdcchScheduler()
{
  return m_pdcchScheduler;
}

void EnbMacEntity::ReceiveCqiIdealControlMessage(CqiIdealControlMessage *msg)
{
#ifdef TEST_CQI_FEEDBACKS
  std::cout << "ReceiveIdealControlMessage (MAC) from  " << msg->GetSourceDevice()->GetIDNetworkNode() << " to " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
#endif

  CqiIdealControlMessage::CqiFeedbacks *cqi = msg->GetMessage();

  UserEquipment *ue = (UserEquipment *)msg->GetSourceDevice();
  ENodeB *enb = (ENodeB *)GetDevice();
  ENodeB::UserEquipmentRecord *record = enb->GetUserEquipmentRecord(ue->GetIDNetworkNode());

  if (record != NULL)
  {
    std::vector<int> cqiFeedback;
    for (CqiIdealControlMessage::CqiFeedbacks::iterator it = cqi->begin(); it != cqi->end(); it++)
    {
      cqiFeedback.push_back((*it).m_cqi);
    }

#ifdef TEST_CQI_FEEDBACKS
    std::cout << "\t CQI: ";
    for (int i = 0; i < cqiFeedback.size(); i++)
    {
      std::cout << cqiFeedback.at(i) << " ";
    }
    std::cout << std::endl;
#endif

#ifdef AMC_MAPPING
    std::cout << "\t CQI: ";
    for (int i = 0; i < cqiFeedback.size(); i++)
    {
      std::cout << cqiFeedback.at(i) << " ";
    }
    std::cout << std::endl;

    std::cout << "\t MCS: ";
    for (int i = 0; i < cqiFeedback.size(); i++)
    {
      std::cout << GetAmcModule()->GetMCSFromCQI(cqiFeedback.at(i)) << " ";
    }
    std::cout << std::endl;

    std::cout << "\t TB: ";
    for (int i = 0; i < cqiFeedback.size(); i++)
    {
      std::cout << GetAmcModule()->GetTBSizeFromMCS(
                       GetAmcModule()->GetMCSFromCQI(cqiFeedback.at(i)))
                << " ";
    }
    std::cout << std::endl;
#endif

    record->SetCQI(cqiFeedback);
  }
  else
  {
    std::cout << "ERROR: received CQI from unknown ue!" << std::endl;
  }
}

void EnbMacEntity::SendPdcchMapIdealControlMessage(PdcchMapIdealControlMessage *pdcchMsg)
{
  if (pdcchMsg->GetMessage()->size() > 0)
  {
    GetDevice()->GetPhy()->SendIdealControlMessage(pdcchMsg);
  }
}

void EnbMacEntity::ReceiveBufferStatusReportingIdealControlMessage(BufferStatusReportingIdealControlMessage *msg)
{
  UserEquipment *ue = (UserEquipment *)msg->GetSourceDevice();
  ENodeB *enb = (ENodeB *)GetDevice();
  ENodeB::UserEquipmentRecord *record = enb->GetUserEquipmentRecord(ue->GetIDNetworkNode());

  int bufferStatusReport = msg->GetBufferStatusReport();

  if (record != NULL)
  {
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " received BSR with " << bufferStatusReport << " byte(s) from UE " << ue->GetIDNetworkNode() << std::endl;
    record->SetSchedulingRequest(bufferStatusReport);

    if (msg->GetDestinationUeD2D())
    {
      record->SetD2DCommunicationUE((UserEquipment *)msg->GetDestinationUeD2D());
    }
  }
  else
  {
    std::cout << "ERROR: received BSR from unknown ue!" << std::endl;
  }

  delete msg;
}

void EnbMacEntity::ReceiveBufferStatusReportingMessage(int source, int bufferSize)
{
  ENodeB *enb = (ENodeB *)GetDevice();
  ENodeB::UserEquipmentRecord *record = enb->GetUserEquipmentRecord(source);

  int bufferStatusReport = bufferSize;

  if (record != NULL)
  {
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " received BSR with " << bufferStatusReport << " byte(s) from UE " << source << std::endl;
    record->SetSchedulingRequest(bufferStatusReport);

    // if (msg->GetDestinationUeD2D()) {
    // record->SetD2DCommunicationUE((UserEquipment *) msg->GetDestinationUeD2D());
    // }
  }
  else
  {
    std::cout << "ERROR: received BSR from unknown ue!" << std::endl;
  }
}

void EnbMacEntity::ReceiveRandomAccessPreambleMessage(RAPreambleIdealControlMessage *RAPreamble)
{
  m_preamble_transmissions++;

  std::cerr << Simulator::Init()->Now()
            << " ENodeB " << GetDevice()->GetIDNetworkNode()
            << " received RA_Preamble " << RAPreamble->GetPreamble()
            << " from UE " << ((UserEquipment *)RAPreamble->GetSourceDevice())->GetIDNetworkNode()
            << std::endl;

  m_preamblesReceived[RAPreamble->GetPreamble()]++;
  std::cout << "Preambulo: " << RAPreamble->GetPreamble() << std::endl;
  all_preambles.push_back(RAPreamble->GetPreamble());
  RAPreamble->setCollision(false);
  RAPreamble->m_processingTime = 4;

  m_RAPreambleContainer->push_back(RAPreamble);
}

// *******************Nuevo método para procesar preámbulos de acceso aleatorio********************//
void EnbMacEntity::ProcessRandomAccessPreambleMessages(int raSlot)
{
    std::string path = "./jsons/";
    std::string nameOfJSonFile = path + "file_" + std::to_string(raSlot) + ".json";
    std::vector<RAPreambleIdealControlMessage *> m_ProcessedRAPreambleContainer;
    std::vector<RAPreambleIdealControlMessage *> m_RAPreambleContainerTmp;

    RAPreambleIdealControlMessage *RAPreamble;

    if (!m_RAPreambleContainer->empty())
    {
        // Procesar los preámbulos con RARNTI igual a raSlot
        for (auto iter = m_RAPreambleContainer->begin(); iter != m_RAPreambleContainer->end(); iter++)
        {
            RAPreamble = *iter;
            if (RAPreamble->getRARNTI() == raSlot)
            {
                // Agregar el preámbulo procesado al contenedor m_ProcessedRAPreambleContainer
                m_ProcessedRAPreambleContainer.push_back(RAPreamble);
            }
            else
            {
                m_RAPreambleContainerTmp.push_back(RAPreamble);
            }
        }

        if (!m_ProcessedRAPreambleContainer.empty())
        {
            std::ofstream myJSonFile(nameOfJSonFile);

            if (myJSonFile.is_open())
            {
                myJSonFile << "{\n  \"Preambles\": [\n";
                // Procesar los preámbulos con RARNTI igual a raSlot
                for (size_t i = 0; i < m_ProcessedRAPreambleContainer.size(); ++i)
                {
                    RAPreamble = m_ProcessedRAPreambleContainer[i];

                    myJSonFile << "    {\n";
                    myJSonFile << "      \"Preamble\": " + std::to_string(RAPreamble->GetPreamble()) + ",\n";
                    myJSonFile << "      \"RARNTI\": " + std::to_string(RAPreamble->getRARNTI()) + ",\n";
                    myJSonFile << "      \"SourceDevice\": " + std::to_string(static_cast<UserEquipment *>(RAPreamble->GetSourceDevice())->GetIDNetworkNode()) + ",\n";
                    myJSonFile << "      \"PosX\": " + std::to_string(static_cast<UserEquipment *>(RAPreamble->GetSourceDevice())->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX()) + ",\n";
                    myJSonFile << "      \"PosY\": " + std::to_string(static_cast<UserEquipment *>(RAPreamble->GetSourceDevice())->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY()) + ",\n";
                    myJSonFile << "      \"Distance\": " + std::to_string(static_cast<UserEquipment *>(RAPreamble->GetSourceDevice())->GetMobilityModel()->GetAbsolutePosition()->GetDistance(static_cast<UserEquipment *>(RAPreamble->GetSourceDevice())->GetTargetNode()->GetMobilityModel()->GetAbsolutePosition())) + "\n";
                    myJSonFile << "    }" + std::string((i < m_ProcessedRAPreambleContainer.size() - 1) ? "," : "") + "\n";
                }
                myJSonFile << "  ]\n}\n";

                // Cerrar el archivo JSON
                myJSonFile.close();
            }
        }

        // Reemplazar el contenedor original con el temporal
        m_RAPreambleContainer->clear();
        *m_RAPreambleContainer = std::move(m_RAPreambleContainerTmp);
    }
}


// *******************Nuevo método para preparar el mensaje de RA********************//
//void EnbMacEntity::PrepareRandomAccessResponseMessage(int raSlot, std::vector<RAPreambleIdealControlMessage *> *processedRAPreambleContainer)
void EnbMacEntity::PrepareRandomAccessResponseMessage(int raSlot, std::vector<RAPreambleIdealControlMessage *> *processedRAPreambleContainer)
{
  RAPreambleIdealControlMessage *RAPreamble;

  // Esperar los datos de MATLAB (simulado aquí como una espera de 1 segundo)
  // Process the MatLab
  // system("&");
  usleep(2000);

  for (auto iter = processedRAPreambleContainer->begin(); iter != processedRAPreambleContainer->end(); iter++)
  {
    RAPreamble = *iter;
    std::cerr << Simulator::Init()->Now()
              << " [DEBUG-RA] @ENodeB: Preamble_Collision " << RAPreamble->getCollision()
              << " from UE " << RAPreamble->GetSourceDevice()->GetIDNetworkNode()
              << " #Preambles_received " << m_RAPreambleContainer->size()
              << std::endl;

    RAPreamble->setIsQueue(true);

    UserEquipment *tmp_ue = ((UserEquipment *)RAPreamble->GetSourceDevice());
    UeMacEntity *tmp_ue_mac = (UeMacEntity *)tmp_ue->GetProtocolStack()->GetMacEntity();

    tmp_ue_mac->m_temp_enb_users_trying_RA = m_RAPreambleContainer->size();
    tmp_ue_mac->m_temp_enb_preambles_collided = 0;
    tmp_ue_mac->m_temp_enb_preambles_distinct = 0;

    std::vector<RAPreambleIdealControlMessage *>::iterator it;
    std::vector<RAPreambleIdealControlMessage *>::iterator jt;
    int ct = 0;
    for (it = m_RAPreambleContainer->begin(); it != m_RAPreambleContainer->end(); it++)
    {
      if (RAPreamble->getRARNTI() == (*it)->getRARNTI())
      {
        ct++;
      }
    }

    ct = (ct > 15 ? 15 : ct);
    tmp_ue_mac->setCounterMatchedRA_RNTI(ct);

    std::cerr << Simulator::Init()->Now()
              << " [DEBUG-RA-CT] UE " << tmp_ue->GetIDNetworkNode()
              << " : got RARNTI counter of " << ct
              << std::endl;

    if (RAPreamble->getCollision() == false)
    {
      MsgToSchedule *msg = new MsgToSchedule();
      msg->m_RARNTI = RAPreamble->getRARNTI();
      msg->m_preamble = RAPreamble->GetPreamble();
      msg->m_timeArrived = Simulator::Init()->Now();
      msg->m_destination = RAPreamble->GetSourceDevice();
      msg->m_scheduled = false;

      m_msgs2ToSchedule->push_back(msg);
    }
  }

  // Limpiar el contenedor de preámbulos procesados
  processedRAPreambleContainer->clear();
  delete processedRAPreambleContainer;
}
//***************************************************************

void EnbMacEntity::ReceiveRandomAccessPreambleIdealControlMessage(RAPreambleIdealControlMessage *RAPreamble)
{
  std::vector<RAPreambleIdealControlMessage *>::iterator iter;

  bool collision = false;

  m_preamble_transmissions++;

  if (RAPreamble->getError() == false)
  {
    std::cerr << Simulator::Init()->Now()
              << " ENodeB " << GetDevice()->GetIDNetworkNode()
              << " received RA_Preamble " << RAPreamble->GetPreamble()
              << " from UE " << ((UserEquipment *)RAPreamble->GetSourceDevice())->GetIDNetworkNode()
              << std::endl;

    for (iter = m_RAPreambleContainer->begin(); iter != m_RAPreambleContainer->end(); iter++)
    {
      if (((*iter)->getRARNTI() == RAPreamble->getRARNTI()) && ((*iter)->GetPreamble() == RAPreamble->GetPreamble()))
      {
        collision = true;

        if (InformationManager::Init()->ra_Method == 9)
        {
          (*iter)->setCollision(true);
        }
      }
    }

    m_preamblesReceived[RAPreamble->GetPreamble()]++;
    // std::cout << "Preambulo: " << RAPreamble->GetPreamble() << std::endl;
    all_preambles.push_back(RAPreamble->GetPreamble());
    RAPreamble->setCollision(collision);
    RAPreamble->m_processingTime = 4;

    m_RAPreambleContainer->push_back(RAPreamble);
    Simulator::Init()->Schedule(0.003, &EnbMacEntity::ReadyRandomAccessPreambleIdealControlMessage1, this, RAPreamble);
  }
  else
  {
    std::cerr << Simulator::Init()->Now()
              << " ENodeB " << GetDevice()->GetIDNetworkNode()
              << " dropped RA_Preamble " << RAPreamble->GetPreamble()
              << " from UE " << ((UserEquipment *)RAPreamble->GetSourceDevice())->GetIDNetworkNode()
              << std::endl;

    Simulator::Init()->Schedule(0.003, &EnbMacEntity::FillCounterUEsFailedPreambles, this, RAPreamble);

    // delete RAPreamble;
  }
}

std::vector<int> EnbMacEntity::GetAllPreambles()
{
  return all_preambles;
}

void EnbMacEntity::FillCounterUEsFailedPreambles(RAPreambleIdealControlMessage *RAPreamble)
{
  UserEquipment *tmp_ue = ((UserEquipment *)RAPreamble->GetSourceDevice());
  UeMacEntity *tmp_ue_mac = (UeMacEntity *)tmp_ue->GetProtocolStack()->GetMacEntity();

  std::vector<RAPreambleIdealControlMessage *>::iterator it;
  std::vector<RAPreambleIdealControlMessage *>::iterator jt;
  int ct = 0;
  for (it = m_RAPreambleContainer->begin(); it != m_RAPreambleContainer->end(); it++)
  {
    if (RAPreamble->getRARNTI() == (*it)->getRARNTI())
    {
      ct++;
    }
  }

  ct = (ct > 15 ? 15 : ct);
  tmp_ue_mac->setCounterMatchedRA_RNTI(ct);
  std::cerr << Simulator::Init()->Now()
            << " [DEBUG-RA-FAILED] @ENodeB: "
            << " Filling Failed UE " << RAPreamble->GetSourceDevice()->GetIDNetworkNode()
            << " with #Preambles_received " << ct
            << std::endl;

  delete RAPreamble;
}

void EnbMacEntity::ReadyRandomAccessPreambleIdealControlMessage()
{
  std::vector<RAPreambleIdealControlMessage *>::iterator iter;

  /*for (iter = m_RAPreambleContainer->begin(); iter != m_RAPreambleContainer->end(); iter++) {
    if ((*iter)->m_processingTime > 0) {
      (*iter)->m_processingTime--;
    } else if ((*iter)->m_processingTime == 0) {
      (*iter)->m_waitingTime++;
    }
  }*/

  if (ONLY_RANDOM_ACCESS == true)
  {
    int qtd = 0;

    m_current_pdcch_cces = InformationManager::Init()->m_NumberOfCCEs;

    if (m_current_pdcch_cces >= 8)
    {

      std::vector<RAPreambleIdealControlMessage *> *RAPreamblesContainerTemp = new std::vector<RAPreambleIdealControlMessage *>;

      RAResponseIdealControlMessage *RAResponse = NULL;
      for (iter = m_RAPreambleContainer->begin(); iter != m_RAPreambleContainer->end(); iter++)
      {
        if (((*iter)->m_processingTime == 0) && ((*iter)->m_waitingTime <= InformationManager::Init()->ra_ResponseWindowSize))
        {
          if ((*iter)->getCollision() == false)
          {
            RAResponse = new RAResponseIdealControlMessage((*iter)->getRARNTI(), InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
            RAResponse->SetSourceDevice(GetDevice());
            m_current_pdcch_cces -= 8;
            // Simulator::Init()->Schedule(0.000, &EnbMacEntity::SendRandomAccessResponseIdealControlMessage, this, RAResponse);
            break;
          }
        }
      }

      for (iter = m_RAPreambleContainer->begin(); iter != m_RAPreambleContainer->end(); iter++)
      {
        if ((RAResponse != NULL) && (qtd < 3))
        {
          if (((*iter)->m_processingTime == 0) && ((*iter)->m_waitingTime <= InformationManager::Init()->ra_ResponseWindowSize) && ((*iter)->getRARNTI() == RAResponse->getRARNTI()) && ((*iter)->getCollision() == false))
          {
            RAResponse->AddNewRecord((*iter)->GetPreamble());
            qtd++;
          }
          else
          {
            if ((*iter)->m_waitingTime < InformationManager::Init()->ra_ResponseWindowSize)
            {
              RAPreamblesContainerTemp->push_back(*iter);
            }
          }
        }
        else
        {
          if ((*iter)->m_waitingTime < InformationManager::Init()->ra_ResponseWindowSize)
          {
            RAPreamblesContainerTemp->push_back(*iter);
          }
        }
      }

      m_RAPreambleContainer->clear();
      delete m_RAPreambleContainer;

      m_RAPreambleContainer = RAPreamblesContainerTemp;
    }
  }
  else
  {
    int qtd = 0;

    m_current_pdcch_cces = GetDevice()->GetPhy()->GetBandwidthManager()->GetPDCCHTotalCces();

    if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format))
    {

      std::vector<RAPreambleIdealControlMessage *> *RAPreamblesContainerTemp = new std::vector<RAPreambleIdealControlMessage *>;

      RAResponseIdealControlMessage *RAResponse = NULL;
      for (iter = m_RAPreambleContainer->begin(); iter != m_RAPreambleContainer->end(); iter++)
      {
        if (((*iter)->m_processingTime == 0) && ((*iter)->m_waitingTime < InformationManager::Init()->ra_ResponseWindowSize))
        {
          if ((*iter)->getCollision() == false)
          {
            RAResponse = new RAResponseIdealControlMessage((*iter)->getRARNTI(), InformationManager::Init()->backoffIndicatorGeneral, InformationManager::Init()->backoffIndicatorHTC, InformationManager::Init()->backoffIndicatorMTC);
            RAResponse->SetSourceDevice(GetDevice());
            m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
            break;
          }
        }
      }

      for (iter = m_RAPreambleContainer->begin(); iter != m_RAPreambleContainer->end(); iter++)
      {
        if ((RAResponse != NULL) && (qtd < 3))
        {
          if (((*iter)->m_processingTime == 0) && ((*iter)->m_waitingTime <= InformationManager::Init()->ra_ResponseWindowSize) && ((*iter)->getRARNTI() == RAResponse->getRARNTI()) && ((*iter)->getCollision() == false))
          {
            RAResponse->AddNewRecord((*iter)->GetPreamble());
            m_allocated_ul_rbs++;
            qtd++;
          }
          else
          {
            if ((*iter)->m_waitingTime < InformationManager::Init()->ra_ResponseWindowSize)
            {
              RAPreamblesContainerTemp->push_back(*iter);
            }
          }
        }
        else
        {
          if ((*iter)->m_waitingTime < InformationManager::Init()->ra_ResponseWindowSize)
          {
            RAPreamblesContainerTemp->push_back(*iter);
          }
        }
      }
    }
  }
}

void EnbMacEntity::SendRandomAccessResponseIdealControlMessage(RAResponseIdealControlMessage *RAResponse)
{
  std::cerr << Simulator::Init()->Now()
            << " ENodeB " << GetDevice()->GetIDNetworkNode()
            << " sent RA_Response with " << RAResponse->GetMessage()->size()
            << " grant(s)" << std::endl;
  GetDevice()->GetPhy()->SendIdealControlMessage(RAResponse);
}

void EnbMacEntity::ReceiveConnectionRequestIdealControlMessage(ConnectionRequestIdealControlMessage *connectionRequest)
{
  std::vector<ConnectionRequestIdealControlMessage *>::iterator iter;

  bool collision = false;

  for (iter = m_ConnectionRequestContainer->begin(); iter != m_ConnectionRequestContainer->end(); iter++)
  {

    if (InformationManager::Init()->harq_method == 1)
    {
      if (((*iter)->getRARNTI() == connectionRequest->getRARNTI()) && ((*iter)->getResourceBlock() == connectionRequest->getResourceBlock()))
      {

        // Should be a collision
        collision = true;
        (*iter)->setCollision(true);

        if (connectionRequest->isFakeTransmission())
        {
          if ((*iter)->isFakeTransmission() == false)
          {
            (*iter)->setCollision(false);
          }
        }

        if ((*iter)->isFakeTransmission())
        {
          collision = false;
        }
      }

      if (connectionRequest->isFakeTransmission())
      {
        collision = true;
      }
    }
    else
    {
      if (((*iter)->getRARNTI() == connectionRequest->getRARNTI()) && ((*iter)->getResourceBlock() == connectionRequest->getResourceBlock()))
      {
        collision = true;
        (*iter)->setCollision(true);
      }
    }
  }

  connectionRequest->setCollision(collision);

  m_ConnectionRequestContainer->push_back(connectionRequest);

  Simulator::Init()->Schedule(0.003, &EnbMacEntity::ReadyConnectionRequestIdealControlMessage, this, connectionRequest);
}

void EnbMacEntity::ReceiveSchedulingRequestIdealControlMessage(SchedulingRequestIdealControlMessage *schedulingRequest)
{
  std::vector<SchedulingRequestIdealControlMessage *>::iterator iter;

  bool collision = false;

  for (iter = m_SchedulingRequestContainer->begin(); iter != m_SchedulingRequestContainer->end(); iter++)
  {
    if (((*iter)->getRARNTI() == schedulingRequest->getRARNTI()) && ((*iter)->getResourceBlock() == schedulingRequest->getResourceBlock()))
    {
      collision = true;
      (*iter)->setCollision(true);
    }
  }

  // /!\ BROKEN: The processing below must be done after concluding collision analysis /!\

  schedulingRequest->setCollision(collision);

  if ((schedulingRequest->getError() == true) || (schedulingRequest->getCollision() == true))
  {
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " dropped Scheduling_Request from UE " << schedulingRequest->GetSourceDevice()->GetIDNetworkNode() << std::endl;
  }
  else
  {
    if (InformationManager::Init()->ra_Method == 3)
    {
      m_unsuccessful_ra_counter += schedulingRequest->getOverloadIndicator();
      m_total_ra_counter += (schedulingRequest->getOverloadIndicator() + 1);
    }

    if ((InformationManager::Init()->ra_Method == 4) && (schedulingRequest->getSensitive()))
    {
      m_unsuccessful_ra_counter += schedulingRequest->getOverloadIndicator();
      m_total_ra_counter += (schedulingRequest->getOverloadIndicator() + 1);
    }

    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " received Scheduling_Request from UE " << schedulingRequest->GetSourceDevice()->GetIDNetworkNode() << std::endl;
  }

  m_SchedulingRequestContainer->push_back(schedulingRequest);

  Simulator::Init()->Schedule(0.003, &EnbMacEntity::ReadySchedulingRequestIdealControlMessage1, this, schedulingRequest);
}

void EnbMacEntity::ReadyConnectionRequestIdealControlMessage(ConnectionRequestIdealControlMessage *connectionRequest)
{
  if ((connectionRequest->getError() == true) || (connectionRequest->getCollision() == true))
  {
    // A fake transmission is read as a collision
    std::cerr << Simulator::Init()->Now()
              << " ENodeB " << GetDevice()->GetIDNetworkNode()
              << " dropped Connection_Request from UE " << connectionRequest->GetSourceDevice()->GetIDNetworkNode()
              << " due to ";

    if (connectionRequest->getError())
    {
      std::cerr << " ERROR ";
    }

    if (connectionRequest->isFakeTransmission())
    {
      std::cerr << " FAKETX ";
    }
    else if (connectionRequest->getCollision())
    {
      std::cerr << " TRUE_COLLISION";
    }

    std::cerr << std::endl;

    if (InformationManager::Init()->harq_method == 1)
    {
      bool fakeack = false;
      std::vector<ConnectionRequestIdealControlMessage *>::iterator iter;
      std::cerr << Simulator::Init()->Now() << "[DEBUG-FAKEACK] REPORT OF ConnectionRequestContainer" << std::endl;
      for (iter = m_ConnectionRequestContainer->begin(); iter != m_ConnectionRequestContainer->end(); iter++)
      {
        std::cerr << "\tSourceUE " << (*iter)->GetSourceDevice()->GetIDNetworkNode()
                  << " isfake " << (*iter)->isFakeTransmission()
                  << " isCollision " << (*iter)->getCollision()
                  << " isError " << (*iter)->getError()
                  << std::endl;
      }

      // Deduce if the receiving end will read a ack
      if (connectionRequest->isFakeTransmission() == true)
      {
        std::vector<ConnectionRequestIdealControlMessage *>::iterator iter;
        for (iter = m_ConnectionRequestContainer->begin(); iter != m_ConnectionRequestContainer->end(); iter++)
        {

          // For the transmissions which I might collide
          if (((*iter)->getRARNTI() == connectionRequest->getRARNTI()) && ((*iter)->getResourceBlock() == connectionRequest->getResourceBlock()))
          {

            if ((*iter)->isFakeTransmission() == false)
            {

              if ((*iter)->getCollision() == false && (*iter)->getError() == false)
              {
                fakeack = true;
              }
            }
          }
        }

        if (fakeack == true)
        {
          std::cerr << Simulator::Init()->Now()
                    << " [DEBUG-RA-FAKEACK] UE " << connectionRequest->GetSourceDevice()->GetIDNetworkNode()
                    << " is getting its fakeack"
                    << std::endl;

          connectionRequest->setAsFakeAck();
        }
      }
    }
  }
  else
  {
    m_unsuccessful_ra_counter += connectionRequest->getOverloadIndicator();
    m_total_ra_counter += (connectionRequest->getOverloadIndicator() + 1);

    if (InformationManager::Init()->ra_Method == 3)
    {
      m_unsuccessful_ra_counter += connectionRequest->getOverloadIndicator();
      m_total_ra_counter += (connectionRequest->getOverloadIndicator() + 1);
    }

    if ((InformationManager::Init()->ra_Method == 4) && (connectionRequest->getSensitive()))
    {
      m_unsuccessful_ra_counter += connectionRequest->getOverloadIndicator();
      m_total_ra_counter += (connectionRequest->getOverloadIndicator() + 1);
    }

    std::cerr << Simulator::Init()->Now()
              << " ENodeB " << GetDevice()->GetIDNetworkNode()
              << " received Connection_Request from UE "
              << connectionRequest->GetSourceDevice()->GetIDNetworkNode()
              << std::endl;
  }

  if ((connectionRequest->getCollision() == false) && (connectionRequest->getError() == false))
  {
    // TODO: ???
    // type = 1;
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " sent ACK to UE " << connectionRequest->GetSourceDevice()->GetIDNetworkNode() << std::endl;
    HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(1, true);
    HARQ->SetSourceDevice(GetDevice());
    HARQ->SetDestinationDevice(connectionRequest->GetSourceDevice());

    GetDevice()->GetPhy()->SendIdealControlMessage(HARQ);

    MsgToSchedule *msg = new MsgToSchedule();
    msg->m_timeArrived = Simulator::Init()->Now() - 0.004;
    msg->m_destination = connectionRequest->GetSourceDevice();
    msg->m_scheduled = false;
    msg->m_ar = connectionRequest->GetHoLTime();
    msg->m_delay = connectionRequest->GetMaxDelay();

    m_msgs4ToSchedule->push_back(msg);
  }
  else
  {
    // type = 2;
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " sent NACK to UE " << connectionRequest->GetSourceDevice()->GetIDNetworkNode() << std::endl;

    HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(2, true);
    HARQ->SetSourceDevice(GetDevice());
    HARQ->SetDestinationDevice(connectionRequest->GetSourceDevice());

    if (InformationManager::Init()->harq_method == 1)
    {
      if (connectionRequest->isFakeAck() == true)
      {
        std::cerr << Simulator::Init()->Now()
                  << " [DEBUG-RA-FAKEACK] SETTING HARQ FAKEACK for UE "
                  << connectionRequest->GetSourceDevice()->GetIDNetworkNode()
                  << std::endl;
        HARQ->setAsFakeAck();
      }

      double p = 1; // TODO: Set probabiliy of cancelling harq
      if (Random::Init()->Uniform(0.0, 1.0) >= p)
      {
        // Cancelling HARQ
        HARQ->cancelHARQ();

        std::cerr << Simulator::Init()->Now()
                  << " [DEBUG-RA-HARQ] UE " << connectionRequest->GetSourceDevice()->GetIDNetworkNode()
                  << " got HARQ process cancelled @ENB  "
                  << std::endl;
      }
      else
      {
        AllocationResourceToMsg3Retransmission();
      }

      GetDevice()->GetPhy()->SendIdealControlMessage(HARQ);
    }
    else
    { // NOT PROBABILISTIC HARQ
      AllocationResourceToMsg3Retransmission();
      GetDevice()->GetPhy()->SendIdealControlMessage(HARQ);
    }
  }

  connectionRequest->setIsQueue(true);
}

void EnbMacEntity::ReadySchedulingRequestIdealControlMessage()
{
  std::vector<SchedulingRequestIdealControlMessage *> *SchedulingRequestContainerTemp = new std::vector<SchedulingRequestIdealControlMessage *>;
  std::vector<SchedulingRequestIdealControlMessage *>::iterator iter;

  int type;
  int qtd = 0;

  for (iter = m_SchedulingRequestContainer->begin(); iter != m_SchedulingRequestContainer->end(); iter++)
  {
    (*iter)->m_waitingTime++;
    if ((*iter)->m_waitingTime <= InformationManager::Init()->mac_ContentionResolutionTimer)
    {
      if ((*iter)->m_processingTime == 0)
      {
        if (((*iter)->getCollision() == false) && ((*iter)->getError() == false))
        {
          type = 1;
        }
        else
        {
          type = 2;
        }

        if ((*iter)->getRespHARQ() == false)
        {
          HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(type, true);
          HARQ->SetSourceDevice(GetDevice());
          HARQ->SetDestinationDevice((*iter)->GetSourceDevice());
          if (type == 1)
          {
            std::cerr << Simulator::Init()->Now() << " MAC@ENodeB " << GetDevice()->GetIDNetworkNode() << " sent ACK to UE " << HARQ->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
          }
          else
          {
            std::cerr << Simulator::Init()->Now() << " MAC@ENodeB " << GetDevice()->GetIDNetworkNode() << " sent NACK to UE " << HARQ->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            // Simulator::Init()->Schedule((double) (0.005, &EnbMacEntity::EABChangeClassBarred, this);
            AllocationResourceToMsg3Retransmission();
          }
          (*iter)->setRespHARQ(true);
          GetDevice()->GetPhy()->SendIdealControlMessage(HARQ);
        }

        if (type == 1)
        {
          if (ONLY_RANDOM_ACCESS == true)
          {
            if (m_current_pdcch_cces >= 4)
            {
              Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyContentionResolutionIdealControlMessage, this, (*iter)->GetSourceDevice());
              m_current_pdcch_cces -= 4;
            }
            else
            {
              SchedulingRequestContainerTemp->push_back(*iter);
            }
          }
          else
          {
            if (m_current_pdcch_cces >= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format) && qtd < 2)
            {
              qtd++;
              m_current_pdcch_cces -= InformationManager::Init()->GetPDCCHNumberOfCCEs(InformationManager::Init()->m_pdcch_format);
              Simulator::Init()->Schedule(0.000, &EnbMacEntity::AssemblyContentionResolutionIdealControlMessage, this, (*iter)->GetSourceDevice());
            }
            else
            {
              SchedulingRequestContainerTemp->push_back(*iter);
            }
          }
        }
      }
      else
      {
        (*iter)->m_processingTime--;
        SchedulingRequestContainerTemp->push_back(*iter);
      }
    }
  }

  // m_SchedulingRequestContainer->clear();
  // delete m_SchedulingRequestContainer;

  // m_SchedulingRequestContainer = SchedulingRequestContainerTemp;

  ////Simulator::Init()->Schedule(0.001, &EnbMacEntity::ReadyConnectionRequestIdealControlMessage, this);
}

void EnbMacEntity::AssemblyContentionResolutionIdealControlMessage(NetworkNode *destination)
{
  ContentionResolutionIdealControlMessage *contentionResolution = new ContentionResolutionIdealControlMessage();
  contentionResolution->SetSourceDevice(GetDevice());
  contentionResolution->SetDestinationDevice(destination);

  // m_allocated_ul_rbs++; //allocated uplink resource block for BSR

  std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " sent Contention_Resolution to UE " << contentionResolution->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
  GetDevice()->GetPhy()->SendIdealControlMessage(contentionResolution);
}

/*
 * Ciclo de monitoramento RACH Overload
 */
void EnbMacEntity::TimeoutMonitoringCycle()
{
  // double Pc, alpha = 0.25;

  // if (m_total_ra_counter == 0) {
  //   Pc = 0;
  // } else {
  //   Pc = (double) m_unsuccessful_ra_counter / (double) m_total_ra_counter;
  // }

  // std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " measure Total Preambles " << tt_pream << " " << FrameManager::Init()->GetRACHCounter() << std::endl;

  // m_unsuccessful_ra_counter = 0;
  // m_total_ra_counter = 0;

  // tt_pream = 0;

  // FrameManager::Init()->ResetRACHCounter();

  // Simulator::Init()->Schedule((double) (InformationManager::Init()->T_Monitoring_Cycle / (double) 1000), &EnbMacEntity::TimeoutMonitoringCycle, this);

  // std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " Collision Probability " << (double) tt_pream / (double) (FrameManager::Init()->GetRACHCounter() * InformationManager::Init()->numberOfRA_Preambles) << std::endl;

  // tt_pream = 0;

  m_information_changed = false;

  if ((InformationManager::Init()->ra_Method == 2) && (InformationManager::Init()->acb_Dynamic == true))
  {
    // double p = (double) tt_pream / (double) (FrameManager::Init()->GetRACHCounter() * InformationManager::Init()->numberOfRA_Preambles);
    double p = 0;

    std::vector<RAPreambleIdealControlMessage *>::iterator it;
    std::vector<RAPreambleIdealControlMessage *>::iterator jt;

    int distinct_preambles = 0;
    double L = (double)InformationManager::Init()->numberOfRA_Preambles;

    for (it = m_RAPreambleContainer->begin(); it != m_RAPreambleContainer->end(); it++)
    {
      bool new_preamble = true;

      for (jt = m_RAPreambleContainer->begin(); jt != it; jt++)
      {
        if ((*jt)->GetPreamble() == (*it)->GetPreamble())
        {
          new_preamble = false;
        }
      }
      if (new_preamble)
      {
        distinct_preambles += 1;
      }
    }
    double I = L - distinct_preambles;
    double ue_estimative = L * std::log(L / I);

    p = L / ue_estimative;

    p = (p < 1 ? p : 1);

    std::cerr << Simulator::Init()->Now()
              << " ENodeB " << GetDevice()->GetIDNetworkNode()
              << " ACB Probability " << p << std::endl;

    // InformationManager::Init()->ac_BarringFactor = 1 - ((double) tt_pream / (double) (FrameManager::Init()->GetRACHCounter() * InformationManager::Init()->numberOfRA_Preambles));
    InformationManager::Init()->ac_BarringFactor = p;

    tt_pream = 0;
    Simulator::Init()->Schedule((double)(InformationManager::Init()->acb_MonitoringPeriod / (double)1000), &EnbMacEntity::TimeoutMonitoringCycle, this);
  }
  else if (InformationManager::Init()->ra_Method == 4)
  {
    double Pc, alpha = 0.25;

    if (m_total_ra_counter == 0)
    {
      Pc = 0;
    }
    else
    {
      Pc = (double)m_unsuccessful_ra_counter / (double)m_total_ra_counter;
    }

    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " measure Collision Probability " << Pc << std::endl;
    if (Pc <= alpha)
    {
      InformationManager::Init()->noSensitive_Blocked = false;
    }
    else
    {
      InformationManager::Init()->noSensitive_Blocked = true;
    }

    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " sent Update_Informations" << std::endl;

    m_unsuccessful_ra_counter = 0;
    m_total_ra_counter = 0;

    Simulator::Init()->Schedule((double)(InformationManager::Init()->QoSDracon_MonitoringPeriod / (double)1000), &EnbMacEntity::TimeoutMonitoringCycle, this);
  }
  else if (InformationManager::Init()->ra_Method == 6)
  {
    if (InformationManager::Init()->eab_Dynamic == true)
    {
      double Cc = 0;
      if (m_preamble_transmissions > 0)
      {
        Cc = (double)1 - ((double)m_rar_sent / (double)m_preamble_transmissions);
      }

      if (Cc < 0)
      {
        Cc = 0;
      }

      std::cout << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " measure Congestion Coefficient " << Cc << " " << m_rar_sent << " " << m_preamble_transmissions << std::endl;

      m_rar_sent = 0;
      m_preamble_transmissions = 0;

      if (Cc >= 0.4)
      {
        if (m_eab_on == false)
        {
          m_eab_on = true;
          InformationManager::Init()->eab_on = m_eab_on;
          m_eab_index = -1;

          EABChangeClassBarred();
        }

        // Simulator::Init()->Schedule(InformationManager::Init()->eab_MonitoringPeriod, &EnbMacEntity::TimeoutMonitoringCycle, this);
      }
      else if (Cc < 0.4)
      {
        if (m_eab_on == true)
        {

          EABChangeClassBarred();

          /*if (m_eab_on) {
            Simulator::Init()->Schedule(0.500, &EnbMacEntity::TimeoutMonitoringCycle, this);
          } else {
            Simulator::Init()->Schedule(0.500, &EnbMacEntity::TimeoutMonitoringCycle, this);
          }*/
          //} else {
          // Simulator::Init()->Schedule(0.500, &EnbMacEntity::TimeoutMonitoringCycle, this);
        }
      }

      Simulator::Init()->Schedule((double)(InformationManager::Init()->eab_MonitoringPeriod / (double)1000), &EnbMacEntity::TimeoutMonitoringCycle, this);
    }
    else
    {

      EABChangeClassBarred();

      Simulator::Init()->Schedule((double)(InformationManager::Init()->eab_MonitoringPeriod / (double)1000), &EnbMacEntity::TimeoutMonitoringCycle, this);
    }
  }

  FrameManager::Init()->ResetRACHCounter();
}

/*
 * Implementa a funcao para mudar a classe permitida de forma ciclica no EAB
 */
void EnbMacEntity::EABChangeClassBarred()
{
  int index;

  if (InformationManager::Init()->eab_Dynamic == true)
  {
    index = 0;
    while (index < 10)
    {
      InformationManager::Init()->eab_BarringBitmap[index++] = 1;
    }

    InformationManager::Init()->eab_BarringBitmap[(++m_eab_index) % 10] = 0;

    if (m_eab_index == 10)
    {
      m_eab_on = false;
      InformationManager::Init()->eab_on = m_eab_on;
      m_eab_index = -1;

      index = 0;
      while (index < 10)
      {
        InformationManager::Init()->eab_BarringBitmap[index++] = 0;
      }
    }
  }
  else
  {
    index = 0;
    while (index < 10)
    {
      InformationManager::Init()->eab_BarringBitmap[index++] = 1;
    }

    InformationManager::Init()->eab_BarringBitmap[(++m_eab_index) % 10] = 0;
  }

  m_information_changed = true;

  index = 0;
  while (index < 10)
  {
    std::cout << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " Access_Class " << index << " " << InformationManager::Init()->eab_BarringBitmap[index++] << std::endl;
  }

  // std::cout << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " changed to Access_Class " << m_eab_index % 10 << std::endl;
}

int EnbMacEntity::getCurrentPDCCHCCEs(void)
{
  return m_current_pdcch_cces;
}

void EnbMacEntity::updateCurrentPDCCHCCEs(int cces)
{
  m_current_pdcch_cces = cces;
}

int EnbMacEntity::GetRealNbOfRBsForULScheduling()
{
  int nRBsToRACH = 0;
  if (FrameManager::Init()->IsRACHSubframe() == true)
  {
    nRBsToRACH = 6;
  }
  return GetDevice()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size() - (m_allocated_ul_rbs + m_allocated_ul_prbs_msg3_retransmition + nRBsToRACH);
}

void EnbMacEntity::ResetAllocatedULRBs()
{
  m_allocated_ul_rbs = 0;
  m_allocated_ul_prbs_msg3_retransmition = 0;
}

void EnbMacEntity::AllocationResourceToMsg3Retransmission()
{
  // std::cerr << Simulator::Init()->Now() << " Allocation Retransmission eNB" << std::endl;
  // m_allocated_ul_rbs++;
  m_allocated_ul_prbs_msg3_retransmition++;
}

void EnbMacEntity::AllocationResourceToDataTransmission()
{
  // std::cerr << Simulator::Init()->Now() << " Allocation Transmission eNB" << std::endl;
  m_allocated_ul_rbs++;
}

void EnbMacEntity::AllocationResourceToDataTransmission(int type)
{
  // std::cerr << Simulator::Init()->Now() << " Allocation Transmission eNB" << std::endl;
  m_allocated_ul_rbs++;

  if (type == 1)
  {
    m_rar_sent++;
  }

  // std::cout << Simulator::Init()->Now() << " RARs Sent " << m_rar_sent << std::endl;
}

void EnbMacEntity::ReadySchedulingRequestIdealControlMessage1(SchedulingRequestIdealControlMessage *schedulingRequest)
{
  // std::cerr << Simulator::Init()->Now() << " ENodeB ACK" << std::endl;

  // int type;

  // HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(type, true);
  // HARQ->SetSourceDevice(GetDevice());
  // HARQ->SetDestinationDevice(schedulingRequest->GetSourceDevice());

  if ((schedulingRequest->getCollision() == false) && (schedulingRequest->getError() == false))
  {
    // type = 1;
    std::cerr << Simulator::Init()->Now()
              << " ENodeB " << GetDevice()->GetIDNetworkNode()
              << " sent ACK to UE " << schedulingRequest->GetSourceDevice()->GetIDNetworkNode()
              << std::endl;

    HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(1, true);
    HARQ->SetSourceDevice(GetDevice());
    HARQ->SetDestinationDevice(schedulingRequest->GetSourceDevice());

    GetDevice()->GetPhy()->SendIdealControlMessage(HARQ);

    MsgToSchedule *msg = new MsgToSchedule();
    msg->m_timeArrived = Simulator::Init()->Now() - 0.004;
    msg->m_destination = schedulingRequest->GetSourceDevice();
    msg->m_scheduled = false;
    msg->m_ar = schedulingRequest->GetHoLTime();
    msg->m_delay = schedulingRequest->GetMaxDelay();

    m_msgs4ToSchedule->push_back(msg);
  }
  else
  {
    // type = 2;
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " sent NACK to UE " << schedulingRequest->GetSourceDevice()->GetIDNetworkNode() << std::endl;

    HarqIdealControlMessage *HARQ = new HarqIdealControlMessage(2, true);
    HARQ->SetSourceDevice(GetDevice());
    HARQ->SetDestinationDevice(schedulingRequest->GetSourceDevice());

    AllocationResourceToMsg3Retransmission();

    GetDevice()->GetPhy()->SendIdealControlMessage(HARQ);
  }

  schedulingRequest->setIsQueue(true);
}

void EnbMacEntity::ReadyRandomAccessPreambleIdealControlMessage1(RAPreambleIdealControlMessage *RAPreamble)
{
  std::cerr << Simulator::Init()->Now()
            << " [DEBUG-RA] @ENodeB: Preamble_Collision " << RAPreamble->getCollision()
            << " from UE " << RAPreamble->GetSourceDevice()->GetIDNetworkNode()
            << " #Preambles_received " << m_RAPreambleContainer->size()
            << std::endl;

  RAPreamble->setIsQueue(true);

  UserEquipment *tmp_ue = ((UserEquipment *)RAPreamble->GetSourceDevice());
  UeMacEntity *tmp_ue_mac = (UeMacEntity *)tmp_ue->GetProtocolStack()->GetMacEntity();

  tmp_ue_mac->m_temp_enb_users_trying_RA = m_RAPreambleContainer->size();
  tmp_ue_mac->m_temp_enb_preambles_collided = 0;
  tmp_ue_mac->m_temp_enb_preambles_distinct = 0;

  std::vector<RAPreambleIdealControlMessage *>::iterator it;
  std::vector<RAPreambleIdealControlMessage *>::iterator jt;
  int ct = 0;
  for (it = m_RAPreambleContainer->begin(); it != m_RAPreambleContainer->end(); it++)
  {
    if (RAPreamble->getRARNTI() == (*it)->getRARNTI())
    {
      ct++;
    }
  }

  ct = (ct > 15 ? 15 : ct);
  tmp_ue_mac->setCounterMatchedRA_RNTI(ct);

  std::cerr << Simulator::Init()->Now()
            << " [DEBUG-RA-CT] UE " << tmp_ue->GetIDNetworkNode()
            << " : got RARNTI counter of " << ct
            << std::endl;

  if (InformationManager::Init()->harq_method_god_mode)
  {

    for (it = m_RAPreambleContainer->begin(); it != m_RAPreambleContainer->end(); it++)
    {
      if ((*it)->getCollision())
      {
        tmp_ue_mac->m_temp_enb_preambles_collided++;
      }
      bool new_preamble = true;
      for (jt = m_RAPreambleContainer->begin(); jt != it; jt++)
      {
        if ((*jt)->GetPreamble() == (*it)->GetPreamble())
        {
          new_preamble = false;
        }
      }
      if (new_preamble)
      {
        tmp_ue_mac->m_temp_enb_preambles_distinct++;
      }
    }

    std::cerr << Simulator::Init()->Now()
              << " [DEBUG-RA-IDEAL] Total_Preambles_Sent " << m_RAPreambleContainer->size()
              << " Preambles_Collided " << tmp_ue_mac->m_temp_enb_preambles_collided
              << " Distinct preambles " << tmp_ue_mac->m_temp_enb_preambles_distinct
              << std::endl;
  }

  if (!RAPreamble->getCollision())
  {
    MsgToSchedule *msg = new MsgToSchedule();
    msg->m_RARNTI = RAPreamble->getRARNTI();
    msg->m_preamble = RAPreamble->GetPreamble();
    msg->m_timeArrived = Simulator::Init()->Now();
    msg->m_destination = RAPreamble->GetSourceDevice();
    msg->m_scheduled = false;

    m_msgs2ToSchedule->push_back(msg);

    // if (!collision) {
    // tt_pream++;
    // }
  }
}

EnbMacEntity::MSGsToSchedule *
EnbMacEntity::getMSGs2ToSchedule()
{
  return m_msgs2ToSchedule;
}

EnbMacEntity::MSGsToSchedule *
EnbMacEntity::getMSGs4ToSchedule()
{
  return m_msgs4ToSchedule;
}

int EnbMacEntity::getAllocatedResourcesToMsg3ReTX()
{
  return m_allocated_ul_prbs_msg3_retransmition;
}

void EnbMacEntity::CleanRandomAccessMessagensContainer()
{
  std::vector<RAPreambleIdealControlMessage *> *RandomAccessPreamblesContainerTmp = new std::vector<RAPreambleIdealControlMessage *>;
  std::vector<RAPreambleIdealControlMessage *>::iterator it;
  for (it = m_RAPreambleContainer->begin(); it != m_RAPreambleContainer->end(); it++)
  {
    if (!(*it)->getIsQueue())
    {
      RandomAccessPreamblesContainerTmp->push_back(*it);
    }
    else
    {
      delete *it;
    }
  }
  m_RAPreambleContainer->clear();
  delete m_RAPreambleContainer;

  m_RAPreambleContainer = RandomAccessPreamblesContainerTmp;

  std::vector<SchedulingRequestIdealControlMessage *> *SchedulingRequestContainerTemp = new std::vector<SchedulingRequestIdealControlMessage *>;
  std::vector<SchedulingRequestIdealControlMessage *>::iterator SRIterator;
  for (SRIterator = m_SchedulingRequestContainer->begin(); SRIterator != m_SchedulingRequestContainer->end(); SRIterator++)
  {
    if (!(*SRIterator)->getIsQueue())
    {
      SchedulingRequestContainerTemp->push_back(*SRIterator);
    }
    else
    {
      delete *SRIterator;
    }
  }
  m_SchedulingRequestContainer->clear();
  delete m_SchedulingRequestContainer;

  m_SchedulingRequestContainer = SchedulingRequestContainerTemp;

  std::vector<ConnectionRequestIdealControlMessage *> *ConnectionRequestContainerTemp = new std::vector<ConnectionRequestIdealControlMessage *>;
  std::vector<ConnectionRequestIdealControlMessage *>::iterator CRIterator;
  for (CRIterator = m_ConnectionRequestContainer->begin(); CRIterator != m_ConnectionRequestContainer->end(); CRIterator++)
  {
    if (!(*CRIterator)->getIsQueue())
    {
      ConnectionRequestContainerTemp->push_back(*CRIterator);
    }
    else
    {
      delete *CRIterator;
    }
  }
  m_ConnectionRequestContainer->clear();
  delete m_ConnectionRequestContainer;

  m_ConnectionRequestContainer = ConnectionRequestContainerTemp;
}

void EnbMacEntity::CleanRandomAccessMessagesQueue()
{
  MSGsToSchedule::iterator iter;

  MSGsToSchedule *MSGs2ToScheduleQueueTmp = new MSGsToSchedule;
  for (iter = m_msgs2ToSchedule->begin(); iter != m_msgs2ToSchedule->end(); iter++)
  {
    if (!(*iter)->m_scheduled)
    {
      MSGs2ToScheduleQueueTmp->push_back(*iter);
    }
    else
    {
      delete *iter;
    }
  }
  m_msgs2ToSchedule->clear();
  delete m_msgs2ToSchedule;

  m_msgs2ToSchedule = MSGs2ToScheduleQueueTmp;

  MSGsToSchedule *MSGs4ToScheduleQueueTmp = new MSGsToSchedule;
  for (iter = m_msgs4ToSchedule->begin(); iter != m_msgs4ToSchedule->end(); iter++)
  {
    if (!(*iter)->m_scheduled)
    {
      MSGs4ToScheduleQueueTmp->push_back(*iter);
    }
    else
    {
      delete *iter;
    }
  }
  m_msgs4ToSchedule->clear();
  delete m_msgs4ToSchedule;

  m_msgs4ToSchedule = MSGs4ToScheduleQueueTmp;

  MSGsToSchedule *bsrGrantsToScheduleQueueTmp = new MSGsToSchedule;
  for (iter = m_bsrGrantsToSchedule->begin(); iter != m_bsrGrantsToSchedule->end(); iter++)
  {
    if (!(*iter)->m_scheduled)
    {
      bsrGrantsToScheduleQueueTmp->push_back(*iter);
    }
    else
    {
      delete *iter;
    }
  }
  m_bsrGrantsToSchedule->clear();
  delete m_bsrGrantsToSchedule;

  m_bsrGrantsToSchedule = bsrGrantsToScheduleQueueTmp;
}

void EnbMacEntity::CheckForDelayedMSGs2OfRandomAccess()
{
  MSGsToSchedule::iterator iter;

  double now = Simulator::Init()->Now();
  double HoL = 0;

  while (getMSGs2ToSchedule()->size() > 0)
  {
    iter = getMSGs2ToSchedule()->begin();
    HoL = now - (*iter)->m_timeArrived;
    if ((HoL * 1000) >= InformationManager::Init()->ra_ResponseWindowSize)
    {
      std::cerr << Simulator::Init()->Now() << " DROP Msg2 " << (*iter)->m_RARNTI << " " << (*iter)->m_timeArrived << " " << (*iter)->m_preamble << " timeout" << std::endl;
      getMSGs2ToSchedule()->pop_front();
      delete *iter;
    }
    else
    {
      break;
    }
  }
}

void EnbMacEntity::CheckForDelayedMSGs4OfRandomAccess()
{
  MSGsToSchedule::iterator iter;

  double now = Simulator::Init()->Now();
  double HoL = 0;

  while (getMSGs4ToSchedule()->size() > 0)
  {
    iter = getMSGs4ToSchedule()->begin();
    HoL = now - (*iter)->m_timeArrived;
    if ((HoL * 1000) >= InformationManager::Init()->mac_ContentionResolutionTimer - 3)
    {
      std::cerr << Simulator::Init()->Now() << " DROP Msg4 " << (*iter)->m_RARNTI << " timeout" << std::endl;
      getMSGs4ToSchedule()->pop_front();
      delete *iter;
    }
    else
    {
      break;
    }
  }
}

bool EnbMacEntity::GetInformationChanged()
{
  return m_information_changed;
}

void EnbMacEntity::ReceiveSchedulingRequestPUCCHIdealControlMessage(SchedulingRequestPUCCHIdealControlMessage *schedulingRequestMessage)
{
  if (schedulingRequestMessage->getError() == true)
  {
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " dropped Scheduling_Request from UE " << schedulingRequestMessage->GetSourceDevice()->GetIDNetworkNode() << std::endl;
    delete schedulingRequestMessage;
  }
  else
  {
    std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " received Scheduling_Request from UE " << schedulingRequestMessage->GetSourceDevice()->GetIDNetworkNode() << std::endl;
    Simulator::Init()->Schedule(0.003, &EnbMacEntity::ReadySchedulingRequestPUCCHIdealControlMessage, this, schedulingRequestMessage);
  }
}

void EnbMacEntity::ReadySchedulingRequestPUCCHIdealControlMessage(SchedulingRequestPUCCHIdealControlMessage *schedulingRequestMessage)
{
  MsgToSchedule *msg = new MsgToSchedule();
  msg->m_timeArrived = Simulator::Init()->Now() - 0.004;
  msg->m_destination = schedulingRequestMessage->GetSourceDevice();
  msg->m_scheduled = false;
  msg->m_ar = schedulingRequestMessage->GetHoLTime();
  msg->m_delay = schedulingRequestMessage->GetMaxDelay();

  m_bsrGrantsToSchedule->push_back(msg);
}

EnbMacEntity::MSGsToSchedule *
EnbMacEntity::GetBsrGrantsToSchedule()
{
  return m_bsrGrantsToSchedule;
}

void EnbMacEntity::AssemblyBsrGrantIdealControlMessage(NetworkNode *destination)
{
  BsrGrantIdealControlMessage *bsrGrantMessage = new BsrGrantIdealControlMessage();
  bsrGrantMessage->SetSourceDevice(GetDevice());
  bsrGrantMessage->SetDestinationDevice(destination);

  std::cerr << Simulator::Init()->Now() << " ENodeB " << GetDevice()->GetIDNetworkNode() << " sent BSR_Grant to UE " << bsrGrantMessage->GetDestinationDevice()->GetIDNetworkNode() << std::endl;

  GetDevice()->GetPhy()->SendIdealControlMessage(bsrGrantMessage);
}

void EnbMacEntity::ProcessPreamblesReceived(void)
{
  std::cout << Simulator::Init()->Now() << " Process Preamble " << std::endl;

  for (int i = 0; i < 64; i++)
  {
    if (m_preamblesReceived[i] > 1)
    {
      std::cout << Simulator::Init()->Now() << " ENodeB Collision Preamble " << i << " " << m_preamblesReceived[i] << std::endl;
      tt_pream++;
    }
    m_preamblesReceived[i] = 0;
  }

  Simulator::Init()->Schedule(0.001, &EnbMacEntity::ProcessPreamblesReceived, this);
}