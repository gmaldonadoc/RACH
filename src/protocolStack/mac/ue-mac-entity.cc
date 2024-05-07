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

#include <fstream>

#include "ue-mac-entity.h"

#include "AMCModule.h"
#include "../packet/Packet.h"
#include "../packet/packet-burst.h"
#include "../../core/idealMessages/ideal-control-messages.h"
#include "../../device/NetworkNode.h"
#include "../../device/UserEquipment.h"
#include "../../device/ENodeB.h"
#include "../../phy/lte-phy.h"
#include "../../protocolStack/rrc/rrc-entity.h"
#include "../../flows/radio-bearer.h"
#include "../../flows/application/Application.h"
#include "../../flows/MacQueue.h"
#include "../../flows/QoS/QoSParameters.h"
#include "../rlc/am-rlc-entity.h"

#include "../../flows/application/WEB.h"
#include "../../componentManagers/FrameManager.h"
#include "../../componentManagers/InformationManager.h"

#include "../../energyConsumption/ue-consumption-model.h"
#include "../../drx/drx-manager.h"

UeMacEntity::UeMacEntity() {
    SetAmcModule(new AMCModule());
    SetDevice(NULL);

    m_hasSchedulingRequestWaiting = false;
    m_isSchedulingRequestProhibitPeriod = false;

    m_BSRInterval = 0.040;

    m_RAStopped = UPLINK;
    //m_RAStopped = true;

    m_hasPreambleWaiting = false;
    m_raRNTI = -1;
    m_raRNTI_Using = -1;

    m_ContentionResolutionTimerHandler = NULL;
    m_RAResponseWindowTimerHandler = NULL;
    m_retxBSRTimerHandler = NULL;

    m_backoffExponent = 0;
    m_waitToSendPreamble = 0;
    m_retxBSR_Time = 2.560;

    m_triggered_BSR = false;
    m_retxBSRTimer = false;
    m_RAResponseWindowTimer = false;
    m_ContentionResolutionTimer = false;

    m_information_changed = false;
    m_eab_barred = true;

    m_executingRA = false;

    m_preamble = -1;
    m_fixedPreamble = false;

    m_noSensitiveBlocked = InformationManager::Init()->noSensitive_Blocked;

    m_expentAtRAStart = 0;
    m_instantAtStart = 0;

    m_counterMatchedRA_RNTI = 0;
    m_premature_harq_stop = false;

    m_temp_enb_users_trying_RA = 1;
    m_temp_enb_preambles_collided = 0;
    m_temp_enb_preambles_distinct = 1;

    for (int i = 0; i < 8; i++) {
        m_harqBuffer[i] = NULL;
    }

    if ((ONLY_RANDOM_ACCESS == false) && (UPLINK == true)) {
        Simulator::Init()->Schedule(0.000, &UeMacEntity::CheckForDropPackets, this);
        Simulator::Init()->Schedule(GetPeriodicBSRInterval(), &UeMacEntity::TimeoutPeriodicBSRTimer, this);
        Simulator::Init()->Schedule(0.000, &UeMacEntity::StartCheckForPagingMessage, this);
    }

    if (InformationManager::Init()->ra_Method == 4) {
        Simulator::Init()->Schedule((double) (InformationManager::Init()->period_systemInformation) / (double) (1000), &UeMacEntity::CheckForSystemInformation, this);
    } else if (InformationManager::Init()->ra_Method == 6) {
        Simulator::Init()->Schedule((double) (InformationManager::Init()->period_systemInformation) / (double) (1000), &UeMacEntity::CheckForSystemInformation, this);
    }
}

UeMacEntity::~UeMacEntity() {
    Destroy();
}

double
UeMacEntity::GetPeriodicBSRInterval() {
    return m_BSRInterval;
}

void
UeMacEntity::SetPeriodicBSRInterval(double time) {
    m_BSRInterval = time;
}

void
UeMacEntity::TimeoutPeriodicBSRTimer() {
    /* only to H2H devices */
    if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
        TriggerBufferStatusReport(3);
        if (((UserEquipment *) GetDevice())->GetSemiSchEnable() == false) {
            Simulator::Init()->Schedule(GetPeriodicBSRInterval(), &UeMacEntity::TimeoutPeriodicBSRTimer, this);
        }
    }
}

void
UeMacEntity::SendBufferStatusReport(int how) {
    NetworkNode *m_d2dDestination = NULL;

    if (((UserEquipment*) GetDevice())->IsRegister()) {

        int bufferSize = 0;
        RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();

        if (rrc->GetRadioBearerContainer()->size() > 0) {

            for (RrcEntity::RadioBearersContainer::iterator it = rrc->GetRadioBearerContainer()->begin(); it != rrc->GetRadioBearerContainer()->end(); it++) {
                RadioBearer *b = (*it);

                if (b->GetDestination()->GetNodeType() == NetworkNode::TYPE_UE) {
                    std::cout << "D2D Buffer!" << std::endl;
                    m_d2dDestination = b->GetDestination();
                }

                if (b->GetApplication()->GetApplicationType() != Application::APPLICATION_TYPE_INFINITE_BUFFER) {
                    bufferSize += b->GetQueueSize();
                } else {
                    bufferSize += 10000000;
                }
            }

        }

        UserEquipment *thisNode = (UserEquipment*) GetDevice();

        BufferStatusReportingIdealControlMessage *msg = new BufferStatusReportingIdealControlMessage(how);
        msg->SetSourceDevice(thisNode);
        msg->SetDestinationDevice(thisNode->GetTargetNode()); // send to the eNB
        msg->SetBufferStatusReport(bufferSize);

        if (m_d2dDestination) {
            //std::cout << "BUFFER -- " << GetDevice()->GetIDNetworkNode() << " " << m_d2dDestination->GetIDNetworkNode() << std::endl;
            msg->SetDestinationUeD2D(m_d2dDestination);
        }

        //if (how == 1) {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent BSR with " << bufferSize << " byte(s) to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << " by " << how << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent BSR with " << bufferSize << " byte(s) to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << " by " << how << std::endl;
        }
        //}

        GetDevice()->GetPhy()->SendIdealControlMessage(msg);

        if (bufferSize > 0) {
            StartRetxBSRTimer();
        } else {
            m_triggered_BSR = false;
        }
    }
}

void
UeMacEntity::SendSchedulingRequest() {
    if (((UserEquipment *) GetDevice())->GetSemiSchEnable() == true) {
        ENodeB::UserEquipmentRecord *record = ((ENodeB*) ((UserEquipment*) GetDevice())->GetTargetNode())->GetUserEquipmentRecord(GetDevice()->GetIDNetworkNode());

        if (record->m_semiSchTime == 0) {
            record->m_semiWait = 0;
            record->m_semiReliase = 0;
            record->m_semiSchTime = (Simulator::Init()->Now() * 1000) + 1;

            m_triggered_BSR = false;

            Simulator::Init()->Schedule(0.004, &UeMacEntity::ScheduleUplinkTransmissionSemiSch, this);
        }
    } else {
        DRXManager *drxMan = ((UserEquipment*) GetDevice())->GetDRXManager();
        if (((UserEquipment *) GetDevice())->IsSchedulingRequestAllocated() == false) {
            if (drxMan != NULL) {
                if (drxMan->getStatus() == DRXManager::LIGHT || drxMan->getStatus() == DRXManager::DEEP) {
#ifdef DRX_DEBUG
                    //std::cerr << Simulator::Init()->Now() << " [DRX] RandomAccess postponed " << drxMan->GetExpectedOffTime() << " ms" << std::endl;
                    std::cerr << Simulator::Init()->Now() << " [DRX] RandomAccess postponed " << (double) drxMan->GetSleepToActiveLightTimer() / (double) 1000 << " ms" << std::endl;
#endif
                    /* Patent */
                    //Simulator::Init()->Schedule(drxMan->GetExpectedOffTime(), &UeMacEntity::StartRAProcedure, this);
                    /**/

                    /**/
                    drxMan->setStatus(DRXManager::ACTIVE);
                    Simulator::Init()->Schedule((double) drxMan->GetSleepToActiveLightTimer() / (double) 1000, &UeMacEntity::StartRAProcedure, this);
                    /**/

                } else if (drxMan->getStatus() == DRXManager::WAKEUP_LIGHT || drxMan->getStatus() == DRXManager::POWERDOWN_LIGHT || drxMan->getStatus() == DRXManager::WAKEUP_DEEP || drxMan->getStatus() == DRXManager::POWERDOWN_DEEP) {
                    Simulator::Init()->Schedule((double) (drxMan->GetSleepToActiveLightTimer() - drxMan->GetSleepToActiveCounter()) / (double) 1000, &UeMacEntity::StartRAProcedure, this);

                } else {
                    StartRAProcedure();
                }
            } else {
                StartRAProcedure();
            }
        } else {
            if (m_hasSchedulingRequestWaiting == false && m_isSchedulingRequestProhibitPeriod == false) {

                m_hasSchedulingRequestWaiting = true;
                if (drxMan != NULL) {
                    if (drxMan->getStatus() == DRXManager::LIGHT || drxMan->getStatus() == DRXManager::DEEP) {
                        drxMan->setStatus(DRXManager::ACTIVE);
                    }
                }
            }
        }
    }
}

void
UeMacEntity::ReceiveUplinkAllocationMap(std::vector<int> prbs, int mcs, int id) {
    if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
        std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " received Allocation Map" << std::endl;
    } else {
        std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " received Allocation Map" << std::endl;
    }

    Simulator::Init()->Schedule(0.003, &UeMacEntity::ScheduleUplinkTransmission, this, prbs, mcs, id);
}

void
UeMacEntity::ScheduleUplinkTransmission(std::vector<int> prbs, int mcs, int id) {

    int TTI = round(Simulator::Init()->Now() * double(1000.0));

    if (m_harqBuffer[TTI % 8] != NULL) {
        PacketBurst *pb = m_harqBuffer[TTI % 8]->Copy();
        pb->m_TTI = TTI;
        pb->m_BSR = -1;

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " resent Data with " << pb->GetSize() << " byte(s) in " << pb->GetUsedPRBs().size() << " PRBs" << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " resent Data with " << pb->GetSize() << " byte(s) in " << pb->GetUsedPRBs().size() << " PRBs" << std::endl;
        }

        //std::cout << pb->GetSize() << " " << Simulator::Init()->Now() * 1000 << " " << TTI << " " << TTI % 8 << std::endl;

        GetDevice()->SendPacketBurst(pb);
    } else {

        int availableBytes = GetAmcModule()->GetTBSizeToUplinkFromMCS(mcs, prbs.size()) / 8;
        std::cerr << Simulator::Init()->Now() << " UE " << GetDevice()->GetIDNetworkNode() << " " << mcs << " " << prbs.size() << " available bytes = " << availableBytes << std::endl;

        if (m_retxBSRTimerHandler != NULL) {
            m_retxBSRTimerHandler->cancel();
        }

        m_triggered_BSR = false;
        m_retxBSRTimer = false;
        m_retxBSRTimerHandler = NULL;

        PacketBurst *pb = new PacketBurst();
        RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();

        pb->SetMcsUsed(mcs);

        /* Set the PRBs used to transmission (Used to check the interference with D2D communication) */
        for (int rb = 0; rb < prbs.size(); rb++) {
            //std::cerr << Simulator::Init()->Now() << " UE ID " << GetDevice()->GetIDNetworkNode() << " Used PRB " << prbs.at(rb) << std::endl;
            pb->SetUsedPRB(prbs.at(rb));
        }
        /**/

        if (rrc->GetRadioBearerContainer()->size() > 0) {
            for (RrcEntity::RadioBearersContainer::iterator it = rrc->GetRadioBearerContainer()->begin(); it != rrc->GetRadioBearerContainer()->end(); it++) {
                RadioBearer *b = (*it);

                if (availableBytes > 0) {
                    RlcEntity *rlc = b->GetRlcEntity();
                    PacketBurst *pb2 = rlc->TransmissionProcedure(availableBytes);
                    if (pb2->GetNPackets() > 0) {
                        std::list<Packet*> packets = pb2->GetPackets();
                        std::list<Packet*>::iterator it;
                        for (it = packets.begin(); it != packets.end(); it++) {
                            Packet *p = (*it);
                            pb->AddPacket(p->Copy());
                        }
                    }
                    availableBytes -= pb2->GetSize();
                    delete pb2;
                }
            }

            if (availableBytes >= 2) {
                //TriggerBufferStatusReport(2);

                //NetworkNode *m_d2dDestination = NULL;

                int bufferSize = 0;
                RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();

                if (rrc->GetRadioBearerContainer()->size() > 0) {

                    for (RrcEntity::RadioBearersContainer::iterator it = rrc->GetRadioBearerContainer()->begin(); it != rrc->GetRadioBearerContainer()->end(); it++) {
                        RadioBearer *b = (*it);

                        if (b->GetDestination()->GetNodeType() == NetworkNode::TYPE_UE) {
                            std::cerr << "D2D Buffer!" << std::endl;
                            //m_d2dDestination = b->GetDestination();
                        }

                        if (b->GetApplication()->GetApplicationType() != Application::APPLICATION_TYPE_INFINITE_BUFFER) {
                            std::cerr << "RRC buffer size " << b->GetQueueSize() << std::endl;
                            bufferSize += b->GetQueueSize();
                        } else {
                            bufferSize += 10000000;
                        }
                    }

                }

                //if (m_d2dDestination) {
                //  std::cout << "BUFFER -- " << GetDevice()->GetIDNetworkNode() << " " << m_d2dDestination->GetIDNetworkNode() << std::endl;
                //msg->SetDestinationUeD2D(m_d2dDestination);
                //}

                //if (how == 1) {
                //  if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                //    std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent BSR with " << bufferSize << " byte(s) to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << " by " << how << std::endl;
                // } else {
                //std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent BSR with " << bufferSize << " byte(s) to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << " by " << how << std::endl;
                //  }
                //}

                pb->m_BSR = bufferSize;

                //if (bufferSize > 0) {
                //StartRetxBSRTimer();
                //} else {
                m_triggered_BSR = false;
                //}
            }

            if (pb->GetSize() > 0 || pb->m_BSR > -1) {
                if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                    std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent Data with " << pb->GetSize() << " byte(s) in " << prbs.size() << " PRBs and BSR with " << pb->m_BSR << " byte(s)" << std::endl;
                } else {
                    std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent Data with " << pb->GetSize() << " byte(s) in " << prbs.size() << " PRBs and BSR with " << pb->m_BSR << " byte(s)" << std::endl;
                }

                //std::cout << pb->GetSize() << " " << TTI << " " << TTI % 8 << std::endl;

                pb->m_TTI = TTI;
                pb->m_sourceDevice = GetDevice()->GetIDNetworkNode();
                m_harqBuffer[TTI % 8] = pb->Copy();

                GetDevice()->SendPacketBurst(pb);
            } else {
                GetDevice()->SendPacketBurst(NULL);
            }

        }
    }
}

void
UeMacEntity::ScheduleUplinkTransmissionSemiSch() {
    ENodeB::UserEquipmentRecord *record = ((ENodeB *) ((UserEquipment *) GetDevice())->GetTargetNode())->GetUserEquipmentRecord(GetDevice()->GetIDNetworkNode());

    int availableBytes = GetAmcModule()->GetTBSizeToUplinkFromMCS(record->m_semiMCS, record->m_semiRBs) / 8;

    m_triggered_BSR = false;
    m_retxBSRTimer = false;
    m_retxBSRTimerHandler = NULL;

    PacketBurst *pb = new PacketBurst();
    RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();

    if (rrc->GetRadioBearerContainer()->size() > 0) {
        for (RrcEntity::RadioBearersContainer::iterator it = rrc->GetRadioBearerContainer()->begin(); it != rrc->GetRadioBearerContainer()->end(); it++) {
            RadioBearer *b = (*it);

            if (availableBytes > 0) {
                RlcEntity *rlc = b->GetRlcEntity();
                PacketBurst *pb2 = rlc->TransmissionProcedure(availableBytes);
                if (pb2->GetNPackets() > 0) {
                    std::list<Packet*> packets = pb2->GetPackets();
                    std::list<Packet*>::iterator it;
                    for (it = packets.begin(); it != packets.end(); it++) {
                        Packet *p = (*it);
                        pb->AddPacket(p->Copy());
                    }
                }
                availableBytes -= pb2->GetSize();
                delete pb2;
            }
        }

        if (pb->GetSize() > 0) {
            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent Data with " << pb->GetSize() << " byte(s) in " << record->m_semiRBs << " PRBs" << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent Data with " << pb->GetSize() << " byte(s) in " << record->m_semiRBs << " PRBs" << std::endl;
            }
            GetDevice()->SendPacketBurst(pb);
        } else {
            GetDevice()->SendPacketBurst(NULL);
        }
    }

    if (record->m_semiSchTime > 0) {
        Simulator::Init()->Schedule(((double) record->m_semiSchPeriod / (double) 1000), &UeMacEntity::ScheduleUplinkTransmissionSemiSch, this);
    }
}

void
UeMacEntity::CheckForDropPackets(void) {
    //if (GetQCINumber() < 5) {
    RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();
    RrcEntity::RadioBearersContainer* bearers = rrc->GetRadioBearerContainer();

    for (std::vector<RadioBearer* >::iterator it = bearers->begin(); it != bearers->end(); it++) {
        //delete packets from queue
        (*it)->GetMacQueue()->CheckForDropPackets((*it)->GetQoSParameters()->GetMaxDelay(), (*it)->GetApplication()->GetApplicationID());

        //delete fragment waiting in AM RLC entity
        if ((*it)->GetRlcEntity()->GetRlcModel() == RlcEntity::AM_RLC_MODE) {

            AmRlcEntity* amRlc = (AmRlcEntity*) (*it)->GetRlcEntity();
            amRlc->CheckForDropPackets((*it)->GetQoSParameters()->GetMaxDelay(), (*it)->GetApplication()->GetApplicationID());
        }
    }

    Simulator::Init()->Schedule(0.005, &UeMacEntity::CheckForDropPackets, this);
    //}
}

/*
 * Usado para começar o contention-base random access procedure
 * Tiago Pedroso da Cruz de Andrade
 */
void
UeMacEntity::StartRAProcedure() {
    if (UPLINK == true) {
        if (m_RAStopped) {
            m_RAStopped = false;
            m_executingRA = true;

            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " started RA_Procedure";
            } else {
                std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " started RA_Procedure";
            }

            if (m_fixedPreamble) {
                std::cerr << " CONTENTION-FREE" << std::endl;
            } else {
                std::cerr << " CONTENTION-BASED" << std::endl;
            }

            m_preambleTransmissionCounter = 0;
            m_backoffIndicator = 0; //InformationManager::Init()->backoffIndicatorGeneral;

            int timeTemp;
            double time;

            switch (InformationManager::Init()->ra_Method) {
                case 1: // conventional LTE RA scheme without overload control (3GPP))
                    m_hasPreambleWaiting = true;
                    break;
                case 2: // Access Class Barring (ACB) (3GPP)
                    m_hasPreambleWaiting = true;
                    break;
                case 3: // Self-adaptive Backoff Overload Control (SAB)
                    m_hasPreambleWaiting = true;
                    m_backoffExponent = InformationManager::Init()->ra_MinBE;
                    /*
                     * m_waitToSendPreamble = Uni[0, (2 ^ m_backoffExponent) - alfa] + beta
                     */
                    if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                        m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 1);
                    } else {
                        m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 2) + 2;
                    }

                    //m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent));
                    break;
                case 4: // QoS-Aware Self-Adaptive RAN Overload Control (QoS-Dracon)
                    m_hasPreambleWaiting = true;
                    m_backoffExponent = GetQCIBackoffExponent();
                    m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent));
                    break;
                case 5: // RACH Resource Separation (RRS) (3GPP)
                    m_hasPreambleWaiting = true;
                    break;
                case 6: // Extended Access Barring (EAB) (3GPP)
                    m_hasPreambleWaiting = true;
                    break;
                case 7: // Specific Backoff (SB) (3GPP)
                    m_hasPreambleWaiting = true;
                    break;
                case 8: // Hybrid
                    m_hasPreambleWaiting = true;
                    break;
                case 9: //
                    m_hasPreambleWaiting = true;
                    break;
                case 10: // Collision-Avoidance Cluster-Based Overload Control Random-access Mechanisms (CC-RA)
                    m_hasPreambleWaiting = true;
                    break;
            }
        }
    } else {
      m_RAStopped = false;
      m_executingRA = true;

      if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
        std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " started RA_Procedure";
      } else {
        std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " started RA_Procedure";
      }

      if (m_fixedPreamble) {
        std::cerr << " CONTENTION-FREE" << std::endl;
      } else {
        std::cerr << " CONTENTION-BASED" << std::endl;
      }

      m_preambleTransmissionCounter = 0;
      m_backoffIndicator = 0;

      int timeTemp;
      double time;

      switch (InformationManager::Init()->ra_Method) {
      case 1: // conventional LTE RA scheme without overload control (3GPP))
        m_hasPreambleWaiting = true;
        break;
      case 2: // Access Class Barring (ACB) (3GPP)
        m_hasPreambleWaiting = true;
        break;
      case 3: // Self-adaptive Backoff (SAB)
        /*m_backoffExponent = InformationManager::Init()->ra_MinBE;
          timeTemp = Random::Init()->Uniform(0, 5 * (1 << m_backoffExponent));
          time = (double) timeTemp / (double) 1000;
          Simulator::Init()->Schedule(time, &UeMacEntity::TimeoutBackoffTimer, this);*/
        m_hasPreambleWaiting = true;
        m_backoffExponent = InformationManager::Init()->ra_MinBE;
        //m_waitToSendPreamble = Random::Init()->Uniform(0, 2 * (1 << m_backoffExponent));
        //m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent));
        /*
         * m_waitToSendPreamble = Uni[0, (2 ^ m_backoffExponent) - alfa] + beta
         */
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
          //m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 1);
          m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 1);
        } else {
          //m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 3) + 2;
          m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 2) + 2;
        }

        break;
      case 4: // QoS-aware Self-adaptive RAN Overload Control (QoS-Dracon)
        m_hasPreambleWaiting = true;
        m_backoffExponent = GetQCIBackoffExponent();
        m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent));
        break;
      case 5: // RACH Resource Separation (RRS) (3GPP)
        m_hasPreambleWaiting = true;
        break;
      case 6: // Extended Access Barring (EAB) (3GPP)
        m_hasPreambleWaiting = true;
        break;
      case 7: // Specific Backoff (SB) (3GPP)
        m_hasPreambleWaiting = true;
        break;
      case 8: // Hybrid
        m_hasPreambleWaiting = true;
        break;
      case 9: //
        m_hasPreambleWaiting = true;
        break;
      case 10: // Collision-Avoidance Cluster-Based Overload Control Random-access Mechanisms (CC-RA)
        m_hasPreambleWaiting = true;
        break;
      }
    }

    NetworkManager::Init()->AddUserEquipmentDoingRandomAccess((UserEquipment*) GetDevice());
}

/*
 * Usado para parar o contention-base random access procedure com sucesso ou falha
 * Tiago Pedroso da Cruz de Andrade
 */
void
UeMacEntity::StopRAProcedure(bool failure) {
    m_hasPreambleWaiting = false;
    m_RAStopped = true;
    m_executingRA = false;

    m_preamble = -1;

    m_temp_enb_users_trying_RA = 1;  // Restarting value for next (just to cover capture effect)
    m_temp_enb_preambles_collided = 0;
    m_temp_enb_preambles_distinct = 1;
    NetworkManager::Init()->RemoveUserEquipmentDoingRandomAccess((UserEquipment*) GetDevice());

    double m_raConsumption = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption() - m_expentAtRAStart;

    if (failure) {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " stopped RA_Procedure incorrectly."
                      << " CONS " << m_raConsumption;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << " UE MTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " stopped RA_Procedure incorrectly."
                      << " CONS " << m_raConsumption;
        }

        if (m_fixedPreamble) {
            std::cerr << " CONTENTION-FREE" << std::endl;
        } else {
            std::cerr << " CONTENTION-BASED" << std::endl;
        }
    } else {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " stopped RA_Procedure correctly."
                      << " CONS " << m_raConsumption;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << " UE MTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " stopped RA_Procedure correctly."
                      << " CONS " << m_raConsumption;
        }

        if (m_fixedPreamble) {
            std::cerr << " CONTENTION-FREE" << std::endl;
        } else {
            std::cerr << " CONTENTION-BASED" << std::endl;
        }

        if ((ONLY_RANDOM_ACCESS == false) && (UPLINK == true)) {
            SendBufferStatusReport(1);
        }
    }

    if ((ONLY_RANDOM_ACCESS == false) && (UPLINK == false)) {
        //((ENodeB *) ((UserEquipment *) GetDevice())->GetTargetNode())->DeleteUserEquipment((UserEquipment *) GetDevice());
        //FrameManager::Init()->GetNetworkManager()->DeleteUserEquipment((UserEquipment *) GetDevice());
        //delete ((UserEquipment *) GetDevice());
    }
}

/*
 * Verifica se esta no processo de RA
 * Tiago Pedroso da Cruz de Andrade
 */
bool
UeMacEntity::GetDoingRAProcedure() {
    //std::cerr << Simulator::Init()->Now() << " UE " << GetDevice()->GetIDNetworkNode() << " QCI " << GetQCINumber() << " wait " << m_waitToSendPreamble << " " << m_hasPreambleWaiting << std::endl;
    if (m_waitToSendPreamble > 0) {
        m_waitToSendPreamble--;

        return false;
    }
    return m_hasPreambleWaiting;
}

/*
 * Envia o preamble sequence utilizando um dos schemes
 * Tiago Pedroso da Cruz de Andrade
 */
void
UeMacEntity::SendRAPreamble() {
    m_hasPreambleWaiting = false;

    resetCounterMatchedRA_RNTI();

    if (InformationManager::Init()->ra_Method == 1) {
        m_msg3TransmissionCounter = 0;

        UserEquipment *thisNode = (UserEquipment*) GetDevice();
        m_preambleTransmissionCounter++;
        if (m_preamble == -1) {
            m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
            //m_preamble = 5; // SAME PREAMBLE FOR EVERY UE
            //m_preamble = Random::Init()->Uniform(0, 3);
            //InformationManager::Init()->numberOfRA_Preambles = 3;
        }
         m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();
        // m_raRNTI = 1 + FrameManager::Init()->GetNbSubframes();

        RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
        msg->SetSourceDevice(thisNode);
        msg->SetDestinationDevice(thisNode->GetTargetNode());

        StartRAResponseWindowTimer();

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                      << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode()
                      //inserir alguma coisa
                      << " posX " << thisNode->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX() 
                      << " posY " << thisNode->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY() 
                      << " distance " << thisNode->GetMobilityModel()->GetAbsolutePosition()->GetDistance(thisNode->GetTargetNode()->GetMobilityModel()->GetAbsolutePosition())
                      << " CONS-STATE " << ((UserEquipment*) GetDevice())->GetConsumptionModel()->state << std::endl;
        } else {
          std::cerr << Simulator::Init()->Now()
                    << " UE MTC " << GetDevice()->GetIDNetworkNode()
                    << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                    << " QCI " << GetQCINumber()
                    << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                    << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode()
                    //inserir a mesma coisa de cima
                    << " posX " << thisNode->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateX() 
                      << " posY " << thisNode->GetMobilityModel()->GetAbsolutePosition()->GetCoordinateY() 
                      << " distance " << thisNode->GetMobilityModel()->GetAbsolutePosition()->GetDistance(thisNode->GetTargetNode()->GetMobilityModel()->GetAbsolutePosition())
                    << " CONS-STATE " << ((UserEquipment*) GetDevice())->GetConsumptionModel()->state << std::endl;
        }

        if(m_preambleTransmissionCounter == 1) {

          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            m_instantAtStart = Simulator::Init()->Now();
            m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
          }
        }

        GetDevice()->GetPhy()->SendIdealControlMessage(msg);
    } else if (InformationManager::Init()->ra_Method == 2) {
      m_msg3TransmissionCounter = 0;
      if ((((UserEquipment*) GetDevice())->GetDeviceAccessClass() >= 0) && (((UserEquipment*) GetDevice())->GetDeviceAccessClass() <= 9)) {
            if (Random::Init()->Uniform(0.0, 1.0) <= InformationManager::Init()->ac_BarringFactor) {

                UserEquipment *thisNode = (UserEquipment*) GetDevice();

                m_preambleTransmissionCounter++;
                m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();
                m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
                // JUST FOR TESTING
                //m_preamble = Random::Init()->Uniform(0, 3);
                //InformationManager::Init()->numberOfRA_Preambles = 3;

                RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
                msg->SetSourceDevice(thisNode);
                msg->SetDestinationDevice(thisNode->GetTargetNode());

                StartRAResponseWindowTimer();

                if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                    std::cerr << Simulator::Init()->Now()
                              << " UE HTC " << GetDevice()->GetIDNetworkNode()
                              << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                              << " QCI " << GetQCINumber()
                              << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                              << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
                } else {
                    std::cerr << Simulator::Init()->Now()
                              << " UE MTC " << GetDevice()->GetIDNetworkNode()
                              << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                              << " QCI " << GetQCINumber()
                              << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                              << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
                }

                if(m_preambleTransmissionCounter == 1) {
                  if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
                    m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
                  }
                }
                GetDevice()->GetPhy()->SendIdealControlMessage(msg);
            } else {
                m_BackoffTimer = true;
                m_RAResponseWindowTimer = false;
                m_ContentionResolutionTimer = false;

                Simulator::Init()->Schedule((0.7 + 0.6 * Random::Init()->Uniform(0.0, 1.0)) * InformationManager::Init()->ac_BarringTime, &UeMacEntity::TimeoutBackoffTimer, this);
            }
        } else {
            UserEquipment *thisNode = (UserEquipment*) GetDevice();

            m_preambleTransmissionCounter++;
            m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
            m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

            RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
            msg->SetSourceDevice(thisNode);
            msg->SetDestinationDevice(thisNode->GetTargetNode());

            StartRAResponseWindowTimer();

            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now()
                          << " UE HTC " << GetDevice()->GetIDNetworkNode()
                          << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                          << " QCI " << GetQCINumber()
                          << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                          << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now()
                          << " UE MTC " << GetDevice()->GetIDNetworkNode()
                          << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                          << " QCI " << GetQCINumber()
                          << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                          << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            }

            if(m_preambleTransmissionCounter == 1) {
              if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
                m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
              }
            }
            GetDevice()->GetPhy()->SendIdealControlMessage(msg);
      }

    } else if (InformationManager::Init()->ra_Method == 3) {
        m_msg3TransmissionCounter = 0;

        UserEquipment *thisNode = (UserEquipment*) GetDevice();

        m_preambleTransmissionCounter++;
        m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
        m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

        RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
        msg->SetSourceDevice(thisNode);
        msg->SetDestinationDevice(thisNode->GetTargetNode());

        StartRAResponseWindowTimer();

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                      << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << " UE MTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                      << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        }

        if(m_preambleTransmissionCounter == 1) {
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
          }
        }
        GetDevice()->GetPhy()->SendIdealControlMessage(msg);
    } else if (InformationManager::Init()->ra_Method == 4) {
        //m_noSensitiveBlocked = InformationManager::Init()->noSensitive_Blocked;
        if (GetQCISensitive() == true) {
            m_msg3TransmissionCounter = 0;

            UserEquipment *thisNode = (UserEquipment*) GetDevice();

            m_preambleTransmissionCounter++;
            m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
            m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

            RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
            msg->SetSourceDevice(thisNode);
            msg->SetDestinationDevice(thisNode->GetTargetNode());

            StartRAResponseWindowTimer();

            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now()
                          << " UE HTC " << GetDevice()->GetIDNetworkNode()
                          << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                          << " QCI " << GetQCINumber()
                          << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                          << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now()
                          << " UE MTC " << GetDevice()->GetIDNetworkNode()
                          << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                          << " QCI " << GetQCINumber()
                          << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                          << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            }

            if(m_preambleTransmissionCounter == 1) {
              if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
                m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
              }
            }
            GetDevice()->GetPhy()->SendIdealControlMessage(msg);
        } else {
            if (m_noSensitiveBlocked == false) {
                m_msg3TransmissionCounter = 0;

                UserEquipment *thisNode = (UserEquipment*) GetDevice();

                m_preambleTransmissionCounter++;
                m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
                m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

                RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
                msg->SetSourceDevice(thisNode);
                msg->SetDestinationDevice(thisNode->GetTargetNode());

                StartRAResponseWindowTimer();

                if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                    std::cerr << Simulator::Init()->Now()
                              << " UE HTC " << GetDevice()->GetIDNetworkNode()
                              << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                              << " QCI " << GetQCINumber()
                              << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                              << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
                } else {
                    std::cerr << Simulator::Init()->Now()
                              << " UE MTC " << GetDevice()->GetIDNetworkNode()
                              << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                              << " QCI " << GetQCINumber()
                              << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI
                              << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
                }

                if(m_preambleTransmissionCounter == 1) {
                  if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
                    m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
                  }
                }
                GetDevice()->GetPhy()->SendIdealControlMessage(msg);
            } else {
                m_BackoffTimer = false;
                m_hasPreambleWaiting = true;
            }
        }

    } else if (InformationManager::Init()->ra_Method == 5) {
        m_msg3TransmissionCounter = 0;

        UserEquipment *thisNode = (UserEquipment*) GetDevice();

        m_preambleTransmissionCounter++;
        if (m_preamble == -1) {
            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles_HTC - 1);
            } else {
                m_preamble = Random::Init()->Uniform(InformationManager::Init()->numberOfRA_Preambles_HTC, InformationManager::Init()->numberOfRA_Preambles - 1);
            }
        }
        m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

        RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
        msg->SetSourceDevice(thisNode);
        msg->SetDestinationDevice(thisNode->GetTargetNode());

        StartRAResponseWindowTimer();

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        }

        if(m_preambleTransmissionCounter == 1) {
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
          }
        }
        GetDevice()->GetPhy()->SendIdealControlMessage(msg);

    } else if (InformationManager::Init()->ra_Method == 6) {
        m_msg3TransmissionCounter = 0;

        if (m_eab_barred == false) {

            m_preambleTransmissionCounter++;
            m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();
            m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
            UserEquipment *thisNode = (UserEquipment*) GetDevice();
            RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
            msg->SetSourceDevice(thisNode);
            msg->SetDestinationDevice(thisNode->GetTargetNode());

            StartRAResponseWindowTimer();

            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            }

            if(m_preambleTransmissionCounter == 1) {
              if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
                m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
              }
            }
            GetDevice()->GetPhy()->SendIdealControlMessage(msg);
        } else {
            m_BackoffTimer = false;
            m_hasPreambleWaiting = true;
        }

    } else if (InformationManager::Init()->ra_Method == 7) {
        m_msg3TransmissionCounter = 0;
        UserEquipment *thisNode = (UserEquipment*) GetDevice();
        m_preambleTransmissionCounter++;
        m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();
        m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
        RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
        msg->SetSourceDevice(thisNode);
        msg->SetDestinationDevice(thisNode->GetTargetNode());

        StartRAResponseWindowTimer();

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        }

        if(m_preambleTransmissionCounter == 1) {
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
          }
        }
        GetDevice()->GetPhy()->SendIdealControlMessage(msg);
    } else if (InformationManager::Init()->ra_Method == 8) {
        m_msg3TransmissionCounter = 0;

        UserEquipment *thisNode = (UserEquipment*) GetDevice();

        m_preambleTransmissionCounter++;
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            //m_preamble = Random::Init()->Uniform(InformationManager::Init()->numberOfRA_Preambles_HTC, InformationManager::Init()->numberOfRA_Preambles - 1);
            m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
        } else {
            //m_preamble = Random::Init()->Uniform(0, (InformationManager::Init()->numberOfRA_Preambles - 1) - InformationManager::Init()->numberOfRA_Preambles_HTC);
            m_preamble = Random::Init()->Uniform(InformationManager::Init()->numberOfRA_Preambles_HTC, InformationManager::Init()->numberOfRA_Preambles - 1);
        }
        m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

        RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
        msg->SetSourceDevice(thisNode);
        msg->SetDestinationDevice(thisNode->GetTargetNode());

        StartRAResponseWindowTimer();

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        }

        if(m_preambleTransmissionCounter == 1) {
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
          }
        }
        GetDevice()->GetPhy()->SendIdealControlMessage(msg);
    } else if (InformationManager::Init()->ra_Method == 9) {
        m_msg3TransmissionCounter = 0;

        UserEquipment *thisNode = (UserEquipment*) GetDevice();

        m_preambleTransmissionCounter++;
        m_preamble = Random::Init()->Uniform(0, InformationManager::Init()->numberOfRA_Preambles - 1);
        m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

        RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
        msg->SetSourceDevice(thisNode);
        msg->SetDestinationDevice(thisNode->GetTargetNode());

        StartRAResponseWindowTimer();

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        } else {

            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        }

        if(m_preambleTransmissionCounter == 1) {
          if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
          }
        }

        GetDevice()->GetPhy()->SendIdealControlMessage(msg);

    } else if (InformationManager::Init()->ra_Method == 10) {
        m_msg3TransmissionCounter = 0;

        UserEquipment *thisNode = (UserEquipment*) GetDevice();

        if (thisNode->GetDeviceType() == 0) {

            m_preambleTransmissionCounter++;
            if (m_preamble == -1) {
                m_preamble = Random::Init()->Uniform(InformationManager::Init()->numberOfPreamblesPerGroup, InformationManager::Init()->numberOfRA_Preambles - 1);
            }
            m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

            RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
            msg->SetSourceDevice(thisNode);
            msg->SetDestinationDevice(thisNode->GetTargetNode());

            StartRAResponseWindowTimer();

            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;


            if(m_preambleTransmissionCounter == 1) {
              if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
                m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
              }
            }
            GetDevice()->GetPhy()->SendIdealControlMessage(msg);
        } else {

            if ((FrameManager::Init()->GetRACHCounter() % InformationManager::Init()->numberOfGroups) == floor(thisNode->GetIDNetworkNode() / (double) InformationManager::Init()->numberOfPreamblesPerGroup)) {

                m_preambleTransmissionCounter++;
                m_preamble = thisNode->GetIDNetworkNode() % InformationManager::Init()->numberOfPreamblesPerGroup;
                m_raRNTI = FrameManager::Init()->GetNbFrames() * 10 + FrameManager::Init()->GetNbSubframes();

                RAPreambleIdealControlMessage *msg = new RAPreambleIdealControlMessage(m_raRNTI, m_preamble, m_preambleTransmissionCounter);
                msg->SetSourceDevice(thisNode);
                msg->SetDestinationDevice(thisNode->GetTargetNode());

                StartRAResponseWindowTimer();

                std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent RA_Preamble " << msg->GetPreamble() << " " << m_raRNTI << " to ENodeB " << msg->GetDestinationDevice()->GetIDNetworkNode() << std::endl;


                if(m_preambleTransmissionCounter == 1) {
                  if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
                    m_expentAtRAStart = ((UserEquipment*) GetDevice())->GetConsumptionModel()->getConsumption();  // This is the first transmission
                  }
                }
                GetDevice()->GetPhy()->SendIdealControlMessage(msg);
            } else {
                m_hasPreambleWaiting = true;
            }

        }
    }
}

/*
 * Usado para receber a mensagem RAR
 * Tiago Pedroso da Cruz de Andrade
 */
void
UeMacEntity::ReceiveRandomAccessResponseIdealControlMessage(RAResponseIdealControlMessage *msg) {
#ifdef ENERGY_CONSUMPTION_DEBUG
    std::cout << "[Counting Time RX] (Ideal Message RAR) at " << GetDevice()->GetIDNetworkNode() << " at " << Simulator::Init()->Now() << std::endl;
#endif

    /*if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
      ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_NO_DATA, DRXManager::PDCCH_RX);
    }
    if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
      ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateDL(DRXManager::PDCCH_RX);
    }*/

    if (msg->getRARNTI() == m_raRNTI) {
        if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
            ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_RX, DRXManager::PDSCH_RX);
        }
        if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
            ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateDL(DRXManager::PDSCH_RX);
        }

        if (msg->getError() == false) {
            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " detected RA_Response from ENodeB " << ((ENodeB*) msg->GetSourceDevice())->GetIDNetworkNode() << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " detected RA_Response from ENodeB " << ((ENodeB*) msg->GetSourceDevice())->GetIDNetworkNode() << std::endl;
            }
            ReadyRandomAccessResponseIdealControlMessage(msg);
        } else {
            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " dropped RA_Response from ENodeB " << ((ENodeB*) msg->GetSourceDevice())->GetIDNetworkNode() << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " dropped RA_Response from ENodeB " << ((ENodeB*) msg->GetSourceDevice())->GetIDNetworkNode() << std::endl;
            }
        }
    }

    delete msg;
}

void
UeMacEntity::ReadyRandomAccessResponseIdealControlMessage(RAResponseIdealControlMessage *msg) {
    m_backoffIndicator = msg->GetBackoffIndicatorGeneral();

    if (InformationManager::Init()->ra_Method == 7) {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            m_backoffIndicator = msg->GetBackoffIndicatorHTC();
        } else {
            m_backoffIndicator = msg->GetBackoffIndicatorMTC();
        }
    }

    RAResponseIdealControlMessage::IdealRAResponse *rar = msg->GetMessage();
    RAResponseIdealControlMessage::IdealRAResponse::iterator iter;

    for (iter = rar->begin(); iter != rar->end(); iter++) {
        if (((*iter).m_preamble == m_preamble) && (m_raRNTI != -1)) {
            m_RAResponseWindowTimerHandler->cancel();

            m_RAResponseWindowTimer = false;
            m_RAResponseWindowTimerHandler = NULL;
            m_raRNTI_Using = m_raRNTI;
            m_raRNTI = -1;

            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now()
                          << " UE HTC " << GetDevice()->GetIDNetworkNode()
                          << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                          << " QCI " << GetQCINumber()
                          << " received RA_Response from ENodeB " << ((ENodeB*) msg->GetSourceDevice())->GetIDNetworkNode()
                          << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now()
                          << " UE MTC " << GetDevice()->GetIDNetworkNode()
                          << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                          << " QCI " << GetQCINumber()
                          << " received RA_Response from ENodeB " << ((ENodeB*) msg->GetSourceDevice())->GetIDNetworkNode()
                          << std::endl;
            }

            if (m_fixedPreamble || (InformationManager::Init()->ra_Method == 10 && ((UserEquipment *) GetDevice())->GetDeviceType() == 1)) {
                /* Set the RA procedure as successfully */
                StopRAProcedure(false);
            } else {
                if (UPLINK) {
                    Simulator::Init()->Schedule(0.003, &UeMacEntity::AssemblySchedulingRequestIdealControlMessage, this);
                } else {
                    Simulator::Init()->Schedule(0.003, &UeMacEntity::AssemblyConnectionRequestIdealControlMessage, this);
                }
            }
        }
    }

    //delete msg;
}

void
UeMacEntity::StartRAResponseWindowTimer() {
    m_RAResponseWindowTimer = true;
    m_RAResponseWindowTimerHandler = Simulator::Init()->Schedule((double) (4 + InformationManager::Init()->ra_ResponseWindowSize + 1) / (double) 1000, &UeMacEntity::TimeoutRAResponseWindowTimer, this);
}

void
UeMacEntity::TimeoutRAResponseWindowTimer() {
    if (m_RAResponseWindowTimer == true) {
        m_RAResponseWindowTimer = false;
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " timeout RA_Response" << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << " UE MTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " timeout RA_Response" << std::endl;
        }

        //m_preambleTransmissionCounter++;
        if (m_preambleTransmissionCounter >= InformationManager::Init()->preambleTransMax) {
            m_raRNTI = -1;
            if (!m_fixedPreamble) {
                m_preamble = -1;
            }
            m_inCollisionAvoidance = false;
            m_RAResponseWindowTimerHandler = NULL;
            StopRAProcedure(true);
        } else {
            m_raRNTI = -1;
            if (!m_fixedPreamble) {
                m_preamble = -1;
            }
            m_inCollisionAvoidance = false;
            m_RAResponseWindowTimerHandler = NULL;
            Simulator::Init()->Schedule(0.000, &UeMacEntity::StartBackoffTimer, this);
        }
    }
}

void
UeMacEntity::StartBackoffTimer() {
    //m_preambleTransmissionCounter++;
    if (m_preambleTransmissionCounter >= InformationManager::Init()->preambleTransMax) {
        StopRAProcedure(true);
    } else {
        int timeTemp = 0;

        //m_executingRA = false;

        m_BackoffTimer = true;

        switch (InformationManager::Init()->ra_Method) {
            case 1: // Traditional LTE RA scheme without overload control
                timeTemp = Random::Init()->Uniform(0, BackoffParameterValue[m_backoffIndicator]);
                break;
            case 2: // Access Class Barring (ACB) (3GPP)
                timeTemp = Random::Init()->Uniform(0, BackoffParameterValue[m_backoffIndicator]);
                break;
            case 3: // Self-adaptive Backoff (SAB)
                m_backoffExponent++;
                if (m_backoffExponent > InformationManager::Init()->ra_MaxBE) {
                    m_backoffExponent = InformationManager::Init()->ra_MaxBE;
                }
                //timeTemp = Random::Init()->Uniform(0, 5 * (1 << m_backoffExponent));
                timeTemp = 3;
                //m_waitToSendPreamble = Random::Init()->Uniform(0, 2 * (1 << m_backoffExponent));

                /*
                 * m_waitToSendPreamble = Uni[0, (2 ^ m_backoffExponent) - alfa] + beta
                 */
                if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                    //m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 1);
                    m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 1);
                } else {
                    //m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 3) + 2;
                    m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent) - 2) + 2;
                }

                //m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent));
                break;
            case 4: // (QoS-Dracon)
                timeTemp = 3;
                m_waitToSendPreamble = Random::Init()->Uniform(0, (1 << m_backoffExponent));
                break;
            case 5: // RACH Resource Separation (RRS) (3GPP)
                timeTemp = Random::Init()->Uniform(0, BackoffParameterValue[m_backoffIndicator]);
                break;
            case 6: // Extented Access Barring (EAB) (3GPP)
                if (InformationManager::Init()->eab_on == false) {
                    timeTemp = Random::Init()->Uniform(0, BackoffParameterValue[m_backoffIndicator]);
                } else {
                    timeTemp = 0;
                }
                break;
            case 7: // Specific Backoff (3GPP)
                timeTemp = Random::Init()->Uniform(0, BackoffParameterValue[m_backoffIndicator]);
                break;
            case 8: // Hybrid
                timeTemp = Random::Init()->Uniform(0, BackoffParameterValue[m_backoffIndicator]);
                break;
            case 9: //
                timeTemp = Random::Init()->Uniform(0, BackoffParameterValue[m_backoffIndicator]);
                break;
            case 10: //
                if (((UserEquipment *) GetDevice())->GetDeviceType() == 0) {
                    timeTemp = Random::Init()->Uniform(0, BackoffParameterValue[m_backoffIndicator]);
                } else {
                    timeTemp = 3;
                }
                break;
        }

        // o atraso minimo para a transmissão de outro preambulo depois da RAR window é 3 subframes (3 ms)
        if (timeTemp < 3) {
            timeTemp = 3;
        }

        double time = (double) timeTemp / (double) 1000;

        std::cerr << Simulator::Init()->Now() << " UE " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " backoff " << time << std::endl;

        Simulator::Init()->Schedule(time, &UeMacEntity::TimeoutBackoffTimer, this);
    }
}

void
UeMacEntity::TimeoutBackoffTimer() {
    //[DRX]
    bool canExe = true;

    /**/
    DRXManager *drxMan = ((UserEquipment*) GetDevice())->GetDRXManager();
    if (drxMan != NULL) {
        if (drxMan->getStatus() == DRXManager::LIGHT) {//UE is in Sleep mode,
            Simulator::Init()->Schedule(drxMan->GetExpectedOffTime(), &UeMacEntity::TimeoutBackoffTimer, this);
            canExe = false;
        }
    }
    /**/

    if (canExe) {
        //m_executingRA = true;
        if (!m_fixedPreamble) {
            m_preamble = -1;
        }

        if (InformationManager::Init()->ra_Method != 3) {
            m_backoffExponent = 0;
        }

        m_BackoffTimer = false;
        m_hasPreambleWaiting = true;
    }
}

void
UeMacEntity::AssemblyConnectionRequestIdealControlMessage() {
    std::cerr << Simulator::Init()->Now()
              << " UE " << GetDevice()->GetIDNetworkNode()
              << " Starting Assembly connection number " << m_msg3TransmissionCounter
              << std::endl;


    if (m_msg3TransmissionCounter < InformationManager::Init()->maxHarq_Msg3Tx && !m_premature_harq_stop) {
        StartContentionResolutionTimer();

        ConnectionRequestIdealControlMessage *connectionRequest = new ConnectionRequestIdealControlMessage(m_raRNTI_Using, m_preamble, m_preambleTransmissionCounter - 1, GetQCISensitive());
        UserEquipment *thisNode = (UserEquipment*) GetDevice();
        connectionRequest->SetSourceDevice(this->GetDevice());
        connectionRequest->SetDestinationDevice(thisNode->GetTargetNode());

        double HoLPacketdelay = 0;
        double maxPacketdelay = 0;

        RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();

        if (rrc->GetRadioBearerContainer()->size() > 0) {
            maxPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetQoSParameters()->GetMaxDelay();

            if (rrc->GetRadioBearerContainer()->at(0)->GetQueueSize() > 0) {
                HoLPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetMacQueue()->Peek().GetTimeStamp();
            }
        }

        connectionRequest->SetMaxDelay(maxPacketdelay);
        connectionRequest->SetHoLTime(HoLPacketdelay);

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << connectionRequest->GetSourceDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " sent Connection_Request to ENodeB " << connectionRequest->GetDestinationDevice()->GetIDNetworkNode()
                      << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << " UE MTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " sent Connection_Request to ENodeB " << connectionRequest->GetDestinationDevice()->GetIDNetworkNode()
                      << std::endl;
        }

        if (m_msg3TransmissionCounter > 0) { // Just First trasmission is granted

            if (m_msg3TransmissionCounter == 1) {

                int matched_RAR = getCounterMatchedRA_RNTI();
                int valid_ttis_post_preamble = (int) (Simulator::Init()->Now() - m_instantPreambleSent - 0.003)*1000;

                // Projection for multiple ttis
                if (valid_ttis_post_preamble > 0 && (matched_RAR / valid_ttis_post_preamble) > 2) {
                    matched_RAR = InformationManager::Init()->max_rar_decodable;
                }

                double expected_collisions = calculateExpectedCollisions(matched_RAR);
                setExpectedPreambleCollisions(expected_collisions);
                std::cerr << Simulator::Init()->Now()
                          << " [DEBUG-RA] UE " << GetDevice()->GetIDNetworkNode()
                          << " Expects " << getExpectedPreambleCollisions()
                          << " collisions. Counted " << getCounterMatchedRA_RNTI()
                          << " RARs. Using " << matched_RAR
                          << " after " << valid_ttis_post_preamble
                          << " TTIs. Sent in " << m_instantPreambleSent
                          << std::endl;
            }

            double p = getExpectedPreambleCollisions();
            if (p < 1){
                p = 1;
            }
            p = 1.0/p; // TODO: Set probabiliy of not sending

            std::cerr << Simulator::Init()->Now()
                          << " [DEBUG-RA] UE " << GetDevice()->GetIDNetworkNode()
                          << " will decide about transmission with p = " << p
                          << std::endl;

            if (Random::Init()->Uniform(0.0, 1.0) >= p) {
                // Not Transmitting
                connectionRequest->setAsFakeTransmission();
                std::cerr << Simulator::Init()->Now()
                          << " [DEBUG-RA] UE " << GetDevice()->GetIDNetworkNode()
                          << " Is abdicating from its tx number " << m_msg3TransmissionCounter
                          << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now()
                          << " [DEBUG-RA] UE " << GetDevice()->GetIDNetworkNode()
                          << " will perform its tx number " << m_msg3TransmissionCounter
                          << " normally"
                          << std::endl;
            }
        }

        m_msg3TransmissionCounter++;
        GetDevice()->GetPhy()->SendIdealControlMessage(connectionRequest);
    } else {
        m_premature_harq_stop = false;

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << "[DEBUG CONS-0] UE HTC " << ((UserEquipment*) GetDevice())->GetIDNetworkNode()
                      << " Starting Backoff with msg3Txct = "  << m_msg3TransmissionCounter
                      << " while maxHarqMsg3Tx = " << InformationManager::Init()->maxHarq_Msg3Tx << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << "[DEBUG CONS-0] UE MTC " << ((UserEquipment*) GetDevice())->GetIDNetworkNode()
                      << " Starting Backoff with msg3Txct = "  << m_msg3TransmissionCounter
                      << " while maxHarqMsg3Tx = " << InformationManager::Init()->maxHarq_Msg3Tx << std::endl;
        }

        m_ContentionResolutionTimer = false;
        m_RAResponseWindowTimer = false;
        Simulator::Init()->Schedule(0.000, &UeMacEntity::StartBackoffTimer, this);
    }
}


void
UeMacEntity::AssemblySchedulingRequestIdealControlMessage() {
    std::cerr << Simulator::Init()->Now()
              << " UE " << GetDevice()->GetIDNetworkNode()
              << " Starting Assembly connection " << m_msg3TransmissionCounter
              << std::endl;


    if (m_msg3TransmissionCounter < InformationManager::Init()->maxHarq_Msg3Tx) {
        // Check

        if (m_msg3TransmissionCounter != 0) { // Just First trasmission is granted
            if( 1/0 ) {} //ERROR

            if (Random::Init()->Uniform(0.0, 1.0) <= 0.5) {
                // Not Transmitting

#ifdef HARQ_HOLDTX_DEBUG
                std::cerr << Simulator::Init()->Now()
                          << " UE " << GetDevice()->GetIDNetworkNode()
                          << " Is abdicating from its tx number " << m_msg3TransmissionCounter
                          << std::endl;
#endif

                return;
            }
        }


        m_msg3TransmissionCounter++;

        StartContentionResolutionTimer();

        SchedulingRequestIdealControlMessage *schedulingRequest = new SchedulingRequestIdealControlMessage(m_raRNTI_Using, m_preamble, m_preambleTransmissionCounter - 1, GetQCISensitive());
        UserEquipment *thisNode = (UserEquipment*) GetDevice();
        schedulingRequest->SetSourceDevice(this->GetDevice());
        schedulingRequest->SetDestinationDevice(thisNode->GetTargetNode());

        double HoLPacketdelay = 0;
        double maxPacketdelay = 0;

        RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();

        if (rrc->GetRadioBearerContainer()->size() > 0) {
            maxPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetQoSParameters()->GetMaxDelay();

            if (rrc->GetRadioBearerContainer()->at(0)->GetQueueSize() > 0) {
                HoLPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetMacQueue()->Peek().GetTimeStamp();
            }
        }

        schedulingRequest->SetMaxDelay(maxPacketdelay);
        schedulingRequest->SetHoLTime(HoLPacketdelay);

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << schedulingRequest->GetSourceDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent Scheduling_Request to ENodeB " << schedulingRequest->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent Scheduling_Request to ENodeB " << schedulingRequest->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
        }

        GetDevice()->GetPhy()->SendIdealControlMessage(schedulingRequest);
    } else {

        m_ContentionResolutionTimer = false;
        m_RAResponseWindowTimer = false;
        //Simulator::Init()->Schedule(0.000, &UeMacEntity::SendRAPreamble, this);
        //SendRAPreamble();
        //StartBackoffTimer();
        Simulator::Init()->Schedule(0.000, &UeMacEntity::StartBackoffTimer, this);
        //SendRAPreamble();

        //m_BackoffTimer = false;
        //m_hasPreambleWaiting = true;
    }
}

void
UeMacEntity::ReceiveContentionResolutionIdealControlMessage(ContentionResolutionIdealControlMessage * contentionResolution) {
    //if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
    //  ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_NO_DATA, DRXManager::PDCCH_RX);
    //}

    if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
        ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateDL(DRXManager::PDCCH_RX);
    }

    if (contentionResolution->getError() == true) {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " dropped Contention_Resolution from ENodeB "
                      << ((ENodeB*) contentionResolution->GetSourceDevice())->GetIDNetworkNode()
                      << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << " UE MTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " dropped Contention_Resolution from ENodeB "
                      << ((ENodeB*) contentionResolution->GetSourceDevice())->GetIDNetworkNode()
                      << std::endl;
        }
        delete contentionResolution;
    } else {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " received Contention_Resolution from ENodeB "
                      << ((ENodeB*) contentionResolution->GetSourceDevice())->GetIDNetworkNode()
                      << std::endl;
        } else {

            std::cerr << Simulator::Init()->Now() << " UE MTC "
                      << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " received Contention_Resolution from ENodeB "
                      << ((ENodeB*) contentionResolution->GetSourceDevice())->GetIDNetworkNode()
                      << std::endl;
        }

        m_ContentionResolutionTimerHandler->cancel();

        m_ContentionResolutionTimer = false;
        m_ContentionResolutionTimerHandler = NULL;

        Simulator::Init()->Schedule(0.003, &UeMacEntity::ReadyContentionResolutionIdealControlMessage, this, contentionResolution);
    }
}

void
UeMacEntity::ReadyContentionResolutionIdealControlMessage(ContentionResolutionIdealControlMessage * contentionResolution) {
    m_raRNTI = -1;

    if (!m_fixedPreamble) {
        m_preamble = -1;
    }

    /* Set the RA procedure as successfully */
    StopRAProcedure(false);

    delete contentionResolution;
}

void
UeMacEntity::StartContentionResolutionTimer() {
    m_ContentionResolutionTimer = true;
    m_ContentionResolutionTimerHandler = Simulator::Init()->Schedule((double) (InformationManager::Init()->mac_ContentionResolutionTimer + 1) / (double) 1000, &UeMacEntity::TimeoutContentionResolutionTimer, this, m_msg3TransmissionCounter);
}

void
UeMacEntity::TimeoutContentionResolutionTimer(int counter) {
    //m_preambleTransmissionCounter++;
    if (m_preambleTransmissionCounter >= InformationManager::Init()->preambleTransMax) {
        StopRAProcedure(true);
    } else {
        if ((m_ContentionResolutionTimer == true) && (counter == m_msg3TransmissionCounter)) {
            m_raRNTI = -1;
            m_preamble = -1;
            m_ContentionResolutionTimer = false;
            m_inCollisionAvoidance = false;
            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now()
                          << " UE HTC " << GetDevice()->GetIDNetworkNode()
                          << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                          << " QCI " << GetQCINumber()
                          << " timeout Contention_Resolution" << std::endl;
            } else {
                std::cerr << Simulator::Init()->Now()
                          << " UE MTC " << GetDevice()->GetIDNetworkNode()
                          << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                          << " QCI " << GetQCINumber()
                          << " timeout Contention_Resolution" << std::endl;
            }

            StartBackoffTimer();
        }
    }
}


void
UeMacEntity::ReceiveHarqIdealControlMessage(HarqIdealControlMessage *harq) {
    if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
        ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_NO_DATA, DRXManager::PDCCH_RX);
    }
    if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
        ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateDL(DRXManager::PDCCH_RX);
    }

    if (harq->getHARQType() == 1) {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " received ACK from ENodeB " << harq->GetSourceDevice()->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << " UE MTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " received ACK from ENodeB " << harq->GetSourceDevice()->GetIDNetworkNode() << std::endl;
        }

        if (harq->GetHarqOfRandomAccessMsg3() == false) {
            delete m_harqBuffer[harq->GetHarqProcessId()];
            m_harqBuffer[harq->GetHarqProcessId()] = NULL;
        }

    } else {

        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now()
                      << " UE HTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " received NACK from ENodeB " << harq->GetSourceDevice()->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now()
                      << " UE MTC " << GetDevice()->GetIDNetworkNode()
                      << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass()
                      << " QCI " << GetQCINumber()
                      << " received NACK from ENodeB " << harq->GetSourceDevice()->GetIDNetworkNode() << std::endl;
        }

        if (harq->GetHarqOfRandomAccessMsg3() == true) {
            m_ContentionResolutionTimerHandler->cancel();
            m_ContentionResolutionTimer = false;
            m_ContentionResolutionTimerHandler = NULL;
            ReadyHarqIdealControlMessage(harq);
        }
    }

    delete harq;
}

void
UeMacEntity::ReadyHarqIdealControlMessage(HarqIdealControlMessage *harq) {
    if (harq->getHARQType() == 2) {
        if (!ONLY_RANDOM_ACCESS && UPLINK) {
            Simulator::Init()->Schedule(0.003, &UeMacEntity::AssemblySchedulingRequestIdealControlMessage, this);
        } else {

            // TODO: Decrement expected UES
            if (InformationManager::Init()->harq_method == 1){
                if (harq->isFakeAck() == true){
                    double exp_col = getExpectedPreambleCollisions();
                    setExpectedPreambleCollisions(exp_col - 1);
                    std::cerr << Simulator::Init()->Now()
                              << " [DEBUG-RA] UE " << GetDevice()->GetIDNetworkNode()
                              << " Received FAKEACK. "
                              << "Reducing Expected collisions from " << exp_col
                              << " to " << exp_col - 1 << std::endl;
                }

                if (harq->isHARQCancelled()){
                    std::cerr << Simulator::Init()->Now()
                              << " [DEBUG-RA-HARQ] UE " << GetDevice()->GetIDNetworkNode()
                              << " HARQ process was cancelled by eNB. "
                              << std::endl;
                    m_premature_harq_stop = true;
                }
            }
            Simulator::Init()->Schedule(0.003, &UeMacEntity::AssemblyConnectionRequestIdealControlMessage, this);
        }
    }
}

void
UeMacEntity::StartRetxBSRTimer() {
    m_retxBSRTimer = true;
    m_retxBSRTimerHandler = Simulator::Init()->Schedule(m_retxBSR_Time, &UeMacEntity::TimeoutRetxBSRTimer, this);
}

void
UeMacEntity::TimeoutRetxBSRTimer() {
    if (m_retxBSRTimer == true) {
        m_retxBSRTimer = false;
        m_retxBSRTimerHandler = NULL;
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " timeout BSR" << std::endl;
        } else {

            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " timeout BSR" << std::endl;
        }

        //Simulator::Init()->Schedule(0.000, &UeMacEntity::StartRAProcedure, this);
        TriggerBufferStatusReport(1);
    }
}

void
UeMacEntity::TriggerBufferStatusReport(int BSRType) {
    //if ((m_triggered_BSR == false) && (((UserEquipment *) GetDevice())->GetSemiSchEnable() == false)) {
    if (m_triggered_BSR == false) {
        m_triggered_BSR = true;
        switch (BSRType) {
            case 1:
                std::cerr << "Trigger 1 - Regular" << std::endl;
                SendSchedulingRequest();
                break;
            case 2:
                std::cerr << "Trigger 2 - Padding" << std::endl;
                SendBufferStatusReport(2);
                break;
            case 3:
                std::cerr << "Trigger 3 - Periodic" << std::endl;
                //SendSchedulingRequest();
                SendBufferStatusReport(1);
                break;
        }
    }
}

int
UeMacEntity::GetRealNbOfRBsForULScheduling() {
}

int
UeMacEntity::getCurrentPDCCHCCEs() {
}

void
UeMacEntity::updateCurrentPDCCHCCEs(int cces) {
}

void
UeMacEntity::CheckForPagingMessage() {
    //bool r = ((EnbMacEntity*) ((UserEquipment*) GetDevice())->GetTargetNode()->GetProtocolStack()->GetMacEntity())->GetInformationChanged();
    //if (m_information_changed == false) {
    //  m_information_changed = r;
    // }

    std::cerr << Simulator::Init()->Now() << " UE " << GetDevice()->GetIDNetworkNode() << " checking for Paging Message" << std::endl;
    Simulator::Init()->Schedule((double) (InformationManager::Init()->length_pagingCycle * 10) / (double) (1000), &UeMacEntity::CheckForPagingMessage, this);
}

void
UeMacEntity::StartCheckForPagingMessage() {

    std::cerr << Simulator::Init()->Now() << " UE " << GetDevice()->GetIDNetworkNode() << " PF = " << ((UserEquipment*) GetDevice())->GetPagingFrame() << " PO = " << ((UserEquipment*) GetDevice())->GetPagingOccasion() << std::endl;
    Simulator::Init()->Schedule((double) ((((UserEquipment*) GetDevice())->GetPagingFrame() * 10) + (((UserEquipment*) GetDevice())->GetPagingOccasion() - 1)) / (double) (1000), &UeMacEntity::CheckForPagingMessage, this);
}

void
UeMacEntity::CheckForSystemInformation() {

    //std::cerr << Simulator::Init()->Now() << " UE " << GetDevice()->GetIDNetworkNode() << " checking for System Information" << std::endl;

    if (InformationManager::Init()->ra_Method == 6) {
        //if (m_information_changed) {
        m_eab_barred = InformationManager::Init()->eab_BarringBitmap[((UserEquipment*) GetDevice())->GetDeviceAccessClass()];
        //}
    }

    if (InformationManager::Init()->ra_Method == 4) {
        m_noSensitiveBlocked = InformationManager::Init()->noSensitive_Blocked;
    }

    //m_information_changed = false;

    Simulator::Init()->Schedule((double) (InformationManager::Init()->period_systemInformation) / (double) (1000), &UeMacEntity::CheckForSystemInformation, this);
}

void
UeMacEntity::TimeoutSchedulingRequestTimer() {
    std::cout << Simulator::Init()->Now() << " UE " << GetDevice()->GetIDNetworkNode() << " Scheduling Request Allocation!" << std::endl;

    int srProhibitTimer = ((UserEquipment *) GetDevice())->GetSRProhibitTimer();
    int srPeriodicity = ((UserEquipment *) GetDevice())->GetSRperiodicity();

    DRXManager *drxMan = ((UserEquipment*) GetDevice())->GetDRXManager();

    if (drxMan != NULL) {
        if (drxMan->getStatus() == DRXManager::ACTIVE) {
            if ((m_hasSchedulingRequestWaiting == true) && (m_isSchedulingRequestProhibitPeriod == false)) {
                m_isSchedulingRequestProhibitPeriod = true;
                m_hasSchedulingRequestWaiting = false;

                SchedulingRequestPUCCHIdealControlMessage *schedulingRequestPUCCHMessage = new SchedulingRequestPUCCHIdealControlMessage();
                schedulingRequestPUCCHMessage->SetSourceDevice(GetDevice());
                schedulingRequestPUCCHMessage->SetDestinationDevice(((UserEquipment*) GetDevice())->GetTargetNode());

                double HoLPacketdelay = 0;
                double maxPacketdelay = 0;

                RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();

                if (rrc->GetRadioBearerContainer()->size() > 0) {
                    maxPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetQoSParameters()->GetMaxDelay();

                    if (rrc->GetRadioBearerContainer()->at(0)->GetQueueSize() > 0) {
                        HoLPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetMacQueue()->Peek().GetTimeStamp();
                    }
                }

                schedulingRequestPUCCHMessage->SetMaxDelay(maxPacketdelay);
                schedulingRequestPUCCHMessage->SetHoLTime(HoLPacketdelay);

                if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                    std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent SR to ENodeB " << schedulingRequestPUCCHMessage->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
                } else {

                    std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent SR to ENodeB " << schedulingRequestPUCCHMessage->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
                }

                GetDevice()->GetPhy()->SendIdealControlMessage(schedulingRequestPUCCHMessage);

                Simulator::Init()->Schedule((double) (srProhibitTimer * srPeriodicity) / (double) (1000), &UeMacEntity::TimeoutSchedulingRequestProhibitTimer, this);
            }
        }
    } else {
        if ((m_hasSchedulingRequestWaiting == true) && (m_isSchedulingRequestProhibitPeriod == false)) {
            m_isSchedulingRequestProhibitPeriod = true;
            m_hasSchedulingRequestWaiting = false;

            SchedulingRequestPUCCHIdealControlMessage *schedulingRequestPUCCHMessage = new SchedulingRequestPUCCHIdealControlMessage();
            schedulingRequestPUCCHMessage->SetSourceDevice(GetDevice());
            schedulingRequestPUCCHMessage->SetDestinationDevice(((UserEquipment*) GetDevice())->GetTargetNode());

            double HoLPacketdelay = 0;
            double maxPacketdelay = 0;

            RrcEntity *rrc = GetDevice()->GetProtocolStack()->GetRrcEntity();

            if (rrc->GetRadioBearerContainer()->size() > 0) {
                maxPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetQoSParameters()->GetMaxDelay();

                if (rrc->GetRadioBearerContainer()->at(0)->GetQueueSize() > 0) {
                    HoLPacketdelay = rrc->GetRadioBearerContainer()->at(0)->GetMacQueue()->Peek().GetTimeStamp();
                }
            }

            schedulingRequestPUCCHMessage->SetMaxDelay(maxPacketdelay);
            schedulingRequestPUCCHMessage->SetHoLTime(HoLPacketdelay);

            if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
                std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent SR to ENodeB " << schedulingRequestPUCCHMessage->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            } else {

                std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " sent SR to ENodeB " << schedulingRequestPUCCHMessage->GetDestinationDevice()->GetIDNetworkNode() << std::endl;
            }

            GetDevice()->GetPhy()->SendIdealControlMessage(schedulingRequestPUCCHMessage);

            Simulator::Init()->Schedule((double) (srProhibitTimer * srPeriodicity) / (double) (1000), &UeMacEntity::TimeoutSchedulingRequestProhibitTimer, this);
        }
    }

    Simulator::Init()->Schedule((double) srPeriodicity / (double) (1000), &UeMacEntity::TimeoutSchedulingRequestTimer, this);
}

void
UeMacEntity::TimeoutSchedulingRequestProhibitTimer() {
    if (m_isSchedulingRequestProhibitPeriod == true) {
        m_isSchedulingRequestProhibitPeriod = false;
        m_hasSchedulingRequestWaiting = true;
    }
}

void
UeMacEntity::ReceiveBsrGrantIdealControlMessage(BsrGrantIdealControlMessage * bsrGrantMessage) {
    //if (((UserEquipment*) GetDevice())->GetConsumptionModel() != NULL) {
    //  ((UserEquipment*) GetDevice())->GetConsumptionModel()->setStateUE(DRXManager::ACTIVE_RX, DRXManager::PDSCH_RX);
    //}

    if (((UserEquipment*) GetDevice())->GetDRXManager() != NULL) {
        ((UserEquipment*) GetDevice())->GetDRXManager()->drxUpdateDL(DRXManager::PDSCH_RX);
    }

    if (bsrGrantMessage->getError() == true) {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " dropped BSR_Grant from ENodeB " << ((ENodeB*) bsrGrantMessage->GetSourceDevice())->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " dropped BSR_Grant from ENodeB " << ((ENodeB*) bsrGrantMessage->GetSourceDevice())->GetIDNetworkNode() << std::endl;
        }
    } else {
        if (((UserEquipment*) GetDevice())->GetDeviceType() == 0) {
            std::cerr << Simulator::Init()->Now() << " UE HTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " received BSR_Grant from ENodeB " << ((ENodeB*) bsrGrantMessage->GetSourceDevice())->GetIDNetworkNode() << std::endl;
        } else {
            std::cerr << Simulator::Init()->Now() << " UE MTC " << GetDevice()->GetIDNetworkNode() << " AC " << ((UserEquipment*) GetDevice())->GetDeviceAccessClass() << " QCI " << GetQCINumber() << " received BSR_Grant from ENodeB " << ((ENodeB*) bsrGrantMessage->GetSourceDevice())->GetIDNetworkNode() << std::endl;
        }

        m_isSchedulingRequestProhibitPeriod = false;
        Simulator::Init()->Schedule(0.003, &UeMacEntity::ReadyBsrGrantIdealControlMessage, this);
    }

    delete bsrGrantMessage;
}

void
UeMacEntity::ReadyBsrGrantIdealControlMessage() {
    if ((ONLY_RANDOM_ACCESS == false) && (UPLINK == true)) {
        SendBufferStatusReport(1);
    }
}

int
UeMacEntity::GetPreambleTransmissionCounter() {
    return m_preambleTransmissionCounter;
}

void
UeMacEntity::SetPowerHeadroom(double phr) {
    m_currentPowerHeadroom = phr;
}

bool
UeMacEntity::executingRA() {
    return m_executingRA;
}

void
UeMacEntity::SetExecutingRA(bool e) {
    m_executingRA = e;
}

void
UeMacEntity::SetPreambleSequence(int preamble) {
    m_preamble = preamble;
    m_fixedPreamble = true;
}

void
UeMacEntity::updateCounterMatchedRA_RNTI(RAResponseIdealControlMessage *msg) {
    RAResponseIdealControlMessage::IdealRAResponse *rar = msg->GetMessage();

    if (msg->getRARNTI() != m_raRNTI) {
        return;
    }

    if ((Simulator::Init()->Now() - m_instantPreambleSent) < 0.003) {
        return;
    }

    m_counterMatchedRA_RNTI += rar->size();

    std::cerr << Simulator::Init()->Now()
              << " [DEBUG-RA] UE " << GetDevice()->GetIDNetworkNode()
              << " got RAR with RARNTI " << msg->getRARNTI()
              << ", " <<   (Simulator::Init()->Now() - m_instantPreambleSent)
              << " delay and " << rar->size()
              << " messages, summing to " << m_counterMatchedRA_RNTI
              << std::endl;

}

void
UeMacEntity::setCounterMatchedRA_RNTI(int x){
    m_counterMatchedRA_RNTI = x;
}

void
UeMacEntity::resetCounterMatchedRA_RNTI(){
    m_counterMatchedRA_RNTI = 0;
    m_instantPreambleSent = Simulator::Init()->Now();
}

int
UeMacEntity::getCounterMatchedRA_RNTI(){
    return m_counterMatchedRA_RNTI;
}

double
UeMacEntity::getExpectedPreambleCollisions(){
    return m_expectedPreambleCollisions;
}

void
UeMacEntity::setExpectedPreambleCollisions(double v){
    m_expectedPreambleCollisions = v;
}

double
UeMacEntity::calculateExpectedCollisions(int matchedRA_RNTI){
    // TODO: Table
    double exp_col;

    if (InformationManager::Init()->harq_method_god_mode) {
        exp_col = m_temp_enb_users_trying_RA / (double) m_temp_enb_preambles_distinct;

        std::cerr << Simulator::Init()->Now()
                  << " [DEBUG-RA-IDEAL] UE " << GetDevice()->GetIDNetworkNode()
                  << " dev_trying " << m_temp_enb_users_trying_RA
                  << " preambles_dist " << m_temp_enb_preambles_distinct
                  << std::endl;

    } else {
        InformationManager* im = InformationManager::Init();
        if (getCounterMatchedRA_RNTI() == im->max_rar_decodable && im->ra_Method == 2) {
            double L = (double) im->numberOfRA_Preambles;
            double ue_estimative = L / im->ac_BarringFactor;

            exp_col = ue_estimative / im->max_rar_decodable;

            std::cerr << Simulator::Init()->Now()
                      << " [DEBUG-RA-REAL] UE " << GetDevice()->GetIDNetworkNode()
                      << " found " << im->max_rar_decodable
                      << " RARNTI identified. Using ACB p = " << im->ac_BarringFactor
                      << " ue_estimative " << ue_estimative
                      << " expected_collisions " << exp_col
                      << std::endl;

        } else {
            double L = (double) im->numberOfRA_Preambles;
            double I = L - getCounterMatchedRA_RNTI();
            double ue_estimative = L*std::log(L/I);

            exp_col = ue_estimative / (double) getCounterMatchedRA_RNTI();

            std::cerr << Simulator::Init()->Now()
                      << " [DEBUG-RA-REAL] UE " << GetDevice()->GetIDNetworkNode()
                      << " Idle_preambles " << I
                      << " ue_estimative " << ue_estimative
                      << " L " << L
                      << " expected_collisions " << exp_col
                      << std::endl;
        }
    }
    return exp_col;
}
