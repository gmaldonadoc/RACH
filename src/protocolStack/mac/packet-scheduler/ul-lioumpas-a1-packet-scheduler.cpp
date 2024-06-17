#include <limits>

#include "ul-lioumpas-a1-packet-scheduler.h"

#include "../mac-entity.h"
#include "../../packet/Packet.h"
#include "../../packet/packet-burst.h"
#include "../../../device/NetworkNode.h"
#include "../../../flows/radio-bearer.h"
#include "../../../protocolStack/rrc/rrc-entity.h"
#include "../../../flows/application/Application.h"
#include "../../../device/ENodeB.h"
#include "../../../protocolStack/mac/AMCModule.h"
#include "../../../phy/lte-phy.h"
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../core/idealMessages/ideal-control-messages.h"
#include "../../../flows/QoS/QoSParameters.h"
#include "../../../flows/MacQueue.h"
#include "../../../utility/eesm-effective-sinr.h"

#include "../../../device/UserEquipment.h"
#include "../../../componentManagers/FrameManager.h"
#include "../pdcch-scheduler/pdcch-scheduler.h"

LioumpasA1UplinkPacketScheduler::LioumpasA1UplinkPacketScheduler() {
  SetMacEntity(0);
  CreateUsersToSchedule();
}

LioumpasA1UplinkPacketScheduler::LioumpasA1UplinkPacketScheduler(UplinkPacketScheduler *sch) {
  SetMacEntity(0);
  objScheduler = sch;
  CreateUsersToSchedule();
}

LioumpasA1UplinkPacketScheduler::~LioumpasA1UplinkPacketScheduler() {
  Destroy();
}

double
LioumpasA1UplinkPacketScheduler::ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel) {
  return 0;
}

double
LioumpasA1UplinkPacketScheduler::ComputeSchedulingMetric(UserToSchedule *user, int subchannel) {
  return 0;
}

void
LioumpasA1UplinkPacketScheduler::RBsAllocation() {
}

double
LioumpasA1UplinkPacketScheduler::ComputeTDSchedulingMetric(UserToSchedule *user) {
  std::vector<double> sinrs;
  for (std::vector<double>::iterator c = user->m_channelCondition.begin(); c != user->m_channelCondition.end(); c++) {
    sinrs.push_back(*c);
  }

  double effectiveSinr = GetEesmEffectiveSinr(sinrs);
  double metric = (effectiveSinr * 180000) / user->m_averageUERate;

  return metric;
}

double
LioumpasA1UplinkPacketScheduler::ComputeFDSchedulingMetric(UserToSchedule *user, int subchannel) {
  double channelCondition = user->m_channelCondition.at(subchannel);

  double metric = channelCondition;

  return metric;
}

void
LioumpasA1UplinkPacketScheduler::TDScheduling(UsersToSchedule *usersToSchedule) {
}

void
LioumpasA1UplinkPacketScheduler::FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule) {
  int nbOfRBs = GetMacEntity()->GetRealNbOfRBsForULScheduling();

  int availableRBs; // No of RB's not allocated
  int unallocatedUsers; // No of users who remain unallocated
  int selectedUser; // user to be selected for allocation
  int selectedPRB; // PRB to be selected for allocation
  double bestMetric; // best metric to identify user/RB combination
  int left, right, first, last; // index of left and left PRB's to check
  bool Allocated[nbOfRBs];
  bool allocationMade;

  UsersToSchedule *users = usersMTCToSchedule;
  UserToSchedule *scheduledUser;

  double metricsFD[nbOfRBs][users->size()];
  int requiredPRBs[users->size()];

  //Some initialization
  availableRBs = nbOfRBs;
  unallocatedUsers = users->size();
  for (int i = 0; i < nbOfRBs; i++) {
    Allocated[i] = false;
  }

  //create a matrix of flow metricsFD
  for (int i = 0; i < nbOfRBs; i++) {
    for (int j = 0; j < users->size(); j++) {
      metricsFD[i][j] = ComputeFDSchedulingMetric(users->at(j), i);
    }
  }

  //create number of required PRB's per scheduled users
  for (int j = 0; j < users->size(); j++) {
    if (users->at(j)->m_allowToSchedule) {
      scheduledUser = users->at(j);

      std::vector<double> sinrs;

      for (std::vector<double>::iterator c = scheduledUser->m_channelCondition.begin(); c != scheduledUser->m_channelCondition.end(); c++) {
        sinrs.push_back(*c);
      }

      double effectiveSinr = GetEesmEffectiveSinr(scheduledUser->m_channelCondition);

      int mcs = GetMacEntity()->GetAmcModule()->GetMCSFromCQI(GetMacEntity()->GetAmcModule()->GetCQIFromSinr(effectiveSinr));

      scheduledUser->m_selectedMCS = mcs;

      /* Determining the required number of PRBs according to buffer size and MCS */
      int prbsRequired = 1;
      while (((scheduledUser->m_dataToTransmit * 8) > (GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(mcs, prbsRequired))) && (prbsRequired < 110)) {
        prbsRequired++;
      }
      /* -------- */

      /* Determining the maximum number of PRBs according to power */
      double P0_PUSCH = ((ENodeB *) GetMacEntity()->GetDevice())->GetP0NominalPusch();
      double alpha_PL = ((ENodeB *) GetMacEntity()->GetDevice())->GetAlphaPL();
      double PL = ((ENodeB*) ((UserEquipment*) scheduledUser->m_userToSchedule)->GetTargetNode())->GetReferenceSignalPower() - ((UserEquipment*) scheduledUser->m_userToSchedule)->GetReferenceSignalReceivedPower();
      int prbsMaximum = pow(10, ((scheduledUser->m_userToSchedule->GetPhy()->GetTxPower() - P0_PUSCH - (alpha_PL * PL)) / 10.0));
      /* -------- */

      if (prbsRequired < prbsMaximum) {
        requiredPRBs[j] = prbsRequired;
      } else {
        requiredPRBs[j] = prbsMaximum;
      }

      std::cout << "Uplink Scheduling " << scheduledUser->m_userToSchedule->GetPhy()->GetTxPower() << " " << alpha_PL << " " << P0_PUSCH << " " << PL << " TTI " << FrameManager::Init()->GetTTICounter() << " UE " << scheduledUser->m_userToSchedule->GetIDNetworkNode() << " prbsRq " << prbsRequired << " prbsAl " << prbsMaximum << " prbsSch " << requiredPRBs[j] << std::endl;
    } else {
      requiredPRBs[j] = 0;
    }
  }

  int nScheduledUEs = 0;
  int nMaxUEsToSchedule;

  if (objScheduler->GetUplinkTDScheduler() != NULL) {
    nMaxUEsToSchedule = ((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->GetMaxUEsToScheduleInUL();
  } else {
    nMaxUEsToSchedule = users->size();
  }

  while (availableRBs > 0 && unallocatedUsers > 0 && nScheduledUEs <= nMaxUEsToSchedule) {
    selectedPRB = -1;
    for (int i = 0; i < nbOfRBs; i++) {
      if (!Allocated[i]) {
        selectedPRB = i;
        break;
      }
    }

    std::cerr << "Selected PRB " << selectedPRB << std::endl;

    selectedUser = -1;

    //bestMetric = (double) - 2000000000;
    bestMetric = (-1) * std::numeric_limits<double>::max();
    for (int j = 0; j < users->size(); j++) {
      if (users->at(j)->m_listOfAllocatedRBs.size() == 0 && requiredPRBs[j] > 0 && users->at(j)->m_allowToSchedule) {
        if (bestMetric < metricsFD[selectedPRB][j]) {
          selectedUser = j;
          bestMetric = metricsFD[selectedPRB][j];
        }
      }
    }

    if (selectedUser != -1) {
      double t, t1 = 0;

      for (int j = 0; j < users->size(); j++) {
        if (j != selectedUser) {
          t += users->at(j)->m_maximumDelay;
          //t1 = 1 - (users->at(u)->real_HoL / users->at(u)->m_maximumDelay);
        }
      }

      scheduledUser = users->at(selectedUser);

      //if (scheduledUser->m_maximumDelay < (t / (2 * (users->size() - 1)))) {

      //if ((scheduledUser->m_maximumDelay - scheduledUser->real_HoL) <= (t / (2 * (users->size() - 1)))) {

      scheduledUser->m_listOfAllocatedRBs.push_back(selectedPRB);

      std::cerr << "User selected " << scheduledUser->m_userToSchedule->GetIDNetworkNode() << std::endl;

      Allocated[selectedPRB] = true;

      left = selectedPRB;
      //right = selectedPRB + 1;

      availableRBs--;
      unallocatedUsers--;

      first = selectedPRB;
      last = selectedPRB;

      allocationMade = true;

      while (requiredPRBs[selectedUser] > 0 && availableRBs > 0 && last < nbOfRBs - 1) {
        last++;
        if (!Allocated[last]) {
          //bestMetric = (double) - 2000000000;
          bestMetric = (-1) * std::numeric_limits<double>::max();
          for (int j = 0; j < users->size(); j++) {
            if (bestMetric < metricsFD[last][j]) {
              right = j;
              bestMetric = metricsFD[last][j];
            }
          }

          if (selectedUser == right) {
            Allocated[last] = true;
            availableRBs--;
            requiredPRBs[selectedUser]--;
            scheduledUser->m_listOfAllocatedRBs.push_back(last);
          } else {
            last--;
            break;
          }
        } else {
          last--;
          break;
        }
      }

      if (allocationMade) {
        if (((ENodeB *) GetMacEntity()->GetDevice())->GetPdcchScheduler()->PDCCHResourceAllocation(scheduledUser->m_userToSchedule->GetIDNetworkNode()) == true) {
          std::cerr << "UE scheduled" << std::endl;
          nScheduledUEs++;
          scheduledUser->m_transmittedData = GetMacEntity()->GetAmcModule()->GetTBSizeToUplinkFromMCS(scheduledUser->m_selectedMCS, scheduledUser->m_listOfAllocatedRBs.size()) / 8;
        } else {
          std::cerr << "UE not scheduled" << std::endl;
          for (int j = first; j <= last; j++) {
            Allocated[j] = false;
            availableRBs++;
          }
          scheduledUser->m_listOfAllocatedRBs.clear();
          scheduledUser->m_transmittedData = 0;
          requiredPRBs[selectedUser] = 0;
        }
      } else {
        scheduledUser->m_transmittedData = 0;
        scheduledUser->m_listOfAllocatedRBs.clear();
      }
      //} else {

      scheduledUser->m_allowToSchedule = false;
      //}
    }
  }

  m_nAvailableRBs = availableRBs;
}

int
LioumpasA1UplinkPacketScheduler::GetAvailablePRBs() {
  return m_nAvailableRBs;
}
