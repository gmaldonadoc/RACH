/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012
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
 * Author: Carlos A. Astudillo Trujillo <castudillo@ieee.org>
 * Laboratório de Redes de Computadores (LRC)
 * Universidade Estadual de Campinas
 * Campinas, Brazil
 */

#include "zbqos-uplink-packet-scheduler.h"

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

#include "../../../componentManagers/FrameManager.h"
#include "../pdcch-scheduler/pdcch-scheduler.h"

ZBQoSUplinkPacketScheduler::ZBQoSUplinkPacketScheduler() {
  SetMacEntity(0);
  CreateUsersToSchedule();
  SetnUsersFD(5);
}

ZBQoSUplinkPacketScheduler::ZBQoSUplinkPacketScheduler(UplinkPacketScheduler *sch) {
  SetMacEntity(0);
  objScheduler = sch;
  CreateUsersToSchedule();
  SetnUsersFD(5);
}

ZBQoSUplinkPacketScheduler::~ZBQoSUplinkPacketScheduler() {
  Destroy();
}

double
ZBQoSUplinkPacketScheduler::ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel) {
  return 0;
}

double
ZBQoSUplinkPacketScheduler::ComputeSchedulingMetric(UserToSchedule* user, int subchannel) {
  return 0;
}

double
ZBQoSUplinkPacketScheduler::ComputeTDSchedulingMetric(UserToSchedule *user) {
  /*
   * For the QoS 1 scheduler the metric is computed as follows:
   *
   * rateMetric =  averageRate[kbps] / gbr[kbps];
   * delayMetric = headOfTheLine[s] / maxDelay[s];
   * TD Metric =  min {delayMetric, rateMetric}
   */

  //ENodeB *node = (ENodeB*) GetMacEntity()->GetDevice();

  double metric; // Min between delayMetric and rateMetric
  double delayMetric; // Delay Metric (GBR and no-GBR)
  double rateMetric; // Rate Metric (no-GBR only)

  double maxDelay = user->m_maximumDelay;
  double headOfTheLine = user->real_HoL; //Reporting to the eNB by the UE through the BSR Message

  double gbr = user->m_gbr;
  double averageRate = user->m_averageUERate;

  int typeTraffic = 0;

  if (gbr == 0) { // No-GBR
    typeTraffic = 0;
  } else { // GBR
    typeTraffic = 1;
  }

  double a, b, maxNoGBR, maxGBR;
  maxNoGBR = 1; //Default: 1
  maxGBR = 1; //Default: 1
  a = 0.8; //Default: 0.7
  b = 1; //Default:1
  double x;
  if (typeTraffic == 1 && maxDelay != 0) { //GBR
    rateMetric = averageRate / gbr;
    x = headOfTheLine / maxDelay;
    delayMetric = 1 - (headOfTheLine / maxDelay);
  } else if (typeTraffic == 0 && maxDelay != 0) { //No-GBR
    rateMetric = 0.; //no-GBR, it is only scheduled by the delayMetric.
    x = headOfTheLine / maxDelay; // (1 Alta prioridad, 0 Baja Prioridad)
    delayMetric = 1 - (headOfTheLine / maxDelay);
    delayMetric = delayMetric + ZShapeFunction(maxNoGBR, x, 0.7, 0.85) + 1 - ZShapeFunction(maxNoGBR, x, 0.85, 1); //
  } else {//Default
    rateMetric = 0.; //no-GBR, it is only scheduled by the delayMetric.
    x = headOfTheLine / maxDelay; // (1 Alta prioridad, 0 Baja Prioridad)
    delayMetric = 1 - (headOfTheLine / maxDelay);
    delayMetric = delayMetric + ZShapeFunction(maxNoGBR, x, 0.7, 0.85) + 1 - ZShapeFunction(maxNoGBR, x, 0.85, 1); //
  }

  if (rateMetric < delayMetric && rateMetric != 0) {
    metric = rateMetric;
  } else {
    metric = delayMetric;
  }

  /*
  #ifdef SHOW_GRANTS_BYTES
    std::cout << Simulator::Init()->Now() << " G " << averageTXBytes << " R " << averageTXBytes/Simulator::Init()->Now() << " U " << user->m_userToSchedule->GetIDNetworkNode()<< std::endl;
  #endif
   */

#ifdef UL_SCHEDULER_DEBUG
  std::cout << "TD Metric for User " << user->m_userToSchedule->GetIDNetworkNode() << std::endl;
  std::cout << "maxDelay: " << maxDelay << std::endl;
  std::cout << "EstHeadOfTheLine: " << headOfTheLine << std::endl;
  std::cout << "realHeadOfTheLine: " << realHeadOfTheLine << std::endl;
  std::cout << "averageScheduledRate: " << averageRate << " [Kbps]" << std::endl;
  std::cout << "gbr: " << gbr << " [Kbps]" << std::endl;
  std::cout << "Type of Bearer (0=No-GBR, 1=GBR): " << typeTraffic << std::endl;
  std::cout << "rateMetric: " << rateMetric << std::endl;
  std::cout << "delayMetric: " << delayMetric << std::endl;
  std::cout << "TD Metric: " << metric << std::endl;
#endif
  //if (Simulator::Init()->Now()>99.0){
  //  std::cout << "averageScheduledRate: " << averageRate << std::endl;
  // std::cout << "averageTXBytes: " << averageTXBytes << std::endl;
  // std::cout << "averageTXRate: " << averageTXBytes*8/Simulator::Init()->Now()/1000 << "Kbps" << std::endl;
  //      }

  //std::cerr << Simulator::Init()->Now() << " Metric ZBQoS = " << metric << std::endl;

  return metric;
}

//FD scheduling, en principo seria esta misma metrica!

double
ZBQoSUplinkPacketScheduler::ComputeFDSchedulingMetric(UserToSchedule* user, int subchannel) {
  return 0;
}

/*
 * TDPS: Escolhe os usuarios que deveriam ser alocados na proxima rodada.
 */
void
ZBQoSUplinkPacketScheduler::TDScheduling(UsersToSchedule *usersToSchedule) {
  /*-------------------------------Start TD Part------------------------------*/
#ifdef UL_SCHEDULER_DEBUG
  std::cout << " ---- eNB Calculating TD Metric for UL Scheduling----" << std::endl;
#endif

  UsersToSchedule *users = usersToSchedule;

  double metricsTD[users->size()]; //Vetor com as metricas no dominio do tempo!

  for (int u = 0; u < users->size(); u++) {
    metricsTD[u] = ComputeTDSchedulingMetric(users->at(u));
    users->at(u)->m_TDMetric = metricsTD[u];
#ifdef UL_SCHEDULER_DEBUG
    std::cout << "TD Metric for User " << users->at(u)->m_userToSchedule->GetIDNetworkNode() << ": " << metricsTD[u] << " " << std::endl;
#endif
  }

  // ordenamos os usuarios por prioridade
  int tam = users->size();
  int k, m;
  double aux; //Para armazenar temporariamente uma metrica
  UserToSchedule *auxScheduledUser;

  for (k = tam - 1; k > 0; k--) {
    for (m = 0; m < k; m++) { //Faz trocas de vetores. usando algoritmo bubble sort!
      if (metricsTD[m] > metricsTD[m + 1]) { //ordena de maior a menor prioridade (0 - High Priority, 1 - Low Priority) o vector metricsTD -- ERRADO???
        aux = metricsTD[m];
        auxScheduledUser = users->at(m);
        metricsTD[m] = metricsTD[m + 1];
        users->at(m) = users->at(m + 1);
        metricsTD[m + 1] = aux;
        users->at(m + 1) = auxScheduledUser;
      }
    }
  }

#ifdef UL_SCHEDULER_DEBUG
  std::cout << "Sorted Users in TD Scheduling" << std::endl;
  for (int u2 = 0; u2 < users->size(); u2++) {
    std::cout << "TD Metric for User " << users->at(u2)->m_userToSchedule->GetIDNetworkNode() << ": " << metricsTD[u2] << " " << std::endl;
    //printf("%5f  ", (double) metricsTD[u2]);
  }
#endif

  //limitUEsInFD();

#ifdef UL_SCHEDULER_DEBUG
  std::cout << "------------------END TD Part----------------------" << std::endl;
#endif
  /*----------------------------------END TD Part------------------------------------*/
}

void
ZBQoSUplinkPacketScheduler::FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule) {
}

void
ZBQoSUplinkPacketScheduler::RBsAllocation() {
}

int
ZBQoSUplinkPacketScheduler::GetAvailablePRBs() {
  return m_nAvailableRBs;
}