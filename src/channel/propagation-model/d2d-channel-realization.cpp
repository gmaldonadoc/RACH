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
 * Author:
 */

#include "d2d-channel-realization.h"
#include "../../device/UserEquipment.h"
#include "../../device/ENodeB.h"
#include "../../device/HeNodeB.h"
#include "../../utility/RandomVariable.h"
#include "shadowing-trace.h"
#include "../../core/spectrum/bandwidth-manager.h"
#include "../../phy/lte-phy.h"
#include "../../core/eventScheduler/simulator.h"
#include "../../load-parameters.h"

D2DChannelRealization::D2DChannelRealization(NetworkNode *src, NetworkNode *dst) {
  SetSamplingPeriod(0.5);

  m_penetrationLoss = 10;
  m_shadowing = 0;
  m_pathLoss = 0;
  SetFastFading(new FastFading());

  SetSourceNode(src);
  SetDestinationNode(dst);

#ifdef TEST_PROPAGATION_LOSS_MODEL
  std::cout << "Created Channe Realization between " << src->GetIDNetworkNode() << " and " << dst->GetIDNetworkNode() << std::endl;
#endif

  if (_simple_jakes_model_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_JAKES);
  if (_PED_A_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  if (_PED_B_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_B);
  if (_VEH_A_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_VEH_A);
  if (_VEH_B_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_VEH_B);

  UpdateModels();
}

D2DChannelRealization::~D2DChannelRealization() {
}

void
D2DChannelRealization::SetPenetrationLoss(double pnl) {
  m_penetrationLoss = pnl;
}

double
D2DChannelRealization::GetPenetrationLoss() {
  return m_penetrationLoss;
}

double
D2DChannelRealization::GetPathLoss() {
  double alpha;

  NetworkNode *src = GetSourceNode();
  NetworkNode *dst = GetDestinationNode();

  double distance = src->GetMobilityModel()->GetAbsolutePosition()->GetDistance(dst->GetMobilityModel()->GetAbsolutePosition());

  double PL_los = (16.9 * log10(distance)) + 46.8 + (20 * log10(2.0 / 5.0));

  double PL_nlos = (40 * log10(distance * 0.001)) + (30 * log10(2000)) + 49;

  if (distance <= 4) {
    alpha = 1;
  } else if (distance > 4 && distance < 60) {
    alpha = exp(-1 * (distance - 4) / 3);
  } else {
    alpha = 0;
  }

  m_pathLoss = (alpha * PL_los) + ((1 - alpha) * PL_nlos);

  //std::cout << "Path-loss between " << src->GetIDNetworkNode() << " and " << dst->GetIDNetworkNode() << " with distance equal to " << distance << " is " << m_pathLoss << std::endl;

  return m_pathLoss;
}

void
D2DChannelRealization::SetShadowing(double sh) {
  m_shadowing = sh;
}

double
D2DChannelRealization::GetShadowing(void) {
  return m_shadowing;
}

void
D2DChannelRealization::UpdateModels() {

#ifdef TEST_PROPAGATION_LOSS_MODEL
  std::cout << "\t --> UpdateModels" << std::endl;
#endif

  //update shadowing
  m_shadowing = 0;
  double probability = GetRandomVariable(101) / 100.0;
  for (int i = 0; i < 201; i++) {
    if (probability <= shadowing_probability[i]) {
      m_shadowing = (shadowing_value[i] / (double) 2);
      break;
    }
  }

#ifdef TEST_PROPAGATION_LOSS_MODEL
  std::cout << "\t\t new shadowing" << m_shadowing << std::endl;
#endif

  UpdateFastFading();
  SetLastUpdate();
}

std::vector<double>
D2DChannelRealization::GetLoss() {
#ifdef TEST_PROPAGATION_LOSS_MODEL
  std::cout << "\t  --> compute loss between " << GetSourceNode()->GetIDNetworkNode() << " and " << GetDestinationNode()->GetIDNetworkNode() << std::endl;
#endif

  if (NeedForUpdate()) {
    UpdateModels();
  }

  std::vector<double> loss;

  int now_ms = Simulator::Init()->Now() * 1000;
  int lastUpdate_ms = GetLastUpdate() * 1000;
  int index = now_ms - lastUpdate_ms;
  int nbOfSubChannels = 0;
  int temp = 0;

  if ((this->GetSourceNode()->GetNodeType() == NetworkNode::TYPE_UE) && (this->GetDestinationNode()->GetNodeType() == NetworkNode::TYPE_ENODEB)) {
    temp = GetSourceNode()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();
    nbOfSubChannels = GetSourceNode()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();
  } else if ((this->GetSourceNode()->GetNodeType() == NetworkNode::TYPE_ENODEB) && (this->GetDestinationNode()->GetNodeType() == NetworkNode::TYPE_UE)) {
    temp = GetSourceNode()->GetPhy()->GetBandwidthManager()->GetDlSubChannels().size();
    nbOfSubChannels = GetSourceNode()->GetPhy()->GetBandwidthManager()->GetDlSubChannels().size();
  } else if ((this->GetSourceNode()->GetNodeType() == NetworkNode::TYPE_UE) && (this->GetDestinationNode()->GetNodeType() == NetworkNode::TYPE_UE)) {
    temp = GetSourceNode()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();
    nbOfSubChannels = GetSourceNode()->GetPhy()->GetBandwidthManager()->GetUlSubChannels().size();
  }

  //std::cout << "#subchannels " << nbOfSubChannels << std::endl;

  for (int i = 0; i < nbOfSubChannels; i++) {
    double l = 0 - GetFastFading()->at(i).at(index) - GetPathLoss() - GetPenetrationLoss() - GetShadowing();
    loss.push_back(l);

#ifdef TEST_PROPAGATION_LOSS_MODEL
    std::cout << "\t\t mlp = " << GetFastFading()->at(i).at(index)
            << " pl = " << GetPathLoss()
            << " pnl = " << GetPenetrationLoss()
            << " sh = " << GetShadowing()
            << " LOSS = " << l
            << std::endl;
#endif
  }

  return loss;
}