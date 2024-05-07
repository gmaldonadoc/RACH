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

#include "macrocell-urban-area-channel-realization.h"
#include "../../device/UserEquipment.h"
#include "../../device/ENodeB.h"
#include "../../device/HeNodeB.h"
#include "../../utility/RandomVariable.h"
#include "shadowing-trace.h"
#include "../../core/spectrum/bandwidth-manager.h"
#include "../../phy/lte-phy.h"
#include "../../core/eventScheduler/simulator.h"
#include "../../load-parameters.h"

MacroCellUrbanAreaChannelRealization::MacroCellUrbanAreaChannelRealization(NetworkNode *src, NetworkNode *dst) {
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

  /*if (_simple_jakes_model_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_JAKES);
  if (_PED_A_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
  if (_PED_B_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_B);
  if (_VEH_A_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_VEH_A);
  if (_VEH_B_)
    SetChannelType(ChannelRealization::CHANNEL_TYPE_VEH_B);*/

  UpdateModels();
}

MacroCellUrbanAreaChannelRealization::~MacroCellUrbanAreaChannelRealization() {
}

void
MacroCellUrbanAreaChannelRealization::SetPenetrationLoss(double pnl) {
  m_penetrationLoss = pnl;
}

double
MacroCellUrbanAreaChannelRealization::GetPenetrationLoss(void) {
  return m_penetrationLoss;
}

double
MacroCellUrbanAreaChannelRealization::GetPathLoss(void) {
  /*
   * According to  ---  insert standard 3gpp ---
   * the Path Loss Model For Urban-macrocell Environment is
   * L = 128.1 + 37.6 * log10(d) -- d (in kilometers) is the distance between two nodes -- C = 4dB, henb = 32m, hue = 1.5m and fc = 1.9GHz
   */

  // Changed from 20dB to 10dB in 2018-06-05, following
  // https://www.arib.or.jp/english/html/overview/doc/STD-T104v4_20/5_Appendix/Rel13/36/36931-d00.pdf
  double externalWallAttenuation = 10; //[dB]

  NetworkNode *src = GetSourceNode();
  NetworkNode *dst = GetDestinationNode();

  double distance = src->GetMobilityModel()->GetAbsolutePosition()->GetDistance(dst->GetMobilityModel()->GetAbsolutePosition());

  m_pathLoss = 128.1 + (37.6 * log10(distance * 0.001));

  UserEquipment *ue;
  if (GetSourceNode()->GetNodeType() == NetworkNode::TYPE_UE) {
    ue = (UserEquipment*) GetSourceNode();
  } else {
    ue = (UserEquipment*) GetDestinationNode();
  }

  if (ue->IsIndoor()) {
    m_pathLoss = m_pathLoss + externalWallAttenuation;
  }

  return m_pathLoss;
}

void
MacroCellUrbanAreaChannelRealization::SetShadowing(double sh) {
  m_shadowing = sh;
}

double
MacroCellUrbanAreaChannelRealization::GetShadowing(void) {
  return m_shadowing;
}

void
MacroCellUrbanAreaChannelRealization::UpdateModels() {

#ifdef TEST_PROPAGATION_LOSS_MODEL
  std::cout << "\t --> UpdateModels" << std::endl;
#endif

  //update shadowing
  m_shadowing = 0;
  double probability = GetRandomVariable(101) / 100.0;
  for (int i = 0; i < 201; i++) {
    if (probability <= shadowing_probability[i]) {
      //m_shadowing = (shadowing_value[i] / (double) 2);
      m_shadowing = shadowing_value[i];
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
MacroCellUrbanAreaChannelRealization::GetLoss() {
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