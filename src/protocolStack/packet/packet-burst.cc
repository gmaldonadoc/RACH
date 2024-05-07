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

#include <stdint.h>
#include <list>
#include <iostream>

#include "packet-burst.h"

PacketBurst::PacketBurst(void) {
  m_BSR = -1;
  m_packets.clear();
  m_listOfUsedPRBs.clear();
}

PacketBurst::~PacketBurst(void) {
  for (std::list<Packet* >::const_iterator iter = m_packets.begin(); iter != m_packets.end(); ++iter) {
    delete *iter;
  }
  m_packets.clear();
}

PacketBurst*
PacketBurst::Copy(void) {
  PacketBurst *pb = new PacketBurst();

  std::list<Packet*> packets = GetPackets();
  std::list<Packet*>::iterator it;

  for (it = packets.begin(); it != packets.end(); it++) {
    Packet *packet = (*it)->Copy();
    pb->AddPacket(packet);
  }

  pb->SetMcsUsed(GetMcsUsed());
  
  pb->m_sourceDevice = m_sourceDevice;
  pb->m_macDestinationID = m_macDestinationID;
  pb->m_targetDevice = m_targetDevice;
  pb->m_TTI = m_TTI;
  pb->m_BSR = m_BSR;

  for (int i = 0; i < m_listOfUsedPRBs.size(); i++) {
    //std::cout << "PB Copy PRB " << m_listOfUsedPRBs.at(i) << std::endl;
    pb->SetUsedPRB(m_listOfUsedPRBs.at(i));
  }

  return pb;
}

void
PacketBurst::AddPacket(Packet *packet) {
  if (packet) {
    m_packets.push_back(packet);
  }
}

std::list<Packet*>
PacketBurst::GetPackets(void) const {
  return m_packets;
}

uint32_t
PacketBurst::GetNPackets(void) const {
  return m_packets.size();
}

uint32_t
PacketBurst::GetSize(void) const {
  uint32_t size = 0;
  for (std::list<Packet* >::const_iterator iter = m_packets.begin(); iter != m_packets.end(); ++iter) {
    Packet *packet = *iter;
    size += packet->GetSize();
  }
  return size;
}

std::list<Packet* >::const_iterator
PacketBurst::Begin(void) const {
  return m_packets.begin();
}

std::list<Packet* >::const_iterator
PacketBurst::End(void) const {
  return m_packets.end();
}

void
PacketBurst::SetMcsUsed(int mcs) {
  m_mcsUsed = mcs;
}

int
PacketBurst::GetMcsUsed(void) {
  return m_mcsUsed;
}

void
PacketBurst::SetUsedPRB(int prb) {
  m_listOfUsedPRBs.push_back(prb);
}

std::vector<int>
PacketBurst::GetUsedPRBs() {
  return m_listOfUsedPRBs;
}