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

#include "VoIP.h"
#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"
#include "WEB.h"
#include <math.h>

VoIP::VoIP() {
  /*
   * G729 codec generates during the ON period a packet with a fixed size (20 bytes).
   * We must add the RTP header (12 bytes)
   */

  m_size = 32; //application + RTP header
  m_stateON = false;

  maxRndNumbers = 20;
  rndIterator = 0;

  for (int i = 0; i < maxRndNumbers; i++) {
    rndNumbers[i] = Random::Init()->Uniform(0., 1.);
  }

  SetApplicationType(Application::APPLICATION_TYPE_VOIP);
}

VoIP::~VoIP() {
  Destroy();
}

void
VoIP::DoStart(void) {
  Simulator::Init()->Schedule(0.0, &VoIP::Send, this);
}

void
VoIP::DoStop(void) {
}

void
VoIP::ScheduleTransmit(double time) {
  if ((Simulator::Init()->Now() + time) <= GetStopTime()) {
    Simulator::Init()->Schedule(time, &VoIP::Send, this);
  }
}

void
VoIP::Send(void) {
  if (!m_stateON) {
    m_stateON = true;

    double random = rndNumbers[rndIterator % maxRndNumbers];

    m_stateDuration = pow(-1. * 0.824 * log(1 - random), (1. / 1.423));
    m_stateDuration = double(floor(m_stateDuration * 1000.) / 1000.);
    
    m_endState =  Simulator::Init()->Now() + m_stateDuration;

#ifdef APPLICATION_DEBUG
    std::cout << " VoIP_DEBUG - Start ON Period, "
            "\n\t Time = " << Simulator::Init()->Now()
            << "\n\t state ON Duration = " << m_stateDuration
            << "\n\t end ON state = " << m_endState << std::endl;
#endif
  }

  //CREATE A NEW PACKET (ADDING UDP, IP and PDCP HEADERS)
  Packet *packet = new Packet();
  int uid = Simulator::Init()->GetUID();

  packet->SetID(uid);
  packet->SetTimeStamp(Simulator::Init()->Now());
  packet->SetSize(GetSize());

  Trace(packet);

  PacketTAGs *tags = new PacketTAGs();
  tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_VOIP);
  tags->SetApplicationSize(packet->GetSize());
  packet->SetPacketTags(tags);

  UDPHeader *udp = new UDPHeader(GetClassifierParameters()->GetSourcePort(), GetClassifierParameters()->GetDestinationPort());
  packet->AddUDPHeader(udp);

  IPHeader *ip = new IPHeader(GetClassifierParameters()->GetSourceID(), GetClassifierParameters()->GetDestinationID());
  packet->AddIPHeader(ip);

  PDCPHeader *pdcp = new PDCPHeader();
  packet->AddPDCPHeader(pdcp);

  GetRadioBearer()->Enqueue(packet);

  if (Simulator::Init()->Now() <= m_endState) {
    ScheduleTransmit(0.020);
  } else {
    //schedule OFF Period
    m_stateON = false;
    double random = rndNumbers[rndIterator % maxRndNumbers];

    m_stateDuration = pow(-1. * 1.089 * log(1 - random), (1. / 0.899));
    m_stateDuration = double(floor(m_stateDuration * 1000.) / 1000.);

#ifdef APPLICATION_DEBUG
    std::cout << " VoIP_DEBUG - Start OFF Period, "
            "\n\t Time = " << Simulator::Init()->Now()
            << "\n\t state OFF Duration = " << m_stateDuration << std::endl;
#endif

    ScheduleTransmit(m_stateDuration + GetStartTime());
  }

  rndIterator++;
}

int
VoIP::GetSize(void) const {
  return m_size;
}