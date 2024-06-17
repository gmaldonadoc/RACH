
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
 * Based on FPS games (i.e., Critical OPS to Android)
 * Author: Tiago Pedroso da Cruz de Andrade
 */

#include "GameUplink.h"

#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"
#include "WEB.h"
#include <math.h>

GameUplink::GameUplink() {
  SetApplicationType(Application::APPLICATION_TYPE_GAME);
}

GameUplink::~GameUplink() {
  Destroy();
}

void
GameUplink::DoStart(void) {
  Simulator::Init()->Schedule(0.0, &GameUplink::Send, this);
}

void
GameUplink::DoStop(void) {
}

void
GameUplink::ScheduleTransmit(double time) {
  if ((Simulator::Init()->Now() + time) <= GetStopTime()) {
    Simulator::Init()->Schedule(time, &GameUplink::Send, this);
  }
}

void
GameUplink::Send(void) {
  
  double u = Random::Init()->Uniform(0., 1.);
  int s = 45. - (5.7 * log(-1 * log(1 - u)));
  SetSize(s);
  
  Packet *packet = new Packet();
  int uid = Simulator::Init()->GetUID();

  packet->SetID(uid);
  packet->SetTimeStamp(Simulator::Init()->Now());
  packet->SetSize(GetSize());

  PacketTAGs *tags = new PacketTAGs();
  tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_GAME);
  tags->SetApplicationSize(packet->GetSize());
  packet->SetPacketTags(tags);


  UDPHeader *udp = new UDPHeader(GetClassifierParameters()->GetSourcePort(), GetClassifierParameters()->GetDestinationPort());
  packet->AddUDPHeader(udp);

  IPHeader *ip = new IPHeader(GetClassifierParameters()->GetSourceID(), GetClassifierParameters()->GetDestinationID());
  packet->AddIPHeader(ip);

  PDCPHeader *pdcp = new PDCPHeader();
  packet->AddPDCPHeader(pdcp);

  Trace(packet);

  GetRadioBearer()->Enqueue(packet);
  
  u = Random::Init()->Uniform(0., 1.);
  int i = 40. - (6. * log(-1 * log(1 - u)));
  SetInterval(i / 1000.0);

  ScheduleTransmit(GetInterval());
}

int
GameUplink::GetSize(void) const {
  return m_size;
}

void
GameUplink::SetSize(int size) {
  m_size = size;
}

void
GameUplink::SetInterval(double interval) {
  m_interval = interval;
}

double
GameUplink::GetInterval(void) const {
  return m_interval;
}