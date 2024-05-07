/*
 *
 */

#include "TimeDriven.h"
#include <cstdlib>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"

TimeDriven::TimeDriven() {
  SetApplicationType(Application::APPLICATION_TYPE_TIME_DRIVEN);
}

TimeDriven::~TimeDriven() {
  Destroy();
}

void
TimeDriven::DoStart(void) {
  Simulator::Init()->Schedule(0.0, &TimeDriven::Send, this);
}

void
TimeDriven::DoStop(void) {
}

void
TimeDriven::ScheduleTransmit(double time) {
  if ((Simulator::Init()->Now() + time) <= GetStopTime()) {
    Simulator::Init()->Schedule(time, &TimeDriven::Send, this);
  }
}

void
TimeDriven::Send(void) {
  Packet *packet = new Packet();
  int uid = Simulator::Init()->GetUID();

  packet->SetID(uid);
  packet->SetTimeStamp(Simulator::Init()->Now());
  packet->SetSize(GetSize());

  PacketTAGs *tags = new PacketTAGs();
  tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_TIME_DRIVEN);
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

  ScheduleTransmit(GetInterval());
}

int
TimeDriven::GetSize(void) const {
  return m_size;
}

void
TimeDriven::SetSize(int size) {
  m_size = size;
}

void
TimeDriven::SetInterval(double interval) {
  m_interval = interval;
}

double
TimeDriven::GetInterval(void) const {
  return m_interval;
}