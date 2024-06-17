/*
 *
 */

#include "EventDriven.h"
#include <cstdlib>
#include <cmath>
#include "../../componentManagers/NetworkManager.h"
#include "../radio-bearer.h"
#include "WEB.h"

EventDriven::EventDriven() {
  SetApplicationType(Application::APPLICATION_TYPE_EVENT_DRIVEN);
}

EventDriven::~EventDriven() {
  Destroy();
}

void
EventDriven::DoStart(void) {
  Simulator::Init()->Schedule(0.0, &EventDriven::Send, this);
}

void
EventDriven::DoStop(void) {
}

void
EventDriven::ScheduleTransmit(double time) {
  if ((Simulator::Init()->Now() + time) <= GetStopTime()) {
    Simulator::Init()->Schedule(time, &EventDriven::Send, this);
  }
}

void
EventDriven::Send(void) {
  Packet *packet = new Packet();
  int uId = Simulator::Init()->GetUID();

  packet->SetID(uId);
  packet->SetTimeStamp(Simulator::Init()->Now());
  packet->SetSize(GetSize());

  PacketTAGs *tags = new PacketTAGs();
  tags->SetApplicationType(PacketTAGs::APPLICATION_TYPE_EVENT_DRIVEN);
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
  
  double t_interval = round(((-1.) * (std::log(1. - Random::Init()->Uniform(0., 1.)) / double(1. / GetMean()))) * 1000.) / 1000.;
  SetInterval(t_interval);
  
  ScheduleTransmit(GetInterval());
}

int
EventDriven::GetSize(void) const {
  return m_size;
}

void
EventDriven::SetSize(int size) {
  m_size = size;
}

void
EventDriven::SetInterval(double interval) {
  m_interval = interval;
}

double
EventDriven::GetInterval(void) const {
  return m_interval;
}

void
EventDriven::SetMean(double mean) {
  m_mean = mean;
}

double
EventDriven::GetMean(void) const {
  return m_mean;
}