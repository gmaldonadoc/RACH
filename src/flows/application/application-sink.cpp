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

#include "application-sink.h"
#include "../../device/IPClassifier/ClassifierParameters.h"
#include "../../componentManagers/NetworkManager.h"
#include "../../core/eventScheduler/simulator.h"
#include "../../load-parameters.h"
#include "../../device/UserEquipment.h"
#include "../QoS/QoSParameters.h"

ApplicationSink::ApplicationSink() {
  m_classifierParameters = NULL;
  m_radioBearer = NULL;
  m_sourceApplication = NULL;
}

ApplicationSink::~ApplicationSink() {
  m_classifierParameters = NULL;
  m_radioBearer = NULL;
  m_sourceApplication = NULL;
}

void
ApplicationSink::SetClassifierParameters(ClassifierParameters* cp) {
  m_classifierParameters = cp;
}

ClassifierParameters*
ApplicationSink::GetClassifierParameters(void) {
  return m_classifierParameters;
}

void
ApplicationSink::SetRadioBearerSink(RadioBearerSink* r) {
  m_radioBearer = r;
}

RadioBearerSink*
ApplicationSink::GetRadioBearerSink(void) {
  return m_radioBearer;
}

void
ApplicationSink::SetSourceApplication(Application* a) {
  m_sourceApplication = a;
}

Application*
ApplicationSink::GetSourceApplication(void) {
  return m_sourceApplication;
}

void
ApplicationSink::Receive(Packet *p) {
  if (!_APP_TRACING_) return;

  std::cerr << Simulator::Init()->Now() << " RX ";
  switch (m_sourceApplication->GetApplicationType()) {
    case Application::APPLICATION_TYPE_VOIP:
    {
      std::cerr << "VOIP";
      break;
    }
    case Application::APPLICATION_TYPE_TRACE_BASED:
    {
      std::cerr << "VIDEO";
      break;
    }
    case Application::APPLICATION_TYPE_CBR:
    {
      std::cerr << "CBR";
      break;
    }
    case Application::APPLICATION_TYPE_INFINITE_BUFFER:
    {
      std::cerr << "INF_BUF";
      break;
    }
    case Application::APPLICATION_TYPE_WEB:
    {
      std::cerr << "WEB";
      break;
    }
    case Application::APPLICATION_TYPE_TIME_DRIVEN:
    {
      std::cerr << "TIME_DRIVEN";
      break;
    }
    case Application::APPLICATION_TYPE_EVENT_DRIVEN:
    {
      std::cerr << "EVENT_DRIVEN";
      break;
    }
    case Application::APPLICATION_TYPE_GAME:
    {
      std::cerr << "GAME";
      break;
    }
    default:
    {
      std::cerr << "UNDEFINED";
      break;
    }
  }

  double delay = ((Simulator::Init()->Now() * 10000) - (p->GetTimeStamp() * 10000)) / 10000;
  if (delay < 0.000001) delay = 0.000001;

  //UserEquipment* ue = (UserEquipment*) GetSourceApplication()->GetDestination();

  std::cerr << " ID " << p->GetID()
          << " BEARE " << m_sourceApplication->GetApplicationID()
          << " QCI " << m_sourceApplication->GetQoSParameters()->GetQCI()
          << " SIZE " << p->GetPacketTags()->GetApplicationSize()
          << " SRC " << p->GetSourceID()
          << " DST " << p->GetDestinationID()
          << " DELAY " << delay << std::endl;

  delete p;
}