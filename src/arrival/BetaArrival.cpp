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
 * Author: Tiago
 */

#include "BetaArrival.h"
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>

BetaArrival *BetaArrival::ptr = NULL;

BetaArrival::BetaArrival() {
}

double
BetaArrival::timeFirstSlot(int index) {
  switch (index) {
    case 6:
      return 0.001;
      break;
  }
}

double
BetaArrival::intervalSlots(int index) {
  switch (index) {
    case 6:
      return 0.005;
      break;
  }
}

std::vector<TimeArrival*> *
BetaArrival::buildArrival(int PRACHIndex, int time, int nbDevices, std::string fileName) {
  int i, max = 0, pos, j = 0;
  double iteratorTotal = 0, timeNow;
  std::vector<TimeArrival*> *vectorArrivals = new std::vector<TimeArrival*>;

  double r = 0;

  std::vector<double> probabilityVector = getProbability(fileName);

  int sizeProbabilityVector = probabilityVector.size();

  for (i = 0, timeNow = timeFirstSlot(PRACHIndex); i < sizeProbabilityVector; i++, timeNow += intervalSlots(PRACHIndex)) {
    iteratorTotal += (probabilityVector.at(i) * nbDevices);
    r += probabilityVector.at(i);

    if (int(iteratorTotal) > 0) {
      if (int(iteratorTotal) > max) {
        max = int(iteratorTotal);
        pos = j;
      }
      TimeArrival *newArrival = new TimeArrival(timeNow, int(iteratorTotal));
      vectorArrivals->push_back(newArrival);
      j++;
    }

    iteratorTotal -= int(iteratorTotal);
  }

  vectorArrivals->at(pos)->setNumberArrivals(vectorArrivals->at(pos)->getNumberArrivals() + 1);

  int iiteratorTotal = 0;
  for (i = 0; i < vectorArrivals->size(); i++) {
    iiteratorTotal += vectorArrivals->at(i)->getNumberArrivals();
    
    //std::cout << i << vectorArrivals->at(i)->getTimeArrivals() << " " << vectorArrivals->at(i)->getNumberArrivals() << std::endl;
  }

  //std::cout << "Total = " << vectorArrivals->size() << " " << iteratorTotal << " " << iiteratorTotal << " " << r << std::endl;

  probabilityVector.clear();

  return vectorArrivals;
}

std::vector<double>
BetaArrival::getProbability(std::string fileName) {
  std::string myLine;
  std::vector<double> vectorProbability;

  std::ifstream betaFile;
  betaFile.open(fileName.c_str(), std::ifstream::in);

  while (getline(betaFile, myLine)) {
    vectorProbability.push_back(atof(myLine.c_str()));
  }

  return vectorProbability;
}
