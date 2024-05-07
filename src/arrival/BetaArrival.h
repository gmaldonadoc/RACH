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

#ifndef BETAARRIVAL_H_
#define BETAARRIVAL_H_

#include <iostream>
#include <vector>

class TimeArrival {
public:

  TimeArrival(double time, int number) {
    timeArrivals = time;
    numberArrivals = number;
  }

  ~TimeArrival();

  void setTimeArrivals(double time) {
    timeArrivals = time;
  }

  void setNumberArrivals(int number) {
    numberArrivals = number;
  }

  double getTimeArrivals(void) {
    return timeArrivals;
  }

  int getNumberArrivals(void) {
    return numberArrivals;
  }
private:
  double timeArrivals;
  int numberArrivals;

};

class BetaArrival {
public:
  BetaArrival();
  ~BetaArrival();

  static BetaArrival* Init(void) {
    if (ptr == NULL) {
      ptr = new BetaArrival();
    }
    return ptr;
  }

  double timeFirstSlot(int index);
  double intervalSlots(int index);

  std::vector<TimeArrival*> *buildArrival(int PRACHIndex, int time, int nbDevices, std::string fileName);

private:
  std::vector<double> getProbability(std::string fileName);

  static BetaArrival *ptr;
};

#endif