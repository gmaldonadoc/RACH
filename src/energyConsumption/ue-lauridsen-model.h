/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Fernando Pereira <fernandopereira@lrc.ic.unicamp.br>
 *
 * File:   ue-lauridsen-model.h
 * Author: Fernando Pereira
 *
 * Created on June 20, 2017, 13:47
 */

#ifndef UE_LAURIDSEN_MODEL_H
#define	UE_LAURIDSEN_MODEL_H

#include "ue-consumption-model.h"
#include "../utility/eesm-effective-sinr.h"

class LauridsenModel : public ConsumptionModel {
public:

  LauridsenModel();
  virtual ~LauridsenModel();

 // Measure the PUSCH energy consumption based on the tx power (taking
 // account the PDCCH consumption)
  virtual void measureConsumptionInTx(double t);

  double getPUSCHConsumptionInTx(double stx);
  double getPUSCHConsumptionIdle(void);
  //TODO: write other energy consultation methods (just to be complete)

 // Measure the PRACH energy consumption based on the tx power (taking
 // account the PDCCH consumption)
  virtual void measureConsumptionInRx(double t);

 // Measure the PRACH energy consumption based on the tx power (taking
 // account the PDCCH consumption)
  virtual void measureConsumptionInTxRx(double t);

 // Measure the RX power only in PDCCH
  virtual void measureConsumptionInRxOnlyPdcch(double t);

 // Untouched in relation to the Miletus Model
  virtual void measureConsumptionInLightSleep(double t);
  virtual void measureConsumptionInDeepSleep(double t);
  virtual void measureConsumptionInTransaction(double t);

private:

};

#endif
