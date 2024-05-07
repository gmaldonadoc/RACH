/* 
 * File:   ue-3gpp-model.h
 * Author: tiago
 *
 * Created on January 24, 2017, 12:35 PM
 */

#ifndef UE_3GPP_MODEL_H
#define	UE_3GPP_MODEL_H

#include "ue-consumption-model.h"

class SimpleModel : public ConsumptionModel {
public:

  SimpleModel();
  virtual ~SimpleModel();

  virtual void measureConsumptionInTx(double t); // // measure the PUSCH energy consumption based on the tx power (taking account the PDCCH consumption)
  virtual void measureConsumptionInRx(double t); // measure the PRACH energy consumption based on the tx power (taking account the PDCCH consumption)
  virtual void measureConsumptionInTxRx(double t); // measure the PRACH energy consumption based on the tx power (taking account the PDCCH consumption)
  //virtual void measureConsumptionInPrach(double t); // measure the PRACH energy consumption based on the tx power (taking account the PDCCH consumption)
  virtual void measureConsumptionInRxOnlyPdcch(double t); // measure the RX power only in PDCCH
  virtual void measureConsumptionInLightSleep(double t);
  virtual void measureConsumptionInDeepSleep(double t);
  virtual void measureConsumptionInTransaction(double t);

private:

};

#endif