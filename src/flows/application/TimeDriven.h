/*
 *
 */

#ifndef TIMEDRIVEN_H_
#define TIMEDRIVEN_H_

#include "Application.h"

class TimeDriven : public Application {
public:
  TimeDriven();
  virtual ~TimeDriven();

  virtual void DoStart(void);
  virtual void DoStop(void);

  void ScheduleTransmit(double time);
  void Send(void);

  void SetSize(int size);
  int GetSize(void) const;
  void SetInterval(double interval);
  double GetInterval(void) const;

private:
  double m_interval;
  int m_size;
};

#endif