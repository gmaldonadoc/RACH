/*
 *
 */

#ifndef EVENTDRIVEN_H_
#define EVENTDRIVEN_H_

#include "Application.h"

class EventDriven : public Application {
public:
  EventDriven();
  virtual ~EventDriven();

  virtual void DoStart(void);
  virtual void DoStop(void);

  void ScheduleTransmit(double time);
  void Send(void);

  void SetSize(int size);
  int GetSize(void) const;
  
  void SetInterval(double interval);
  double GetInterval(void) const;
  
  void SetMean(double mean);
  double GetMean(void) const;

private:
  double m_mean;
  double m_interval;
  int m_size;
};

#endif