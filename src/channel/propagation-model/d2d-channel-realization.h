/* 
 * File:   d2d-channel-realization.h
 * Author: tiago
 *
 * Created on May 15, 2017, 9:15 PM
 */

#ifndef D2D_CHANNEL_REALIZATION_H
#define	D2D_CHANNEL_REALIZATION_H

#include "channel-realization.h"
#include "vector"

class NetworkNode;

class D2DChannelRealization : public ChannelRealization {
public:
  D2DChannelRealization(NetworkNode* src, NetworkNode* dst);
  virtual ~D2DChannelRealization();
  void SetPenetrationLoss(double pnl);
  double GetPenetrationLoss(void);
  double GetPathLoss(void);
  void SetShadowing(double sh);
  double GetShadowing(void);
  virtual void UpdateModels(void);

  virtual std::vector<double> GetLoss();

private:
  double m_penetrationLoss;
  double m_pathLoss;
  double m_shadowing;
};

#endif