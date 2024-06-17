/* 
 * File:   spectral-efficiency.h
 * Author: tiago
 *
 * Created on August 9, 2016, 2:48 PM
 */

#ifndef SPECTRAL_EFFICIENCY_H
#define	SPECTRAL_EFFICIENCY_H

#include <math.h>
#include <vector>

static double
GetSpectralEfficiency(double eSinr) {

  double spectralEfficiency = 0;
  double BER = 0.00005;
  
  spectralEfficiency = log2(1 + (pow(10.0, (eSinr / 10.0)) / ((-log(5.0 * BER)) / 1.5)));
  
  return spectralEfficiency;
}

#endif	/* SPECTRAL_EFFICIENCY_H */