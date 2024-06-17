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

#ifndef USERSDISTRIBTION_H_
#define USERSDISTRIBTION_H_

#include "../core/cartesianCoodrdinates/CartesianCoordinates.h"
#include "CellPosition.h"
#include "../componentManagers/NetworkManager.h"

#include <vector>
#include <iostream>

static CartesianCoordinates*
GetCartesianCoordinatesFromPolar(double r, double angle) {
  double x = r * cos(angle);
  double y = r * sin(angle);

  CartesianCoordinates *coordinates = new CartesianCoordinates();
  coordinates->SetCoordinates(x, y);
  return coordinates;
}

static vector<CartesianCoordinates*>*
GetUniformUsersDistribution(int idCell, int nbUE) {
  NetworkManager *networkManager = NetworkManager::Init();
  vector<CartesianCoordinates*> *vectorOfCoordinates = new vector<CartesianCoordinates*>;

  Cell *cell = networkManager->GetCellByID(idCell);

  double radius = (cell->GetRadius() * 1000) * 0.8;
  double minDist = cell->GetMinDistance() * 1000;

  CartesianCoordinates *cellCoordinates = cell->GetCellCenterPosition();
  double r;
  double angle;

  for (int i = 0; i < nbUE; i++) {
    CartesianCoordinates *newCoordinates;
    do {
      r = (double) (rand() % (int) radius);
      angle = (double) (rand() % 360) * ((2 * 3.14) / 360);

      newCoordinates = GetCartesianCoordinatesFromPolar(r, angle);
    } while (r < minDist);

    //Compute absoluteCoordinates
    newCoordinates->SetCoordinateX(cellCoordinates->GetCoordinateX() + newCoordinates->GetCoordinateX());
    newCoordinates->SetCoordinateY(cellCoordinates->GetCoordinateY() + newCoordinates->GetCoordinateY());

    vectorOfCoordinates->push_back(newCoordinates);
  }

  return vectorOfCoordinates;
}

static vector<CartesianCoordinates*>*
GetUniformUsersDistributionCarlos(int idCell, int nbUE) {
  NetworkManager * networkManager = NetworkManager::Init();
  vector<CartesianCoordinates*> *vectorOfCoordinates = new vector<CartesianCoordinates*>;

  Cell *cell = networkManager->GetCellByID(idCell);

  double radius = (cell->GetRadius() * 1000) * 0.95;

  CartesianCoordinates *cellCoordinates = cell->GetCellCenterPosition();
  double r;
  double angle;
  int minDistance;
  const int nintervals = 5;
  const int nrolls = nbUE; // number of experiments
  const int nstars = nbUE; // maximum number of stars to distribute
  int p[nintervals] = {};
  int rng;
  int z;
  int allocatedUsers = 0;
  int usersPerInterval;
  int resto;


  usersPerInterval = (int) nbUE / nintervals;
  minDistance = cell->GetMinDistance() * 1000; //[in metros] Minima distancia entre la estacion base y los usuarios
  z = 0;

  int width = (int) (radius - minDistance) / nintervals;

  if (radius < 1050 && radius > 100) { //Radio minimo e maximo

    while (allocatedUsers < nbUE) {

      resto = allocatedUsers % nintervals;

      for (int j = 0; j < 1000; j++) {
        rand();
      }

      rng = rand();

      if (resto == 0) { //VOIP

        r = (int) minDistance + (double) (rng / (RAND_MAX + 1.0)) * (int) (width);
      } else if (resto == 1) {//VOIP
        r = (int) minDistance + (int) (width) + (double) (rng / (RAND_MAX + 1.0)) * (int) (width);
      } else if (resto == 2) {//VIDEO
        r = (int) minDistance + (int) (2 * width) + (double) (rng / (RAND_MAX + 1.0)) * (int) (width);
      } else if (resto == 3) {//VIDEO
        r = (int) minDistance + (int) (3 * width) + (double) (rng / (RAND_MAX + 1.0)) * (int) (width);
      } else if (resto == 4) {//CBR
        r = (int) minDistance + (int) (4 * width) + (double) (rng / (RAND_MAX + 1.0)) * (int) (width);
      }

      //if (p[int (nintervals * ((r-minDistance)/(radius-minDistance)))] < usersPerInterval){
      ++p[int (nintervals * ((r - minDistance) / (radius - minDistance)))];
      allocatedUsers++;
      angle = (double) (rand() % 360) * ((2 * 3.14) / 360); //(2*PI/360)
      //Crea el vector de coordenadas
      CartesianCoordinates *newCoordinates = GetCartesianCoordinatesFromPolar(r, angle);
      //std::cout << "newCoordinates->GetCoordinateX (): " << newCoordinates->GetCoordinateX() << std::endl;
      //std::cout << "newCoordinates->GetCoordinateY (): " << newCoordinates->GetCoordinateY() << std::endl;
      //std::cout << "Rais (X^2 + Y^2): " << newCoordinates->GetCoordinateY () << std::endl;
      //Compute absoluteCoordinates
      newCoordinates->SetCoordinateX(cellCoordinates->GetCoordinateX() + newCoordinates->GetCoordinateX());
      newCoordinates->SetCoordinateY(cellCoordinates->GetCoordinateY() + newCoordinates->GetCoordinateY());

      vectorOfCoordinates->push_back(newCoordinates);
      //std::cout << " Coordenadas do Usuário " << allocatedUsers << " Geradas! " << std::endl;

      //}
      /*else{
        //descarta el numero gerado!
       std::cout << " Numero Descartado! "<< std::endl;

      }*/

    } //END WHILE
  }//End IF

  else {
    //std::default_random_engine generator;
    //std::uniform_real_distribution<double> distribution(0.0,1.0);

    for (int i = 0; i < nbUE; i++) {
      //auxRadius = (double) rand()/RAND_MAX; //-
      //auxAngle = (double) rand()/RAND_MAX; //-

      minDistance = cell->GetMinDistance() * 1000; //[in metros] Minima distancia entre la estacion base y los usuarios

      z = 0;

      for (int j = 0; j < 1000; j++) {
        z = z + rand();
      }

      //rng=int (z/1000);

      rng = rand();

      //r = (double)(rand() %(int)radius); //Forma Original

      //r = (int) minDistance + (double) (rand() %(int) (radius-minDistance));

      r = (int) minDistance + (double) (rng / (RAND_MAX + 1.0))* (int) (radius - minDistance);

      angle = (double) (rand() % 360) * ((2 * 3.14) / 360); //(2*PI/360)

      //  r = (double) auxRadius*(int) radius;
      // angle = (double) (auxAngle*360)*((2*3.14)/360);

      // ++p[int(nintervals*(r/radius))];

      ++p[int (nintervals * (r / radius))];

      //std::cout << "uniform_real_distribution (0.0,radio):" << std::endl;
      // std::cout << std::fixed; std::cout.precision(1);

      CartesianCoordinates *newCoordinates = GetCartesianCoordinatesFromPolar(r, angle);
      //std::cout << "newCoordinates->GetCoordinateX (): " << newCoordinates->GetCoordinateX() << std::endl;
      //std::cout << "newCoordinates->GetCoordinateY (): " << newCoordinates->GetCoordinateY() << std::endl;
      //std::cout << "Rais (X^2 + Y^2): " << newCoordinates->GetCoordinateY () << std::endl;
      //Compute absoluteCoordinates
      newCoordinates->SetCoordinateX(cellCoordinates->GetCoordinateX() + newCoordinates->GetCoordinateX());
      newCoordinates->SetCoordinateY(cellCoordinates->GetCoordinateY() + newCoordinates->GetCoordinateY());

      vectorOfCoordinates->push_back(newCoordinates);

    }
  }

  //for (int i = 0; i < nintervals; ++i) {
  //  std::cout << float(i) / nintervals * radius << " m - " << float(i + 1) / nintervals * radius << " m: ";
  //  std::cout << std::string(p[i] * nstars / nrolls, '*') << std::endl;
  //}

  return vectorOfCoordinates;
}

static vector<CartesianCoordinates*>*
GetUniformUsersDistributionInFemtoCell(int idCell, int nbUE) {
  NetworkManager * networkManager = NetworkManager::Init();
  vector<CartesianCoordinates*> *vectorOfCoordinates = new vector<CartesianCoordinates*>;

  Femtocell *cell = networkManager->GetFemtoCellByID(idCell);

  double side = cell->GetSide();

  CartesianCoordinates *cellCoordinates = cell->GetCellCenterPosition();
  double r;
  double angle;

  for (int i = 0; i < nbUE; i++) {
    r = (double) (rand() % (int) side);
    angle = (double) (rand() % 360) * ((2 * 3.14) / 360);

    CartesianCoordinates *newCoordinates = GetCartesianCoordinatesFromPolar(r, angle);

    //Compute absoluteCoordinates
    newCoordinates->SetCoordinateX(cellCoordinates->GetCoordinateX() + newCoordinates->GetCoordinateX());
    newCoordinates->SetCoordinateY(cellCoordinates->GetCoordinateY() + newCoordinates->GetCoordinateY());

    vectorOfCoordinates->push_back(newCoordinates);
  }

  return vectorOfCoordinates;
}

#endif