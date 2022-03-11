// stdlib
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iomanip>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// User headers
#include "Motor.hpp"
#include "Coil.hpp"
#include "Dipole.hpp"
#include "World.hpp"
#include "Controller.hpp"




Motor::Motor(int _poles, float r, float I, float _dt) :
  radius(r), inertia(I), poles(_poles), dt(_dt)
{}


void Motor::generateCoils(float l, float offset, float r, int N, int res){
  for(int i = 0; i < poles; i++){
    float angle = i*2*M_PI/poles;
    cv::Point2d pos(0, 0);
    Coil temp_coil = Coil(l, r, N, angle, pos, offset, res, dt);
    coils.push_back(temp_coil);
  }
}


void Motor::generateDl(){
  Coil coil(0, 0, 0, 0, {0, 0}, 0, 0, 0);
  coil.coil_wire_vectors.clear();
  FieldVector vec;

  // Coil 1
  vec.pos = cv::Vec3d(0, 0, 0);
  vec.dir = cv::Vec3d(0, 0, 100);
  coil.coil_wire_vectors.push_back(vec);

  // Coil 2
  // vec.pos[0] = 50;
  // coil.coil_wire_vectors.push_back(vec);

  coils.push_back(coil);
}


void Motor::generateDipoles(float l, float r, int N, int res){
  
}


cv::Mat Motor::renderMotorCoils(){
  cv::Mat canvas = coils[0].renderCoil_xy();

  for(int i = 1; i < coils.size(); i++){
    cv::add(canvas, coils[i].renderCoil_xy(), canvas);
  }
  return canvas;
}


std::vector<Coil> Motor::getCoils(){
  return coils;
}


std::vector<Dipole> Motor::getDipoles(){
  return dipoles;
}

