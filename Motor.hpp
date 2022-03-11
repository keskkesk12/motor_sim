#pragma once

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

// user headers
#include "Coil.hpp"
#include "Dipole.hpp"
#include "util.hpp"



class Motor {
  std::vector<Coil> coils;
  std::vector<Dipole> dipoles;
  float angle;
  float radius;
  float inertia;
  int poles;
  float dt;

public:
  Motor(int poles, float r, float inertia, float dt);
  void generateCoils(float l, float offset, float r, int N, int res);
  void generateDipoles(float l, float r, int N, int res);
  void generateDl();
  void update(float dt);

  void setVoltages(float U, float V, float W);
  void setCurrents(float U, float V, float W);
  std::vector<float> getCurrents();
  float getAngle();
  std::vector<Coil> getCoils();
  std::vector<Dipole> getDipoles();

  // Render
  cv::Mat renderMotorCoils();
};

