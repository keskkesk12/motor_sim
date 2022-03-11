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
#include "util.hpp"
#include "Controller.hpp"
#include "Motor.hpp"
#include "Coil.hpp"
#include "Dipole.hpp"



class World {
  float time = 0;
  float dt;
  Motor motor;
  Controller controller;
  std::vector<std::vector<cv::Vec3d>> magnetic_field;
  cv::Vec3b getColor(float);
  cv::Vec3b getColor(cv::Vec3d);
public:
  World(float dt, Motor, Controller);
  void update();
  float getTime();
  void generateField();
  cv::Mat renderMotor();
  cv::Mat renderMagnitudeField();
  cv::Mat renderVectorField();
  std::vector<std::vector<cv::Vec3d>> getMagneticField();
};

