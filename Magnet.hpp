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



class Magnet{
  float radius;
  float angle;
  float orientation;
  float depth;
  float height;
  float current_density;
  bool polarity; // true = north, south = false
  int res;

  std::vector<Dipole> dipoles;
public:
  Magnet(float radius, float angle, float orientation, float d, float h, float i_density, int res, bool polarity);
  void generateDipolesPolar(float rotor_angle);
  void generateDipolesCartesian();
  cv::Vec3d getFieldVectorAtPos(cv::Vec3d);

  std::vector<Dipole> getDipoles();
  cv::Mat renderMagnet_xy(cv::Mat& canvas);
  cv::Mat renderMagnet_xz(cv::Mat& canvas);
  cv::Mat renderMagnet_yz(cv::Mat& canvas);
};
