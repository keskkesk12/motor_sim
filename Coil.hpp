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



class Coil {
  cv::Point2d position;
  float current = 0;
  float orientation;
  float L;
  float r;
  int N;
  float dt;

public:
  Coil(float l, float r, int N, float orientation, cv::Point2d pos, float offset,float res, float dt);

  std::vector<FieldVector> coil_wire_vectors;
  void update(float time);
  void setCurrent(float);

  cv::Vec3d calcFieldStrength(FieldVector, cv::Vec3d);
  cv::Vec3d getFieldVectorAtPos(cv::Vec3d);
  cv::Vec3d forceOnWireDL(FieldVector, float);
  cv::Mat renderCoil_yz(cv::Mat& canvas);
  cv::Mat renderCoil_xz(cv::Mat& canvas);
  cv::Mat renderCoil_xy(cv::Mat& canvas);
};

