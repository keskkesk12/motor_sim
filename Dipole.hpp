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



class Dipole {
  float current;
  float orientation;
  float radius;
public:
  Dipole(float offset, float height, float orientation, float current, float radius, int res);
  Dipole(cv::Point2f pos, float orientation, float current, float radius, float res);
  std::vector<FieldVector> dipole_wire_vectors;
  cv::Vec3d getFieldVectorAtPos(cv::Vec3d);
  cv::Vec3d calcFieldStrength(FieldVector, cv::Vec3d);
  cv::Vec3d forceOnWireDL(FieldVector, float);

  // Get methods
  float getCurrent();
};

