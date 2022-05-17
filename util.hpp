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


#define RAD_2_DEG 57.295779579
#define DEG_2_RAD 0.0174532925


struct FieldVector{
  cv::Vec3d pos = cv::Vec3d(0, 0, 0);
  cv::Vec3d dir = cv::Vec3d(0, 0, 0);
};


extern cv::Size canvas_size;

// template <typename T> int sign(T val);
cv::Point posToPixel(cv::Point2d p);
cv::Point posToPixel(cv::Point p);
cv::Point2d posOnCircle(float r, float angle);
cv::Point2d rotateVector2D(cv::Point2d vector, float angle);
cv::Vec3d rotateVector3D_z(cv::Vec3d vector, float angle);

cv::Vec2d clark(cv::Vec3d);
cv::Vec3d clarkInv(cv::Vec2d);

