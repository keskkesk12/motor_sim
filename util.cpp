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
#include "util.hpp"





cv::Size canvas_size = {600, 600};





cv::Point posToPixel(cv::Point2d p){
  return cv::Point(p.x, p.y);
}

cv::Point posToPixel(cv::Point p){
  return cv::Point(p.x, p.y);
}


cv::Point2d posOnCircle(float r, float angle){
  return cv::Point2d(r*cos(angle), r*sin(angle));
}


cv::Point2d rotateVector2D(cv::Point2d vector, float angle){
  return cv::Point2d(vector.x*cos(angle) - vector.y*sin(angle), vector.x*sin(angle) + vector.y*cos(angle));
}


cv::Vec3d rotateVector3D_z(cv::Vec3d vector, float angle){
  return cv::Vec3d(vector[0]*cos(angle) - vector[1]*sin(angle), vector[0]*sin(angle) + vector[1]*cos(angle), vector[2]);
}
