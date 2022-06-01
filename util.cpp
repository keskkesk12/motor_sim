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




const uint16_t dim = 600;
cv::Size canvas_size = {dim, dim};





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


cv::Vec2d clark(cv::Vec3d uvw){
  cv::Mat clark = cv::Mat_<float>(2, 3);
  cv::Mat uvw_mat = (cv::Mat_<float>(3, 1) << uvw[0], uvw[1], uvw[2]);

  clark.at<float>(0, 0) = 1;
  clark.at<float>(0, 1) = -1.0/2.0;
  clark.at<float>(0, 2) = -1.0/2.0;
  clark.at<float>(1, 0) = 0;
  clark.at<float>(1, 1) = sqrt(3)/2.0;
  clark.at<float>(1, 2) = -sqrt(3)/2.0;

  cv::Mat ab = 2.0/3.0 * clark * uvw_mat;

  cv::Vec2d ab_vec(0, 0);
  ab_vec[0] = ab.at<float>(0, 0);
  ab_vec[1] = ab.at<float>(1, 0);

  return ab_vec; 
}


cv::Vec3d clarkInv(cv::Vec2d ab){
  cv::Mat clark_inv = cv::Mat_<float>(3, 2);
  cv::Mat ab_mat = (cv::Mat_<float>(2, 1) << ab[0], ab[1]);

  clark_inv.at<float>(0, 0) = 2.0/3.0;
  clark_inv.at<float>(0, 1) = 0;
  clark_inv.at<float>(1, 0) = -1.0/3.0;
  clark_inv.at<float>(1, 1) = sqrt(3)/3.0;
  clark_inv.at<float>(2, 0) = -1.0/3.0;
  clark_inv.at<float>(2, 1) = -sqrt(3)/3.0;

  cv::Mat uvw = 3.0/2.0 * clark_inv * ab_mat;

  cv::Vec3d uvw_vec(0, 0, 0);
  uvw_vec[0] = uvw.at<float>(0, 0);
  uvw_vec[1] = uvw.at<float>(1, 0);
  uvw_vec[2] = uvw.at<float>(2, 0);

  return uvw_vec;
}

