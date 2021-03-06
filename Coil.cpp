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


Coil::Coil(float _L, float _r, int _N, float _orientation, cv::Point2d _pos, float offset, float res, float _dt) :
  L(_L), r(_r), N(_N), orientation(_orientation), position(_pos), dt(dt)
{
  // Generate coil
  // Res = sections pr revolution
  float turn_length = L/float(N);
  float step_length = turn_length / res;
  float d_theta = 2.0 * M_PI / res;

  // Vectors in homogeneous coordinates
  cv::Point3f start(offset, 0, 0);
  cv::Point3f end(offset, r, 0);
  cv::Mat start_mat = (cv::Mat_<float>(4, 1) << start.x, start.y, start.z, 1);
  cv::Mat end_mat = (cv::Mat_<float>(4, 1) << end.x, end.y, end.z, 1);
  cv::Mat dir_mat = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);

  // Make coil transform
  cv::Mat coil_transform(cv::Size(4, 4), CV_32FC1, cv::Scalar(0));
    // Set position change
    coil_transform.at<float>(0, 3) = step_length;
    // Set rotation
    coil_transform.at<float>(0, 0) = 1;
    coil_transform.at<float>(1, 1) = cos(d_theta);
    coil_transform.at<float>(2, 1) = sin(d_theta);
    coil_transform.at<float>(1, 2) = -sin(d_theta);
    coil_transform.at<float>(2, 2) = cos(d_theta);
    // Scale
    coil_transform.at<float>(3, 3) = 1;

  // Make orientation transform
  cv::Mat orientation_transform(cv::Size(4, 4), CV_32FC1, cv::Scalar(0));
    // Set position change
    // Set rotation
    orientation_transform.at<float>(0, 0) = cos(orientation);
    orientation_transform.at<float>(0, 1) = -sin(orientation);
    orientation_transform.at<float>(1, 0) = sin(orientation);
    orientation_transform.at<float>(1, 1) = cos(orientation);
    orientation_transform.at<float>(2, 2) = 1;
    // Scale
    orientation_transform.at<float>(3, 3) = 1;

  int vec_num = N*res;
  for(int i = 0; i < vec_num; i++){
    FieldVector field_vector;
    // Set start equals to end
    end_mat.copyTo(start_mat);
    // Rotate end
    end_mat = coil_transform * end_mat;

    // Set desired z-orientation
    cv::Mat end_mat_rotated = orientation_transform * end_mat;
    cv::Mat start_mat_rotated = orientation_transform * start_mat;

    // Dir = end - start
    dir_mat = end_mat_rotated - start_mat_rotated;

    // Set field vector values
    field_vector.pos = cv::Vec3d(start_mat_rotated.at<float>(0, 0), start_mat_rotated.at<float>(1, 0), start_mat_rotated.at<float>(2, 0));
    field_vector.dir = cv::Vec3d(dir_mat.at<float>(0, 0), dir_mat.at<float>(1, 0), dir_mat.at<float>(2, 0));

    coil_wire_vectors.push_back(field_vector);
  }
}

// Calculates magnetic field generated by coil at given 3d position
cv::Vec3d Coil::getFieldVectorAtPos(cv::Vec3d pos){
  cv::Vec3d d_field;

  int coil_wire_vectors_size = coil_wire_vectors.size();

  for(int i = 0; i < coil_wire_vectors_size; i++){
    FieldVector temp_field_vector = coil_wire_vectors[i];
    d_field += calcFieldStrength(temp_field_vector, pos);
  }

  return d_field;
}

// Calculates magnetic field at position generated by dL wire element
cv::Vec3d Coil::calcFieldStrength(FieldVector vec, cv::Vec3d p){
  float u0 = 1;

  cv::Vec3d r_vec = p - vec.pos;
  float r = cv::norm(r_vec);
  cv::Vec3d r_hat = r_vec / r;
  cv::Vec3d ds = vec.dir;

  return (current * ds.cross(r_hat)) / (pow(r, 2));
}


cv::Vec3d Coil::forceOnWireDL(FieldVector field_vector, float current){
  // Calculates magnetic field set up at position of wire-dL from coil
  cv::Vec3d d_field = getFieldVectorAtPos(field_vector.pos);
  
  // dF = idL x B
  cv::Vec3d d_force = current * d_field.cross(field_vector.dir);

  return d_force;
}



cv::Mat Coil::renderCoil_xy(cv::Mat& canvas){
  cv::Point offset = cv::Point(canvas_size.width/2, canvas_size.height/2);

  for(int i = 0; i < coil_wire_vectors.size(); i++){
    FieldVector v = coil_wire_vectors[i];
    cv::Point start = cv::Point(v.pos[0], v.pos[1]) + offset;
    cv::Point end = cv::Point(v.pos[0] + v.dir[0], v.pos[1] + v.dir[1]) + offset;
    cv::line(canvas, start, end, cv::Scalar(255, 255, 255), 1);
  }

  return canvas;
}


cv::Mat Coil::renderCoil_xz(cv::Mat& canvas){
  // Center on height
  cv::Point offset = cv::Point(0, canvas_size.height/2);

  for(int i = 0; i < coil_wire_vectors.size(); i++){
    FieldVector v = coil_wire_vectors[i];
    cv::Point start = cv::Point(v.pos[0], v.pos[2]) + offset;
    cv::Point end = cv::Point(v.dir[0] + v.pos[0], v.dir[2] + v.pos[2]) + offset;
    cv::line(canvas, start, end, cv::Scalar(200, 200, 200), 1);
    cv::circle(canvas, start, 2, cv::Scalar(255, 255, 255), -1);
  }

  return canvas;
}


cv::Mat Coil::renderCoil_yz(cv::Mat& canvas){
  // Center on height
  cv::Point offset = cv::Point(0, canvas_size.height/2);

  for(int i = 0; i < coil_wire_vectors.size(); i++){
    FieldVector v = coil_wire_vectors[i];

    cv::Point start = cv::Point(v.pos[1], v.pos[2]) + offset;
    cv::Point end = cv::Point(v.dir[1] + v.pos[1], v.dir[2] + v.pos[2]) + offset;
    cv::line(canvas, start, end, cv::Scalar(255, 255, 0), 1);
  }

  return canvas;
}


void Coil::setCurrent(float _current){
  current = _current;
}





