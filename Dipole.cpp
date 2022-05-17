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



// Polar constructor for dipole
Dipole::Dipole(float offset, float height, float _orientation, float _current, float _radius, int res) : 
  current(_current), orientation(_orientation), radius(_radius)
{
  // Generate dipole
  float d_theta = 2 * M_PI / res;

  cv::Point3f start(offset, 0, 0);
  cv::Point3f end(offset, _radius, 0);
  cv::Mat start_mat = (cv::Mat_<float>(4, 1) << start.x, start.y, start.z, 1);
  cv::Mat end_mat = (cv::Mat_<float>(4, 1) << end.x, end.y, end.z, 1);
  cv::Mat dir_mat = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);

  // Make coil transform
  cv::Mat coil_transform(cv::Size(4, 4), CV_32FC1, cv::Scalar(0));
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
    orientation_transform.at<float>(2, 3) = height;
    // Set rotation
    orientation_transform.at<float>(0, 0) = cos(orientation);
    orientation_transform.at<float>(0, 1) = -sin(orientation);
    orientation_transform.at<float>(1, 0) = sin(orientation);
    orientation_transform.at<float>(1, 1) = cos(orientation);
    orientation_transform.at<float>(2, 2) = 1;
    // Scale
    orientation_transform.at<float>(3, 3) = 1;

  for(int i = 0; i < res; i++){
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
    dipole_wire_vectors.push_back(field_vector);
  }
}


// Cartesian constructor for dipole
Dipole::Dipole(cv::Point2f _pos, float _orientation, float _current, float _radius, float res) : 
  current(_current), orientation(_orientation), radius(_radius)
{
  /* 
    Idea of this constructor:
      Generate dipole at center(0, 0, 0)
      Rotate to given orientation
      Move to given position
   */

  // Generate dipole
  float d_theta = 2 * M_PI / res;

  cv::Point3f end(0, _radius, 0);
  cv::Mat start_mat = (cv::Mat_<float>(4, 1) << 0, 0, 0);
  cv::Mat end_mat = (cv::Mat_<float>(4, 1) << end.x, end.y, end.z, 1);
  cv::Mat dir_mat = (cv::Mat_<float>(4, 1) << 0, 0, 0, 0);
  cv::Mat offset_mat = (cv::Mat_<float>(4, 1) << _pos.x, _pos.y, 0, 0);

  // Make coil transform
  cv::Mat coil_transform(cv::Size(4, 4), CV_32FC1, cv::Scalar(0));
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
    orientation_transform.at<float>(2, 3) = 0;
    // Set rotation
    orientation_transform.at<float>(0, 0) = cos(orientation);
    orientation_transform.at<float>(0, 1) = -sin(orientation);
    orientation_transform.at<float>(1, 0) = sin(orientation);
    orientation_transform.at<float>(1, 1) = cos(orientation);
    orientation_transform.at<float>(2, 2) = 1;
    // Scale
    orientation_transform.at<float>(3, 3) = 1;

  for(int i = 0; i < res; i++){
    FieldVector field_vector;
    // Set start equals to end
    end_mat.copyTo(start_mat);
    // Rotate end
    end_mat = coil_transform * end_mat;
    // Set desired z-orientation
    cv::Mat end_mat_rotated = orientation_transform * end_mat;
    cv::Mat start_mat_rotated = orientation_transform * start_mat;
    // Offset transformation
    end_mat_rotated = end_mat_rotated + offset_mat;
    start_mat_rotated = start_mat_rotated + offset_mat;
    // Dir = end - start
    dir_mat = end_mat_rotated - start_mat_rotated;
    // Set field vector values
    field_vector.pos = cv::Vec3d(start_mat_rotated.at<float>(0, 0), start_mat_rotated.at<float>(1, 0), start_mat_rotated.at<float>(2, 0));
    field_vector.dir = cv::Vec3d(dir_mat.at<float>(0, 0), dir_mat.at<float>(1, 0), dir_mat.at<float>(2, 0));
    dipole_wire_vectors.push_back(field_vector);
  }
}


cv::Vec3d Dipole::getFieldVectorAtPos(cv::Vec3d pos){
  cv::Vec3d d_field;
  int coil_wire_vectors_size = dipole_wire_vectors.size();
  for(int i = 0; i < coil_wire_vectors_size; i++){
    FieldVector temp_field_vector = dipole_wire_vectors[i];
    d_field += calcFieldStrength(temp_field_vector, pos);
  }
  return d_field;
}


cv::Vec3d Dipole::calcFieldStrength(FieldVector vec, cv::Vec3d p){
  float u0 = 1;
  cv::Vec3d r_vec = p - vec.pos;
  float r = cv::norm(r_vec);
  cv::Vec3d r_hat = r_vec / r;
  cv::Vec3d ds = vec.dir;
  return (current * ds.cross(r_hat)) / (pow(r, 2));
}


cv::Vec3d Dipole::forceOnWireDL(FieldVector field_vector, float current){
  // Calculates magnetic field set up at position of wire-dL from dipole
  cv::Vec3d d_field = getFieldVectorAtPos(field_vector.pos);
  
  // dF = idL x B
  cv::Vec3d d_force = current * field_vector.dir.cross(d_field);

  return d_force;
}


// Get methods
float Dipole::getCurrent(){
  return current;
}

