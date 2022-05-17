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



Magnet::Magnet(float _radius, float _angle, float _orientation, float d, float h, float i_density, int _res, bool _polarity) : 
  radius(_radius), angle(_angle), orientation(_orientation), depth(d), height(h), current_density(i_density), polarity(_polarity), res(_res)
{
  generateDipolesPolar(0);
  // generateDipolesCartesian();
}


void Magnet::generateDipolesPolar(float rotor_angle){
  dipoles.clear();

  for(float d_theta = 0; d_theta < angle; d_theta+=0.02){
    for(int d = 0; d < depth; d+=3){
      for(int h = 0; h < height; h+=2){

        float offset = radius + depth;
        int current = (polarity) ? current_density : -current_density;
        float dipole_radius = angle * radius / (M_PI);

        Dipole temp_dipole(offset, height, orientation + d_theta + rotor_angle, current, 1, res);
        dipoles.push_back(temp_dipole);
      }
    }
  }
}


void Magnet::generateDipolesCartesian(){
  // for(int i = 0; i < 10; i++){
  //   cv::Point2f pos = cv::Point2f(-150 + 20*i, 0);
  //   float angle = 2*M_PI/360.0 * i * 10;
  //   Dipole temp_dipole(pos, angle, 1000, 20, 20);
  //   dipoles.push_back(temp_dipole);
  // }
  cv::Point2f pos = cv::Point2f(0,0);
  float angle = 0;
  Dipole temp_dipole(pos, angle, 1000, 20, 20);
  dipoles.push_back(temp_dipole);
}


cv::Vec3d Magnet::getFieldVectorAtPos(cv::Vec3d pos){
  cv::Vec3d d_field;
  for(int i = 0; i < dipoles.size(); i++){
    Dipole temp_dipole = dipoles[i];
    d_field += temp_dipole.getFieldVectorAtPos(pos);
  }  
  return d_field;
}


cv::Mat Magnet::renderMagnet_xy(){
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0, 0, 0));

  cv::Point offset = cv::Point(canvas_size.width/2, canvas_size.height/2);

  for(int n = 0; n < dipoles.size(); n++){
    Dipole temp_dipole = dipoles[n];
    for(int i = 0; i < temp_dipole.dipole_wire_vectors.size(); i++){
      FieldVector v = temp_dipole.dipole_wire_vectors[i];
      cv::Point start = cv::Point(v.pos[0], v.pos[1]) + offset;
      cv::Point end = cv::Point(v.pos[0] + v.dir[0], v.pos[1] + v.dir[1]) + offset;
      if(polarity){
        cv::line(canvas, start, end, cv::Scalar(0, 0, 255), 1);
      }
      else{
        cv::line(canvas, start, end, cv::Scalar(255, 0, 0), 1);
      }
    }
  }
  return canvas;
}


cv::Mat Magnet::renderMagnet_xz(){
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0, 0, 0));
  
  cv::Point offset = cv::Point(0, canvas_size.height/2);

  for(int n = 0; n < dipoles.size(); n++){
    Dipole temp_dipole = dipoles[n];
    for(int i = 0; i < temp_dipole.dipole_wire_vectors.size(); i++){
      FieldVector v = temp_dipole.dipole_wire_vectors[i];
      cv::Point start = cv::Point(v.pos[0], v.pos[2]) + offset;
      cv::Point end = cv::Point(v.pos[0] + v.dir[0], v.pos[2] + v.dir[2]) + offset;
      cv::line(canvas, start, end, cv::Scalar(0, 255, 0), 1);
    }
  }
  return canvas;
}


cv::Mat Magnet::renderMagnet_yz(){
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0, 0, 0));

  cv::Point offset = cv::Point(0, canvas_size.height/2);

  for(int n = 0; n < dipoles.size(); n++){
    Dipole temp_dipole = dipoles[n];
    for(int i = 0; i < temp_dipole.dipole_wire_vectors.size(); i++){
      FieldVector v = temp_dipole.dipole_wire_vectors[i];
      cv::Point start = cv::Point(v.pos[1], v.pos[2]) + offset;
      cv::Point end = cv::Point(v.pos[1] + v.dir[1], v.pos[2] + v.dir[2]) + offset;
      cv::line(canvas, start, end, cv::Scalar(0, 255, 0), 1);
    }
  }
  return canvas;
}


std::vector<Dipole> Magnet::getDipoles(){
  return dipoles;
}