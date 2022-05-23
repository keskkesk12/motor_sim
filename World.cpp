// stdlib
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iomanip>
#include <algorithm>

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

template <typename T> int sign(T val){
  return (T(0) < val) - (val < T(0));
}


World::World(float _dt, Motor _motor, Controller _controller) :
  dt(_dt), motor(_motor), controller(_controller)
{
  // Initialize magnetic field
  magnetic_field = std::vector<std::vector<cv::Vec3d>>(600, std::vector<cv::Vec3d>(600, cv::Vec3d(0, 0, 0)));
  force_field = std::vector<std::vector<cv::Vec3d>>(600, std::vector<cv::Vec3d>(600, cv::Vec3d(0, 0, 0)));
}


void World::update(){
  time += dt;
}


float World::getTime(){
  return time;
}


void World::generateField(double z){
  // Generate vector field in xy-plane at given z-height

  // Reset magnetic field
  magnetic_field = std::vector<std::vector<cv::Vec3d>>(600, std::vector<cv::Vec3d>(600, cv::Vec3d(0, 0, 0)));

  // Center view
  cv::Vec3d offset(-300, -300, 0);
  
  // Generate field for coils
  for(int n = 0; n < motor.getCoils().size(); n++){
    Coil temp_coil = motor.getCoils()[n];
    for(int y = 0; y < magnetic_field.size(); y++){ // Row or Y
      for(int x = 0; x < magnetic_field[y].size(); x++){ // Collumn or X
        cv::Vec3d pos = cv::Vec3d(x, y, z) + offset;
        magnetic_field[y][x] += temp_coil.getFieldVectorAtPos(pos);
      }
    }
  }

  // Generate field for magnets
  for(int n = 0; n < motor.getMagnets().size(); n++){
    Magnet temp_magnet = motor.getMagnets()[n];
    for(int y = 0; y < magnetic_field.size(); y++){ // Row or Y
      for(int x = 0; x < magnetic_field[y].size(); x++){ // Collumn or X
        cv::Vec3d pos = cv::Vec3d(x, y, z) + offset;
        magnetic_field[y][x] += temp_magnet.getFieldVectorAtPos(pos);
      }
    }
  }
}


cv::Mat World::renderVectorField(){
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0));
  
  cv::cvtColor(canvas, canvas, cv::COLOR_BGR2HSV);

  for(int y = 0; y < canvas.size().height; y++){
    for(int x = 0; x < canvas.size().width; x++){
      cv::Vec3d color = getColor(magnetic_field[y][x]);
      canvas.at<cv::Vec3b>(cv::Point(x, y)) = color;
    }
  }
  cv::cvtColor(canvas, canvas, cv::COLOR_HSV2BGR);

  // Field lines
  for(int y = 0; y < canvas.size().height; y++){
    for(int x = 0; x < canvas.size().width; x++){
      if(((y % 11) == 0) && ((x % 11) == 0)){
        cv::Vec3d field = magnetic_field[y][x];
        cv::Point2d pos = cv::Point2d(x, y);
        cv::Point2d dir = cv::Point2d(field[0], field[1]);
        dir /= cv::norm(dir);
        dir *= 5;
        cv::line(canvas, pos, pos+dir, cv::Scalar(0, 0, 0), 1);
      }
    }
  }

  return canvas;
}


cv::Mat World::renderMagnitudeField(){
  // Color
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0));
  cv::cvtColor(canvas, canvas, cv::COLOR_BGR2HSV);
  for(int y = 0; y < canvas.size().height; y++){
    for(int x = 0; x < canvas.size().width; x++){
      cv::Vec3d color = getColor(cv::norm(magnetic_field[y][x]));
      canvas.at<cv::Vec3b>(cv::Point(x, y)) = color;
    }
  }
  cv::cvtColor(canvas, canvas, cv::COLOR_HSV2BGR);
  
  // Edges
  cv::Mat edge;
  cv::Mat result = cv::Mat(canvas.size(), CV_8UC3, cv::Scalar(0));
  cv::Canny(canvas, edge, 20, 30, 3);
  for(int y = 0; y < canvas.rows; y++){
    for(int x = 0; x < canvas.cols; x++){
      if(edge.at<uint8_t>(cv::Point(x, y)) == 0){
        result.at<cv::Vec3b>(cv::Point(x, y)) = canvas.at<cv::Vec3b>(cv::Point(x, y));
      }
      else{
        result.at<cv::Vec3b>(cv::Point(x, y)) = canvas.at<cv::Vec3b>(cv::Point(x, y)) * 0.9;
      }
    }
  }

  // Field lines
  for(int y = 0; y < result.size().height; y++){
    for(int x = 0; x < result.size().width; x++){
      if(((y % 19) == 0) && ((x % 19) == 0)){
        cv::Vec3d field = magnetic_field[y][x];
        cv::Point2d pos = cv::Point2d(x, y);
        cv::Point2d dir = cv::Point2d(field[0], field[1]);
        dir /= cv::norm(dir);
        dir *= 10;
        cv::line(result, pos, pos+dir, cv::Scalar(255, 255, 255), 1);
      }
    }
  }


  // // Summed up mag
  // cv::Vec3d mag_dir(0, 0, 0);
  // for(int y = 0; y < result.size().height; y++){
  //   for(int x = 0; x < result.size().width; x++){
  //     if(cv::norm(magnetic_field[y][x]) < 1000){
  //       mag_dir += magnetic_field[y][x];
  //     }
  //   }
  // }
  // cv::Point offset = cv::Point(canvas_size.width/2, canvas_size.height/2);
  // cv::line(result, offset, offset + cv::Point(mag_dir[0], mag_dir[1]), cv::Scalar(123, 23, 241), 1);

  return result;
}




void World::generateForceField(){
  // Reset force field
  force_field = std::vector<std::vector<cv::Vec3d>>(600, std::vector<cv::Vec3d>(600, cv::Vec3d(0, 0, 0)));

  for(int y = 0; y < canvas_size.height; y++){
    for(int x = 0; x < canvas_size.width; x++){
      float angle = atan2(magnetic_field[y][x][1], magnetic_field[y][x][0]);

      Dipole test_dipole = Dipole(cv::Point2f(-300 + x, -300 + y), angle, 100, 1, 4);

      force_field[y][x] = motor.getForceOnDipoleAtPos(test_dipole);
    }
  }
}


cv::Mat World::renderNorthSouth(){
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0));
  // cv::cvtColor(canvas, canvas, cv::COLOR_BGR2HSV);

  for(int x = 0; x < canvas.size().width; x++){
    for(int y = 0; y < canvas.size().height; y++){
      cv::Vec3d force_vec = force_field[y][x];
      cv::Vec3d magnetic_field_vec = magnetic_field[y][x];

      float value = force_vec.dot(magnetic_field_vec);
      float mag = sign(value)*log10(abs(value)+1);

      int color = mag * 50;

      color = std::min(255, color);
      color = std::max(-255, color);
      
      if(color > 0){
        canvas.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(0, 0, color);
      }else{
        canvas.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(-color, 0, 0);
      }

    }
  }

  // cv::cvtColor(canvas, canvas, cv::COLOR_HSV2BGR);
  return canvas;
}





cv::Vec3b World::getColor(float field_strength){
  double magnitude = sign(field_strength)*log10(abs(field_strength)+1);
  cv::Vec3b color;
  color[0] = 90 + magnitude * 15;
  color[1] = 255;
  color[2] = 255;
  return color;
}


cv::Vec3b World::getColor(cv::Vec3d vec){
  cv::Vec3b color;
  color[0] = 90 + atan2(vec[1], vec[0]) * 180.0/M_PI/2;
  color[1] = 255;
  color[2] = 255;
  return color;
}


std::vector<std::vector<cv::Vec3d>> World::getMagneticField(){
  return magnetic_field;
}


std::vector<std::vector<cv::Vec3d>> World::getForceField(){
  return force_field;
}


