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
#include "Coil.hpp"
#include "Dipole.hpp"
#include "util.hpp"
#include "Magnet.hpp"





class Motor {
  std::vector<Coil> U;
  std::vector<Coil> V;
  std::vector<Coil> W;
  std::vector<Magnet> magnets;
  float rotor_angle;
  float radius;
  float inertia;
  int poles;
  float dt;
  float torque;
  cv::Vec3d current; // U-V-W

public:
  Motor(int poles, float r, float inertia, float dt);
  void generateCoils(float l, float offset, float r, int N, int res);
  void generateMagnets(int N, int I, float depth, float height, float radius, int res);
  std::vector<float> generateTorqueRippleVector();
  float calculateTorque();
  void update(float dt);

  // Set
  void setRotorAngle(float angle);
  void setVoltages(float U, float V, float W);
  void setCurrents(float U, float V, float W);
  void setCurrentVector(cv::Vec2d);
  void setCurrentVector(float angle, float magnitude);

  // Get
  float getAngle();
  cv::Vec3d getCurrents();
  std::vector<Coil> getCoils();
  std::vector<Magnet> getMagnets();
  cv::Vec3d getForceOnDipoleAtPos(Dipole);

  // Render
  cv::Mat renderMotorCoils(cv::Mat& canvas);
  cv::Mat renderMagnets(cv::Mat& canvas);
  cv::Mat renderMotor();
  cv::Mat renderCurrentVector();
};

