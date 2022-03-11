// stdlib
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iomanip>
#include <string>

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


/* 
  Simulation of motor dynamics:
    V1: Render magnetic fields from coils and dipoles
    V2: Simulate torque and other dynamics on motor from magnetic field
    V3: Simulate motor dynamics and control from torque graph

  World class keeps track of time. All other classes has an update function which takes the world time as input and updates according to their dt

 */



void onMouse (int event, int y, int x, int flags, void* param);



int main(){
  std::string name = "window";
  cv::namedWindow(name);
  Motor motor(1, 10, 10, 0.00001);
  // motor.generateDl();
  motor.generateCoils(200, -100, 100, 20, 100);
  cv::Mat motor_render = motor.renderMotorCoils();

  Controller controller;

  World world(0.0001, motor, controller);
  world.generateField();
  cv::Mat vector_field = world.renderVectorField();
  cv::Mat magnitude_field = world.renderMagnitudeField();

  cv::setMouseCallback(name, onMouse, (void*)&world);

  // Coil coil(300,100, 3, 0, cv::Point2d(0, 0), 0, 12, 0.00001);
  // Coil coil(300,100, 3, M_PI/8, cv::Point2f(100, 100), 0, 50, 0.00001);
  // cv::Mat img1 = coil.renderCoil_xz();
  // cv::Mat img2 = coil.renderCoil_yz();
  // cv::Mat img3 = coil.renderCoil_xy();

  while(1){
    cv::imshow("motor", motor_render);
    cv::imshow("vector", vector_field);
    cv::imshow(name, magnitude_field);
    // cv::imshow("xz", img1);
    // cv::imshow("yz", img2);
    // cv::imshow("xy", img3);

    char key = cv::waitKey(0);
    if(key == 'q'){
      break;
    }
  }
  return 0;
}


void onMouse (int event, int x, int y, int flags, void* param){
  World& world = *((World*)param);
  if(event == cv::EVENT_LBUTTONDOWN){
    std::cout << world.getMagneticField()[y][x] << std::endl;
  }
}

