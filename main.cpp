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
  Motor motor(3, 0, 10, 0.00001);
  motor.generateCoils(60, 60, 50, 10, 10);
  // motor.generateMagnets(1, 10, 1, 1, 180, 16);
  Controller controller;


  for(int i = 0; i < 100; i++){
    float angle = 2*M_PI/100.0 * float(i);

    motor.setCurrentVector(angle, 100);
    // motor.setRotorAngle(angle);
    // float torque = motor.calculateTorque();
    // std::cout << "Torque " << torque << std::endl;
    // cv::Mat motor_render = motor.renderMotor();

    World world(0.0001, motor, controller);
    // cv::setMouseCallback(name, onMouse, (void*)&world);

    world.generateField(0);
    // cv::Mat vector_field = world.renderVectorField();
    // cv::Mat magnitude_field = world.renderMagnitudeField();

    world.generateForceField();
    cv::Mat north_south = world.renderNorthSouth();

    // Coil coil(300,100, 3, 0, cv::Point2d(0, 0), 0, 12, 0.00001);
    // cv::Mat img1 = coil.renderCoil_xz();
    // cv::Mat img2 = coil.renderCoil_yz();
    // cv::Mat img3 = coil.renderCoil_xy();

    cv::imwrite("north_south/" + std::to_string(i) + ".png", north_south);
  }

  // while(1){
  //   // cv::imshow("motor", motor_render);
  //   cv::imshow("vector", vector_field);
  //   cv::imshow("magnitude", magnitude_field);
  //   cv::imshow(name, north_south);
  //   // cv::imshow("xz", img1);
  //   // cv::imshow("yz", img2);
  //   // cv::imshow("xy", img3);

  //   char key = cv::waitKey(100);
  //   if(key == 'q'){
  //     break;
  //   }
  // }
  return 0;
}


void onMouse (int event, int x, int y, int flags, void* param){
  World& world = *((World*)param);
  if(event == cv::EVENT_LBUTTONDOWN){
    std::cout << world.getForceField()[y][x] << std::endl;
  }
}

