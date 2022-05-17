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




Motor::Motor(int _poles, float r, float I, float _dt) :
  radius(r), inertia(I), poles(_poles), dt(_dt)
{}


// Generator methods
void Motor::generateCoils(float l, float offset, float r, int N, int res){
  for(int i = 0; i < poles; i++){
    float angle = i*2*M_PI/poles;
    cv::Point2d pos(0, 0);
    Coil temp_coil = Coil(l, r, N, angle, pos, offset, res, dt);
    // Push back to UVW vectors
    if((i%3) == 0){ // U
      U.push_back(temp_coil);
    }else if((i%3) == 1){ // V
      V.push_back(temp_coil);
    }else if((i%3) == 2){ // W
      W.push_back(temp_coil);
    }
  }
}


void Motor::generateMagnets(int N_pairs, int I, float depth, float height, float radius, int res){
  float angle = 2*M_PI / (N_pairs * 2);

  for(int i = 0; i < N_pairs; i++){
    float orientation = 2*i*angle;

    Magnet magnet_north(radius, angle, orientation, depth, height, I, res, false);
    Magnet magnet_south(radius, angle, orientation + angle, depth, height, I, res, true);
    magnets.push_back(magnet_north);
    magnets.push_back(magnet_south);
  }
}


std::vector<float> Motor::generateTorqueRippleVector(){
  std::vector<float> torque_curve;
  // For full rotation
  for(int theta_deg = 0; theta_deg < 360; theta_deg++){
    // Set angle of rotor and current vector
    float theta_rad = float(theta_deg) * DEG_2_RAD;

    setRotorAngle(theta_rad + M_PI);
    setCurrentVector(theta_rad, 1);

    torque_curve.push_back(calculateTorque());
  }
  return torque_curve;
}


float Motor::calculateTorque(){
  std::vector<Dipole> dipoles;
  // Get all dipoles in motor
  for(int i = 0; i < magnets.size(); i++){
    std::vector<Dipole> temp_dipoles = magnets[i].getDipoles();

    for(int j = 0; j < temp_dipoles.size(); j++){
      dipoles.push_back(temp_dipoles[j]);
    }
  }

  float torque = 0;

  for(int coil_num = 0; coil_num < getCoils().size(); coil_num++){
    Coil coil = getCoils()[coil_num];
      
    for(int dipole_num = 0; dipole_num < dipoles.size(); dipole_num++){

      Dipole dipole = dipoles[dipole_num];
      std::vector<FieldVector> dipole_field_vectors = dipole.dipole_wire_vectors;

      for(int dipole_field_vector_num = 0; dipole_field_vector_num < dipole_field_vectors.size(); dipole_field_vector_num++){
        FieldVector field_vector = dipole_field_vectors[dipole_field_vector_num];
        
        cv::Vec3d force = coil.forceOnWireDL(field_vector, dipole.getCurrent());
        cv::Vec3d d_torque = field_vector.pos.cross(force);

        torque += d_torque[2];
      }
    }
  }
  return torque;
}


// Set methods
void Motor::setRotorAngle(float angle){
  for(int i = 0; i < magnets.size(); i++){
    Magnet& magnet = magnets[i];
    magnet.generateDipolesPolar(angle);
  }
}


void Motor::setCurrentVector(cv::Vec2d current_vector){
  current = clarkInv(current_vector);


  for(int i = 0; i < U.size(); i++){
    U[i].setCurrent(current[0]);
  }
  for(int i = 0; i < V.size(); i++){
    V[i].setCurrent(current[1]);
  }
  for(int i = 0; i < W.size(); i++){
    W[i].setCurrent(current[2]);
  }
}


void Motor::setCurrentVector(float angle, float magnitude){
  cv::Vec2d vec(0, 0);
  vec[0] = cos(angle) * magnitude;
  vec[1] = sin(angle) * magnitude;
  setCurrentVector(vec);
}


// Get metods
std::vector<Coil> Motor::getCoils(){
  std::vector<Coil> coils;

  for(int i = 0; i < U.size(); i++){
    coils.push_back(U[i]);
  }
  for(int i = 0; i < V.size(); i++){
    coils.push_back(V[i]);
  }
  for(int i = 0; i < W.size(); i++){
    coils.push_back(W[i]);
  }

  return coils;
}


std::vector<Magnet> Motor::getMagnets(){
  return magnets;
}


cv::Vec3d Motor::getCurrents(){
  return current;
}


// Render methods
cv::Mat Motor::renderMotorCoils(){
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0));
  for(int i = 0; i < getCoils().size(); i++){
    cv::add(canvas, getCoils()[i].renderCoil_xy(), canvas);
  }
  return canvas;
}


cv::Mat Motor::renderMagnets(){
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0));
  for(int i = 0; i < magnets.size(); i++){
    cv::add(canvas, magnets[i].renderMagnet_xy(), canvas);
  }
  return canvas;
}

cv::Mat Motor::renderMotor(){
  cv::Mat canvas = renderMotorCoils();
  cv::add(canvas, renderMagnets(), canvas);
  cv::add(canvas, renderCurrentVector(), canvas);
  return canvas;
}


cv::Mat Motor::renderCurrentVector(){
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(0));

  // Convert currents to xyz-frame
  cv::Vec2d ab = clark(current);

  cv::Point offset = cv::Point(canvas_size.width/2, canvas_size.height/2);
  cv::line(canvas, offset, offset + cv::Point(ab[0], ab[1]), {0, 255, 255}, 1);

  return canvas;
}


