// Main Loop for the Vision Module
// Main Loop for the Vision Module
// Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <vision_module/Vision.hpp>
#include <vision_module/Tools/VisoinRate.hpp>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/LineSegmentTester.hpp>
#include <vision_module/Tools/ContourTester.hpp>
#include <math.h>
#include <vision_module/Tools/General.hpp>
#include <vision_module/Tools/VisionTimer.hpp>
#include <vision_module/Tools/LinearInterpolator.hpp>
#include <vision_module/Projections/DistortionModel.hpp>
#include <nimbro_op_interface/Buzzer.h>
#include <walk_and_kick/BehaviourState.h>
int main(int,char**);
