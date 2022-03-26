// Copyright 2022 CNRS - Toward SAS
// Author: Florent Lamiraux
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:

// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <dynamic-graph/factory.h>
#include "contact-simulation.hh"

namespace dynamicgraph{
namespace agimus{

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ContactSimulation, "ContactSimulation");
void ContactSimulation::display(std::ostream& os) const
{
  os << "ContactSimulation " << getName();
}

ContactSimulation::ContactSimulation(const std::string& name):
  Entity(name),
  wristPoseSIN
  (0x0, "ContactSimulation("+name+")::input(MatriHomo)::wristPose"),
  wrenchSOUT(boost::bind(&ContactSimulation::computeWrench, this, _1, _2),
             wristPoseSIN,
             "ContactSimulation("+name+")::output(vector)::wrench"),
  planeHeight_(0.75), stiffness_(1e4)
{
  fingers_ <<
    -0.025,  0   ,  0.025,
    0.05 , -0.05,  0.05,
    -0.3 , -0.3, -0.3;
  wForces_.setZero();
  signalRegistration(wristPoseSIN << wrenchSOUT);
}

Vector& ContactSimulation::computeWrench(Vector& res, int time)
{
  res.resize(6);
  const MatrixHomogeneous& wristPose(wristPoseSIN(time));
  // Rotation of wrist
  wRj_ = wristPose.matrix().topLeftCorner<3,3>();
  // Translation of wrist
  wtj_ = wristPose.matrix().topRightCorner<3,1>();
  for (Matrix::Index i=0; i<fingers_.cols(); ++i){
    wFingers_.col(i) = wristPose * fingers_.col(i);
    if ((wRj_ * fingers_.col(i) + wtj_ - wFingers_.col(i)).squaredNorm() >
        1e-12) {
      std::cout << "ContactSimulation squared norm = " <<
        (wRj_ * fingers_.col(i) + wtj_ - wFingers_.col(i)).squaredNorm()
                << std::endl;
      assert(false);
    }
    // Height of the finger
    double z = wFingers_(2,i);
    if (z < planeHeight_){
      wForces_(2,i) = stiffness_ * (planeHeight_ - z);
    } else {
      wForces_(2,i) = 0;
    }
  }
  // Forces in local joint frame
  jForces_ = wRj_.transpose()*wForces_;
  res.setZero();
  for (Matrix::Index i=0; i<fingers_.cols(); ++i){
    res.head<3>() += jForces_.col(i);
    res.tail<3>() += fingers_.col(i).cross(jForces_.col(i));
  }
  return res;
}
}// namespace agimus
}// namespace dynamicgraph
