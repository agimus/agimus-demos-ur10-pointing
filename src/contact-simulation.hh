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

#ifndef AGIMUS_DEMOS_CONTACT_SIMULATION_HH
#define AGIMUS_DEMOS_CONTACT_SIMULATION_HH

#include <dynamic-graph/all-commands.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-geometry.hh>

#include <agimus/demos/config.hh>

namespace dynamicgraph{
namespace agimus{

/// Simulation of the force applied by a horizontal plane on the left
/// gripper of talos when there is a contact between them.
///
/// This entity is only for testing ContactAdmittance behavior since
/// Gazebo contact simulation is not satisfactory.

class AGIMUS_DEMOS_DLLAPI ContactSimulation : public Entity
{
 public:
  typedef dynamicgraph::sot::MatrixHomogeneous MatrixHomogeneous;
  static const std::string CLASS_NAME;
  virtual void display(std::ostream &os) const;
  virtual const std::string &getClassName(void) const { return CLASS_NAME; }

  /// Constructor
  ContactSimulation(const std::string& name);

 private:
  // Input signal
  SignalPtr<MatrixHomogeneous, int> wristPoseSIN;
  // Output signal
  SignalTimeDependent<Vector, int> wrenchSOUT;

  // compute the wrench with respect to the wrist pose. The function makes the
  // asumption that the horizontal plane is at 0.75 above the ground. This
  // correspond to the rolling table provided by gerard-bauzil package.
  Vector& computeWrench(Vector& res, int time);
  // Position of the fingers in wrist frame: each colum of the matrix stores
  // the position of one finger.
  Eigen::Matrix<double, 3, 3> fingers_;
  // Height of the horizontal plane
  double planeHeight_;
  // stiffness of the contact
  double stiffness_;
  // Temporary variables for computation
  // finger positions in world frame
  mutable Eigen::Matrix<double, 3, 3> wFingers_;
  // Contact forces in world frame
  mutable Eigen::Matrix<double, 3, 3> wForces_;
  // Contact forces in wrist joint frame
  mutable Eigen::Matrix<double, 3, 3> jForces_;
  // Orientation of the wrist
  mutable Eigen::Matrix<double, 3, 3> wRj_;
  // Translation of the wrist
  mutable Eigen::Matrix<double, 3, 1> wtj_;
};
  
}// namespace agimus
}// namespace dynamicgraph
#endif // AGIMUS_DEMOS_CONTACT_SIMULATION_HH
