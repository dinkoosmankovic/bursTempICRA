//
// Copyright (c) 2015, Adnan Ademovic
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#ifndef COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_ENVIRONMENT_FEEDBACK_H_
#define COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_ENVIRONMENT_FEEDBACK_H_

#include <vector>
#include <memory>

#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>

namespace com {
namespace ademovic {
namespace bubblesmp {

class Bubble;
class Bur;

namespace environment {

class EnvironmentInterface;

class EnvironmentFeedback {
 public:
  explicit EnvironmentFeedback(EnvironmentInterface* environment);
  virtual ~EnvironmentFeedback();
  // Caller needs to handle ownership of the returned Bubble.
  Bubble* NewBubble(
      const std::vector<double>& coordinates, bool extended = true) const;

  Bur* NewBur(const std::vector<double>& coordinates,
          const std::vector<double>& q_target, int burSize = 7) const;

  Bur* NewGenBur(const std::vector<double>& coordinates,
          const std::vector<double>& q_target, double dmin) const;

  bool IsCollision(const std::vector<double>& coordinates) const;
  bool IsCollisionAugmented(const std::vector<double>& coordinates) const;
  std::vector<std::pair<double, double> > GetAngleRanges() const;

  std::vector<double> getRs(
          const std::vector<double>& coordinates) const;

  double optimizeBurNeedle(const std::vector<double> &q, std::vector<double> &qe,
                           double dmin, const std::vector<double> &rs) const;

  std::vector<fcl::Vec3f> GetJointPositions(const std::vector<double> &q) const;

  std::vector<double> generatePoint(const std::vector<double> &qnear) const;
  std::vector<double> generatePoint(int size) const;
  double calculateDeltaK(double fi, const std::vector<double> &rs, const std::vector<double> &qe,
                         const std::vector<double> &qk) const;
  double calculateFi(double dmin, const std::vector<fcl::Vec3f>& pn,
                     const std::vector<fcl::Vec3f>& p0) const;
  double norm(const std::vector<double> &p1, const std::vector<double> &p2) const;
  double norm(std::vector<double> &vec) const;
  double getDmin(const std::vector<double> &coordinates) const;
  double getDmin(const std::vector<double> &coordinates, std::vector<std::pair<fcl::Vec3f, fcl::Vec3f> > &points) const;
  double norm1(const std::vector<double> &p1, const std::vector<double> &p2) const;

  std::vector<fcl::Vec3f> getOBBVertices(int segment) const;

private:
  std::unique_ptr<EnvironmentInterface> environment_;
};

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_ENVIRONMENT_FEEDBACK_H_
