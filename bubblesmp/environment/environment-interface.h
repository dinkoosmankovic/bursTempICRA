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

#ifndef COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_ENVIRONMENT_INTERFACE_H_
#define COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_ENVIRONMENT_INTERFACE_H_

#include <utility>
#include <vector>

#include <fcl/distance.h>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {

class EnvironmentInterface {
public:
    virtual ~EnvironmentInterface() {}
    typedef std::vector<std::pair<double, std::vector<double> > > DistanceProfile;

    virtual bool IsCollision(const std::vector<double>& q) const = 0;
    virtual bool IsCollisionAugmented(const std::vector<double>& q) const = 0;
    virtual DistanceProfile GetDistanceProfile(
            const std::vector<double>& q) const = 0;
    virtual std::vector<fcl::Vec3f> GetJointPositions(
            const std::vector<double>& q) const = 0;
    virtual double GetDmin(
            const std::vector<double>& q) const = 0;
    virtual double GetDmin(const std::vector<double> &q, std::vector<std::pair<fcl::Vec3f, fcl::Vec3f> > &points) const = 0;
    virtual std::vector<std::pair<double, double> > GetAngleRanges() const = 0;
    virtual std::vector<fcl::Vec3f> getOBBVertices(int segment) const = 0;
};

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_ENVIRONMENT_ENVIRONMENT_INTERFACE_H_
