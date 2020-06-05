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

#include "bubblesmp/environment/environment-feedback.h"

#include <cmath>
#include <limits>
#include <random>
#include <algorithm>
#include <utility>

#include "bubblesmp/bubble.h"
#include "bubblesmp/bur.h"
#include "bubblesmp/environment/environment-interface.h"

#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>

#include <glog/logging.h>

namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace {

constexpr double rad_to_deg() {
    return 45.0 / atan(1);
}

constexpr double deg_to_rad() {
    return atan(1) / 45.0;
}

}  // namespace

std::default_random_engine generator;
std::normal_distribution<double> distribution(0,1);
const double PI = atan(1) * 4;

EnvironmentFeedback::~EnvironmentFeedback() {}

double EnvironmentFeedback::norm(std::vector<double>& vec) const
{
    double result = 0;
    for (unsigned int i = 0; i < vec.size(); i++)
    {
        result += vec[i] * vec[i];
    }
    return sqrt(result);
}

double EnvironmentFeedback::norm(const std::vector<double> &p1, const std::vector<double> &p2) const
{
    double sum = 0;
    for (unsigned int i = 0; i < p1.size(); i++)
        sum += ( p1[i]*deg_to_rad() - p2[i]*deg_to_rad() )
                * ( p1[i]*deg_to_rad() - p2[i]*deg_to_rad() );
    return sqrt(sum);
    /*std::vector<fcl::Vec3f> joint_positions_qn(
                environment_->GetJointPositions(p1));

    std::vector<fcl::Vec3f> joint_positions_q0(
                environment_->GetJointPositions(p2));

    double max_norm = -1e9;

    for (unsigned int i = 0; i < joint_positions_qn.size(); i++) {
        fcl::Vec3f temp = joint_positions_qn[i] - joint_positions_q0[i];
        max_norm = std::max( max_norm, temp.length() );
    }
    return max_norm;*/
}

double EnvironmentFeedback::norm1(const std::vector<double> &p1, const std::vector<double> &p2) const
{
    /*double sum = 0;
    for (unsigned int i = 0; i < p1.size(); i++)
        sum += ( p1[i] - p2[i] )
                * ( p1[i] - p2[i] );
    return sqrt(sum);*/
    std::vector<fcl::Vec3f> joint_positions_qn(
                environment_->GetJointPositions(p1));

    std::vector<fcl::Vec3f> joint_positions_q0(
                environment_->GetJointPositions(p2));

    double max_norm = -1*std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < joint_positions_qn.size(); i++) {
        fcl::Vec3f temp = joint_positions_qn[i] - joint_positions_q0[i];
        max_norm = std::max( max_norm, temp.length() );
    }
    return max_norm;
}

std::vector<fcl::Vec3f> EnvironmentFeedback::getOBBVertices(int segment) const
{
    return environment_->getOBBVertices(segment);
}

/*std::vector<double> normalizeVector(std::vector<double>& p, double R)
{
    vector<double> result(p.size());
    for (unsigned int i = 0; i < p.size(); i++)
    {
        result[i] = R * p[i]/norm(p);
    }
    return result;
}*/

double EnvironmentFeedback::calculateFi(double dmin, const std::vector<fcl::Vec3f> &pn,
                                        const std::vector<fcl::Vec3f> &p0) const
{

    double max_norm = -std::numeric_limits<double>::infinity();

    for (unsigned int i = 0; i < pn.size(); i++) {
        fcl::Vec3f temp = pn[i] - p0[i];
        max_norm = std::max( max_norm, temp.length() );
    }

    return dmin - max_norm / 1000.0 ;
}

double EnvironmentFeedback::calculateDeltaK(double fi, const std::vector<double>& rs,
                                            const std::vector<double>& qe,
                                            const std::vector<double>& qk) const
{
    double result = 0;
    for (unsigned int i = 0; i < qe.size(); i++)
    {
        result += rs[i] / 1000.0 * fabs( qe[i]*deg_to_rad() - qk[i]*deg_to_rad() );
    }
    return fi / result;
}

std::vector<double> EnvironmentFeedback::generatePoint(int size) const
{
    std::vector<double> point(size);
    for (int i = 0; i < size; i++)
        point[i] = distribution(generator);

    double resultNorm = norm(point);
    for (int i = 0; i < size; i++) {
        point[i] *= PI / resultNorm;
        point[i] *= rad_to_deg();
    }

    return point;

}

std::vector<double> EnvironmentFeedback::generatePoint(const std::vector<double>& qnear) const
{
    std::vector<double> point(generatePoint(qnear.size()));

    /*for (unsigned int i = 0; i < qnear.size(); i++)
        point[i] *= 180.0 / resultNorm;*/

    double _norm = norm(point, qnear);
    for (unsigned int i = 0; i < qnear.size(); i++) {
        point[i] =  qnear[i]*deg_to_rad() + PI * (point[i]*deg_to_rad() - qnear[i]*deg_to_rad() ) /
                _norm;
        point[i] *= rad_to_deg();
    }

    //std::transform(point.begin(),point.end(),qnear.begin(),
    //               point.begin(),std::plus<double>());

    return point;
}

double EnvironmentFeedback::optimizeBurNeedle(const std::vector<double>& q,
                                              std::vector<double>& qe,
                                              double dmin, const std::vector<double>& rs) const
{
    std::vector<double> qk(q); double tk = 0; int iter = 0;
    std::vector<fcl::Vec3f> joint_positions_qn(
                environment_->GetJointPositions(q));

    for (iter = 0; iter < 5; iter++)
    {
        double fi_min = std::numeric_limits<double>::infinity();

        std::vector<fcl::Vec3f> joint_positions_q0(
                    environment_->GetJointPositions(qk));

        for (unsigned int i = 0; i < q.size(); i++)
        {
            fi_min = std::min(fi_min, calculateFi(dmin, joint_positions_qn, joint_positions_q0) );
            //qk[i] = q[i] + tk * (qe[i] - q[i]);
        }
        double deltaK = calculateDeltaK(fi_min, rs, qe, qk);
        tk = tk +  deltaK * (1 - tk);
        if (tk >= 1) {
            return tk;}

        for (unsigned int i = 0; i < q.size(); i++)
        {
            qk[i] = q[i]*deg_to_rad() + tk * (qe[i]*deg_to_rad()  - q[i]*deg_to_rad() );
            qk[i] = qk[i]*rad_to_deg();
        }

        if (fi_min/dmin < 0.05)
        {
            qe = qk;
            return tk;
        }

    }
    //exit(1);
    qe = qk;
    return tk;
}

std::vector<fcl::Vec3f> EnvironmentFeedback::GetJointPositions(const std::vector<double> &q) const
{
    return environment_->GetJointPositions(q);
}

double EnvironmentFeedback::getDmin(const std::vector<double>& coordinates) const
{
    /*std::vector<std::pair<double, std::vector<double> > > distance_profile(
                environment_->GetDistanceProfile(coordinates));

    double distance = std::numeric_limits<double>::infinity();
    for (const auto& profile_part : distance_profile)
        distance = std::min(distance, profile_part.first);

    return distance;*/
    return environment_->GetDmin(coordinates);
}

double EnvironmentFeedback::getDmin(const std::vector<double> &coordinates, std::vector<std::pair<fcl::Vec3f, fcl::Vec3f> > &points) const
{
    return environment_->GetDmin(coordinates, points);
}

std::vector<double> EnvironmentFeedback::getRs(
        const std::vector<double>& coordinates) const
{
    std::vector<std::pair<double, std::vector<double> > > distance_profile(
                environment_->GetDistanceProfile(coordinates));

    std::vector<double> rs(distance_profile.size());

    for (unsigned int i = 0; i < distance_profile.size(); i++)
    {
        double maxV = -1 * std::numeric_limits<double>::infinity();
        for (unsigned int j = 0; j < distance_profile.size(); j++)
        {
            maxV = std::max( maxV, distance_profile[j].second[i] );

        }
        rs[i] = maxV;
    }
    return rs;

}


Bur *EnvironmentFeedback::NewBur(const std::vector<double> &coordinates,
                                 const std::vector<double> &q_target, int burSize) const
{
    std::vector<std::vector<double> > points(burSize);
    for (int i = 0; i < burSize - 1; i++)
    {
        std::vector<double> random_point(generatePoint(coordinates));
        points[i] = random_point;
    }

    points[burSize-1] = q_target;

    double _norm = norm(q_target, coordinates);
    for (unsigned int i = 0; i < coordinates.size(); i++) {
        points[burSize-1][i] =  coordinates[i]*deg_to_rad() +
                PI * (points[burSize-1][i]*deg_to_rad() - coordinates[i]*deg_to_rad() ) /
                _norm;
        points[burSize-1][i] *= rad_to_deg();
    }

    std::vector<std::pair<double, std::vector<double> > > distance_profile(
                environment_->GetDistanceProfile(coordinates));

    std::vector<double> rs(distance_profile.size());
    double dmin = std::numeric_limits<double>::infinity();

    for (const auto& profile_part : distance_profile)
            dmin = std::min(dmin, profile_part.first);

    for (unsigned int i = 0; i < distance_profile.size(); i++)
    {
        double maxV = -1 * std::numeric_limits<double>::infinity();
        dmin = std::min(dmin, distance_profile[i].first);
        for (unsigned int j = 0; j < distance_profile.size(); j++)
        {
            maxV = std::max( maxV, distance_profile[j].second[i] );
        }
        rs[i] = maxV;
    }
    dmin /= 1000.0;
    for (int i = 0; i < burSize; i++)
    {
        // Classic Bur
        //LOG(INFO) << "extending bur point " << i;
        double iter = optimizeBurNeedle(coordinates, points[i], dmin, rs);
    }
    return new Bur(coordinates, points, dmin);
}

Bur *EnvironmentFeedback::NewGenBur(const std::vector<double> &coordinates, const std::vector<double> &q_target, double dmin) const
{
    std::vector<std::vector<double> > points(1);
    points[0] = q_target;

    double _norm = norm(q_target, coordinates);
    for (unsigned int i = 0; i < coordinates.size(); i++) {
        points[0][i] =  coordinates[i]*deg_to_rad() +
                PI * (points[0][i]*deg_to_rad() - coordinates[i]*deg_to_rad() ) /
                _norm;
        points[0][i] *= rad_to_deg();
    }

    std::vector<std::pair<double, std::vector<double> > > distance_profile(
                environment_->GetDistanceProfile(coordinates));

    std::vector<double> rs(distance_profile.size());

    for (unsigned int i = 0; i < distance_profile.size(); i++)
    {
        double maxV = -1 * std::numeric_limits<double>::infinity();
        dmin = std::min(dmin, distance_profile[i].first);
        for (unsigned int j = 0; j < distance_profile.size(); j++)
        {
            maxV = std::max( maxV, distance_profile[j].second[i] );
        }
        rs[i] = maxV;
    }
    dmin /= 1000.0;
    double iter = optimizeBurNeedle(coordinates, points[0], dmin, rs);

    return new Bur(coordinates, points, dmin);

}

EnvironmentFeedback::EnvironmentFeedback(EnvironmentInterface* environment)
    : environment_(environment) {}

Bubble* EnvironmentFeedback::NewBubble(
        const std::vector<double>& coordinates, bool extended) const {

    std::vector<std::pair<double, std::vector<double> > > distance_profile(
                environment_->GetDistanceProfile(coordinates));

    std::vector<double> output(
                coordinates.size(), std::numeric_limits<double>::infinity());

    double distance = std::numeric_limits<double>::infinity();
    if (!extended) {
        for (const auto& profile_part : distance_profile)
            distance = std::min(distance, profile_part.first);
    }

    for (const auto& profile_part : distance_profile) {
        if (extended)
            distance = profile_part.first;
        int segment = 0;
        for (double subdistance : profile_part.second) {
            if (subdistance != 0.0)
                output[segment] = std::min(
                            output[segment], rad_to_deg() * distance / subdistance);
            ++segment;
        }
    }

    return new Bubble(coordinates, output);
}

bool EnvironmentFeedback::IsCollision(
        const std::vector<double>& coordinates) const {
    return environment_->IsCollision(coordinates);
}

bool EnvironmentFeedback::IsCollisionAugmented(
        const std::vector<double>& coordinates) const {
    return environment_->IsCollisionAugmented(coordinates);
}

std::vector<std::pair<double, double> >
EnvironmentFeedback::GetAngleRanges() const {
    return environment_->GetAngleRanges();
}

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
