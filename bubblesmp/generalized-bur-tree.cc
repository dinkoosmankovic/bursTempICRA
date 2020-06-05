//
// Copyright (c) 2017, Dinko Osmankovic
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

#include <cmath>
#include <cstdio>
#include <memory>

#include <glog/logging.h>
#include <cstdio>

#include "bubblesmp/environment/environment-feedback.h"
#include "bubblesmp/tree-node.h"
#include "bubblesmp/bur.h"
#include "bubblesmp/generalized-bur-tree.h"

//#define CSPACE

int num_extension = 0;
int extension_per_iteration = 0;

namespace com {
namespace ademovic {
namespace bubblesmp {

double gnorm(const std::vector<double> &p1, const std::vector<double> &p2)
{
    double sum = 0;
    for (unsigned int i = 0; i < p1.size(); i++)
        sum += ( p1[i] - p2[i] )
                * ( p1[i] - p2[i] );
    return sqrt(sum);
}

constexpr double rad_to_deg() {
    return 45.0 / atan(1);
}

constexpr double deg_to_rad() {
    return atan(1) / 45.0;
}

GenBurTree::~GenBurTree() {
    if (num_extension != 0)
        printf("average extensions: %.4f\n", (double)extension_per_iteration / num_extension);
    num_extension = 0;
    extension_per_iteration = 0;
}

GenBurTree::GenBurTree(double max_step, unsigned substeps, const std::vector<double>& root,
                       std::shared_ptr<environment::EnvironmentFeedback> bur_source,
                       const IndexSettings& index_settings, int burSize, double threshold, int numberOfExtensions)
    : RrtTree(root, index_settings), eps_(max_step / substeps),
      substeps_(substeps), bur_source_(bur_source), change_counter(0) {
    CHECK(!bur_source_->IsCollision(root)) << "Collision at root point";
    eps_ = 180 / 10.0;
    threshold_ = threshold; threshold_ = 70;
    burSize_ = burSize;
    numberOfExtensions_ = numberOfExtensions;
#ifdef CSPACE
    FILE* f = fopen("CSpace.txt","w");
    for (double i = -180; i < 180; i+=0.5) {
        for (double j = -180; j < 180; j+=0.5) {
            if (bur_source_->IsCollision({i, j}))
                fprintf(f, "%.2f %.2f\n", i, j);
        }
    }
    fclose(f);
#endif
}

bool GenBurTree::Connect(TreeNode* node, const std::vector<double>& q_target) {

    std::vector<double> qnew;

    double ds = std::numeric_limits<double>::infinity();
    int steps = 0;

    TreeNode* current = node;
    ExtensionResult result;

    do {
        std::unique_ptr<Bur> _bur = nullptr;
        _bur = std::unique_ptr<Bur>(
                    bur_source_->NewBur(current->point->position(), q_target, burSize_));
        Bur* bur_ptr = _bur.get();
        if (bur_ptr->getDmin() * 1000.0 < threshold_) {
            do {
                result = ExtendFromNodeClassic(current->point->position(), current, q_target);
                current = GetNewestNode();
            } while (result == ExtensionResult::ADVANCED);

            return result == ExtensionResult::REACHED;
        }
        qnew = bur_ptr->burNodes()[0];
        ds = gnorm(current->point->position(), qnew);
        if (ds < eps_)
            break;

        AddNode(qnew, current);
        current = GetNewestNode();

        std::vector<std::pair<fcl::Vec3f, fcl::Vec3f> > points;
        bur_source_->getDmin(current->point->position(), points);

        TreeNode* burEndpointNode = GetNewestNode();
        // Generalized Bur procedure
        std::vector<double> extendedQ;
        std::vector<double> qt = qnew;

        num_extension++;
        size_t i = 0;
        //numberOfExtensions_ = 1;
        for (i = 0; i < numberOfExtensions_; ++i) {
            std::vector<fcl::Vec3f> jointPositions = bur_source_->GetJointPositions(qt);
            extendedQ = extendToGenBur(burEndpointNode->point->position(), qt, points, jointPositions);
            bool test = false;
            for (size_t j = 0; j < extendedQ.size(); ++j) {
                if (extendedQ[j] < -180) extendedQ[j] = -179.5;
                if (extendedQ[j] > 180) extendedQ[j] = 179.5;

                if (gnorm(extendedQ, qt) < eps_) {
                    test = true;
                    break;
                }
            }
            if (test) {
                break;
            }
            AddNode(extendedQ, burEndpointNode);
            burEndpointNode = GetNewestNode();
            qt = extendedQ;
        }
        extension_per_iteration += i;
        current = node;

        if ( gnorm( q_target, qnew ) < eps_/100.0 )
        {
            return true;
        }
        steps++;

    } while (ds > eps_ && steps < 8 );
    return false;
}

TreeNode* GenBurTree::AddNode(
        const std::vector<double>& q, TreeNode* parent) {
    TreeNode* current_node = new TreeNode(new TreePoint(q), parent);
    nodes_.emplace_back(current_node);
    point_index_->AddPoint(q, current_node);
    return current_node;
}

ExtensionResult GenBurTree::ExtendFrom(
        const AttachmentPoint& point, const std::vector<double>& q_target) {
    return ExtendFromNode(point.position, point.parent, q_target);
}

std::vector<double> GenBurTree::extendToGenBur(const std::vector<double> &q,
                                               const std::vector<double> &qBur,
                                               std::vector<std::pair<fcl::Vec3f, fcl::Vec3f> > &points,
                                               std::vector<fcl::Vec3f> &jointPositions) const
{
    int number_of_obstacles = points.size() / q.size();
    double mind = std::numeric_limits<float>::max();
    std::vector<double> Ds;
    //LOG(INFO) << q.size() << "|" << points.size() << "|" << number_of_obstacles;
    //std::vector<fcl::Vec3f> jointPositions = bur_source_->GetJointPositions(qBur);
    for (size_t j = 0; j < number_of_obstacles; ++j) {
        for (size_t i = j; i < points.size(); i+=number_of_obstacles) {
            fcl::Vec3f n = points[i].first - points[i].second;
            double A = n.data[0], B = n.data[1], C = n.data[2];
            double D = -A*points[i].second.data[0] -
                    -B*points[i].second.data[1] -
                    -C*points[i].second.data[2];
            double m = 1 / std::sqrt(A*A + B*B + C*C);
            int v = (int) i/number_of_obstacles;
            std::vector<fcl::Vec3f> obb_points = bur_source_->getOBBVertices(v);
            /*double xP1 = jointPositions[v].data[0], yP1 = jointPositions[v].data[1], zP1 = jointPositions[v].data[2];
            double xQ1 = jointPositions[v+1].data[0], yQ1 = jointPositions[v+1].data[1], zQ1 = jointPositions[v+1].data[2];
            double dij = std::min( std::abs(A*xP1 + B*yP1 + C*zP1 + D)*m ,
                                   std::abs(A*xQ1 + B*yQ1 + C*zQ1 + D)*m);*/

            double dij = 1e9;

            for (fcl::Vec3f p : obb_points) {
                dij  = std::min(dij, std::abs(A*p[0] + B*p[1] + C*p[2] + D)*m);
            }
            //LOG(INFO) << "dij: " << dij;

            Ds.emplace_back(dij);
        }
        //double mind_i = *std::min_element(Ds.begin(), Ds.end());
        //mind = std::min(mind, mind_i);
    }
    mind = *std::min_element(Ds.begin(), Ds.end());
    std::vector<double> qe(q.size());
    double _norm = gnorm(q, qBur);
    const double PI = atan(1) * 4;
    for (unsigned int i = 0; i < q.size(); i++) {
        qe[i] =  qBur[i]*deg_to_rad() +
                PI * (qBur[i]*deg_to_rad() - q[i]*deg_to_rad() ) /
                _norm;
        qe[i] *= rad_to_deg();
    }
    std::unique_ptr<Bur> gBur(
                bur_source_->NewGenBur(qBur, qe, mind));
    return gBur.get()->burNodes()[0];

}

ExtensionResult GenBurTree::ExtendFromNode(
        const std::vector<double>& q, TreeNode* node,
        const std::vector<double>& q_target) {


    std::unique_ptr<Bur> bur(
                bur_source_->NewBur(q, q_target, burSize_));

    Bur* bur_ptr = bur.get();
    if ( bur_ptr->getDmin() * 1000 < threshold_ ) {
        return ExtendFromNodeClassic(q, node, q_target);
    }

    TreeNode* current = node;

    int num = 0;
    std::vector<std::pair<fcl::Vec3f, fcl::Vec3f> > points;
    bur_source_->getDmin(q, points);

    for (auto qt: bur_ptr->burNodes() )
    {
        if (gnorm(qt, q) < eps_/100.0) /*&& num != burSize_)*/
            continue;
        else {
            num++;
            //point_index_->AddPoint(qt, current_node);
            AddNode(qt, current);
            TreeNode* burEndpointNode = GetNewestNode();
            // Generalized Bur procedure
            std::vector<double> extendedQ;

            num_extension++;
            size_t i = 0;
            //numberOfExtensions_ = 1;
            for (i = 0; i < numberOfExtensions_; ++i) {
                std::vector<fcl::Vec3f> jointPositions = bur_source_->GetJointPositions(qt);
                extendedQ = extendToGenBur(q, qt, points, jointPositions);
                bool test = false;
                for (size_t j = 0; j < extendedQ.size(); ++j) {
                    if (extendedQ[j] < -180) extendedQ[j] = -179.5;
                    if (extendedQ[j] > 180) extendedQ[j] = 179.5;

                    if (gnorm(extendedQ, qt) < eps_) {
                        test = true;
                        break;
                    }
                }
                if (test) {
                    break;
                }
                AddNode(extendedQ, burEndpointNode);
                burEndpointNode = GetNewestNode();
                qt = extendedQ;
            }
            extension_per_iteration += i;
            current = node;
            //current = GetNewestNode();
            if (gnorm(qt, q_target) < eps_ )
                return ExtensionResult::REACHED;
        }

    }

    if ( num < burSize_ )
        return ExtensionResult::TRAPPED;

    return ExtensionResult::ADVANCED;

}

ExtensionResult GenBurTree::ExtendFromNodeClassic(
        const std::vector<double>& q, TreeNode* node,
        const std::vector<double>& q_target) {
    std::vector<double> current(q);
    std::vector<double> step(q_target);
    size_t axis_count = q_target.size();

    double length = 0.0;
    for (size_t i = 0; i < axis_count; ++i) {
        step[i] -= current[i];
        length += fabs(step[i]);
    }

    bool not_reached = length > eps_ * substeps_;
    if (not_reached)
        length = eps_ / length;
    else
        length = 1.0 / substeps_;

    for (size_t i = 0; i < axis_count; ++i)
        step[i] *= length;

    for (int s = 0; s < substeps_; ++s) {
        for (size_t i = 0; i < axis_count; ++i)
            current[i] += step[i];
        if (bur_source_->IsCollision(current))
            return ExtensionResult::TRAPPED;
    }
    AddNode(not_reached ? current : q_target, node);
    return not_reached
            ? ExtensionResult::ADVANCED
            : ExtensionResult::REACHED;
}

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
