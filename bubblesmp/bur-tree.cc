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

#include "bubblesmp/bur-tree.h"

#include <cmath>
#include <cstdio>
#include <memory>

#include <glog/logging.h>

#include "bubblesmp/environment/environment-feedback.h"
#include "bubblesmp/tree-node.h"
#include "bubblesmp/bur.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

double norm(const std::vector<double> &p1, const std::vector<double> &p2)
{
    double sum = 0;
    for (unsigned int i = 0; i < p1.size(); i++)
        sum += ( p1[i] - p2[i] )
                * ( p1[i] - p2[i] );
    return sqrt(sum);
}

BurTree::~BurTree() {}

BurTree::BurTree(
        double max_step, unsigned substeps, const std::vector<double>& root,
        std::shared_ptr<environment::EnvironmentFeedback> bur_source,
        const IndexSettings& index_settings, int burSize, double threshold)
    : RrtTree(root, index_settings), eps_(max_step / substeps),
      substeps_(substeps), bur_source_(bur_source), change_counter(0) {
    CHECK(!bur_source_->IsCollision(root)) << "Collision at root point";
    eps_ = 180 / 20.0;
    threshold_ = threshold; threshold_ = 70;
    burSize_ = burSize;
}

bool BurTree::Connect(TreeNode* node, const std::vector<double>& q_target) {
    //LOG(INFO) << "CONNECT";
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
        //LOG(INFO) << "dmin: " << bur_ptr->getDmin()*1000;
        if (bur_ptr->getDmin() * 1000.0 < threshold_) {
            do {
                result = ExtendFromNodeClassic(current->point->position(), current, q_target);
                current = GetNewestNode();
            } while (result == ExtensionResult::ADVANCED);

            return result == ExtensionResult::REACHED;
        }
        qnew = bur_ptr->burNodes()[0];
        ds = norm(current->point->position(), qnew);
        if (ds < eps_)
            break;

        AddNode(qnew, current);
        current = GetNewestNode();


        if ( norm( q_target, qnew ) < eps_/1000.0 ) {
            return true;
        }
        steps++;

    } while (ds > eps_ && steps < 5 );
    return false;
}

TreeNode* BurTree::AddNode(
        const std::vector<double>& q, TreeNode* parent) {
    TreeNode* current_node = new TreeNode(new TreePoint(q), parent);
    nodes_.emplace_back(current_node);
    point_index_->AddPoint(q, current_node);
    /*LOG(INFO) << q[0] << " " << q[1] << " "
                         << parent->point->position()[0] << " "
                         << parent->point->position()[1];*/
    return current_node;
}

ExtensionResult BurTree::ExtendFrom(
        const AttachmentPoint& point, const std::vector<double>& q_target) {

    return ExtendFromNode(point.position, point.parent, q_target);
}

ExtensionResult BurTree::ExtendFromNode(
        const std::vector<double>& q, TreeNode* node,
        const std::vector<double>& q_target) {

    std::unique_ptr<Bur> bur(
                bur_source_->NewBur(q, q_target, burSize_));

    Bur* bur_ptr = bur.get();
    //LOG(INFO) << bur_ptr->getDmin()*1000 << "|" << threshold_;
    if ( bur_ptr->getDmin() * 1000 < threshold_ ) {
        return ExtendFromNodeClassic(q, node, q_target);
    }

    //LOG(INFO) << "EXTEND BUR";

    TreeNode* current = node;

    int num = 0;

    for (auto qt: bur_ptr->burNodes() )
    {
        if (norm(qt, q) < eps_ && num != burSize_ )
            continue;
        else {
            num++;
            //point_index_->AddPoint(qt, current_node);
            AddNode(qt, current);
            //current = GetNewestNode();
            if (norm(qt, q_target) < eps_ )
                return ExtensionResult::REACHED;
        }

    }

    if ( num < burSize_ )
        return ExtensionResult::TRAPPED;

    return ExtensionResult::ADVANCED;

}

ExtensionResult BurTree::ExtendFromNodeClassic(
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
