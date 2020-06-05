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

#include "bubblesmp/classic-tree.h"

#include <cmath>
#include <cstdio>

#include <glog/logging.h>

#include "bubblesmp/environment/environment-feedback.h"
#include "bubblesmp/tree-node.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

ClassicTree::~ClassicTree() {}

ClassicTree::ClassicTree(
        double max_step, unsigned substeps, const std::vector<double>& root,
        std::shared_ptr<environment::EnvironmentFeedback> collision_source,
        const IndexSettings& index_settings)
    : RrtTree(root, index_settings), eps_(max_step / substeps),
      substeps_(substeps), collision_source_(collision_source) {
    CHECK(!collision_source_->IsCollision(root)) << "Collision at root point";
}

bool ClassicTree::Connect(TreeNode* node, const std::vector<double>& q_target) {
    TreeNode* current = node;
    ExtensionResult result;

    do {
        result = ExtendFromNode(current->point->position(), current, q_target);
        current = GetNewestNode();
    } while (result == ExtensionResult::ADVANCED);

    return result == ExtensionResult::REACHED;
}

TreeNode* ClassicTree::AddNode(
        const std::vector<double>& q, TreeNode* parent) {
    TreeNode* current_node = new TreeNode(new TreePoint(q), parent);
    nodes_.emplace_back(current_node);
    point_index_->AddPoint(q, current_node);
    /*LOG(INFO) << q[0] << " " << q[1] << " "
                         << parent->point->position()[0] << " "
                         << parent->point->position()[1];*/
    return current_node;
}

ExtensionResult ClassicTree::ExtendFrom(
        const AttachmentPoint& point, const std::vector<double>& q_target) {
    return ExtendFromNode(point.position, point.parent, q_target);
}

ExtensionResult ClassicTree::ExtendFromNode(
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
        if (collision_source_->IsCollision(current))
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
