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

#ifndef COM_ADEMOVIC_BUBBLESMP_GENERALIZED_BUR_TREE_H_
#define COM_ADEMOVIC_BUBBLESMP_GENERALIZED_BUR_TREE_H_

#include <memory>
#include <vector>

#include "bubblesmp/rrt-tree.h"
#include "bubblesmp/bur.h"

namespace com {
namespace ademovic {
namespace bubblesmp {

namespace environment {
class EnvironmentFeedback;
}  // namespace environment

class GenBurTree : public RrtTree {
public:
    GenBurTree(
            double max_step, unsigned substeps, const std::vector<double>& root,
            std::shared_ptr<environment::EnvironmentFeedback> collision_source,
            const IndexSettings& index_settings, int burSize, double threshold, int numberOfExtensions);
    virtual ~GenBurTree();

    virtual bool Connect(TreeNode* node, const std::vector<double>& q_target);

    std::vector<double> extendToGenBur(const std::vector<double> &q, const std::vector<double> &qBur, std::vector<std::pair<fcl::Vec3f, fcl::Vec3f> > &points, std::vector<fcl::Vec3f> &jointPositions) const;
private:
    // Does not take ownership of parent.
    // Has ownership of returned pointer.
    virtual TreeNode* AddNode(const std::vector<double>& q, TreeNode* parent);
    virtual ExtensionResult ExtendFrom(
            const AttachmentPoint& point, const std::vector<double>& q_target);
    ExtensionResult ExtendFromNode(const std::vector<double>& q, TreeNode* node,
                                   const std::vector<double>& q_target);
    ExtensionResult ExtendFromNodeClassic(const std::vector<double> &q, TreeNode *node,
                                          const std::vector<double> &q_target);

    double eps_;
    int substeps_;
    std::shared_ptr<environment::EnvironmentFeedback> bur_source_;
    int change_counter;
    double threshold_; //std::numeric_limits<double>::infinity();
    int burSize_;
    int numberOfExtensions_;
};

}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com

#endif  // COM_ADEMOVIC_BUBBLESMP_GENERALIZED_BUR_TREE_H_
