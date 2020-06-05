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

#include "bubblesmp/environment/fcl-environment.h"

#include <algorithm>
#include <cmath>
#include <fstream>

#include <fcl/BVH/BVH_model.h>
#include <fcl/distance.h>
#include <fcl/broadphase/broadphase.h>

#include <glog/logging.h>
#include <google/protobuf/text_format.h>

#include "bubblesmp/environment/environment.pb.h"
#include "test_fcl_utility.h"
#include "obj_loader.h"


namespace com {
namespace ademovic {
namespace bubblesmp {
namespace environment {
namespace {

constexpr double deg_to_rad() {
    return atan(1) / 45.0;
}

int num_collision = 0;
int num_distance = 0;

double PointDistanceToAxis(const fcl::Vec3f& point, const fcl::Vec3f& axis) {
    double l = point.dot(axis);
    return (point - axis * l).length();
}

double PointDistanceToVector(const fcl::Vec3f& point, const fcl::Vec3f& vect) {
    double point_a_distance(point.sqrLength());
    fcl::Vec3f d = point - vect;
    double point_b_distance(d.sqrLength());
    fcl::Vec3f u = -vect;
    u.normalize();
    if (point_a_distance < point_b_distance) {
        d = point;
        d.normalize();
        double c = d.dot(fcl::Vec3f(vect).normalize());
        return (c > 0.0)
                ? sqrt(point_a_distance * (1.0 - c * c))
                : sqrt(point_a_distance);
    } else {
        d.normalize();
        double c = d.dot((-vect).normalize());
        return (c > 0.0)
                ? sqrt(point_b_distance * (1.0 - c * c))
                : sqrt(point_b_distance);
    }
}

// TODO: Make parsing more sophisticated and do other parsing besides bin STL.
// TODO: Fix parsing for big endian machines.
fcl::BVHModel<fcl::OBBRSS>* ParseModel(
        const std::string& filename, fcl::Transform3f* t,
        double x, double y, double z, double* l, double* r) {
    FILE* f = fopen(filename.c_str(), "rb");
    CHECK(f != nullptr) << "Could not open model file: " << filename;
    uint8_t header[80];
    uint32_t triangle_count;
    size_t fread_retval;
    fread_retval = fread(header, sizeof(header[0]), 80, f);
    CHECK_EQ(fread_retval, 80) << "Error reading file: " << filename;
    fread_retval = fread(&triangle_count, sizeof(triangle_count), 1, f);
    CHECK_EQ(fread_retval, 1) << "Error reading file: " << filename;
    float input[3][3];
    fcl::Vec3f p[3];
    float normal[3];
    uint16_t attribute;
    fcl::BVHModel<fcl::OBBRSS>* model = new fcl::BVHModel<fcl::OBBRSS>;
    model->beginModel();
    *l = 0.0;
    *r = 0.0;
    for (uint32_t triangle = 0; triangle < triangle_count; ++triangle) {
        fread_retval = fread(normal, sizeof(normal[0]), 3, f);
        CHECK_EQ(fread_retval, 3) << "Error reading file: " << filename;
        fread_retval = fread(input[0], sizeof(input[0][0]), 3, f);
        CHECK_EQ(fread_retval, 3) << "Error reading file: " << filename;
        fread_retval = fread(input[1], sizeof(input[1][0]), 3, f);
        CHECK_EQ(fread_retval, 3) << "Error reading file: " << filename;
        fread_retval = fread(input[2], sizeof(input[2][0]), 3, f);
        CHECK_EQ(fread_retval, 3) << "Error reading file: " << filename;
        fread_retval = fread(&attribute, sizeof(attribute), 1, f);
        CHECK_EQ(fread_retval, 1) << "Error reading file: " << filename;
        for (int tri = 0; tri < 3; ++tri) {
            p[tri] = t->transform(fcl::Vec3f(
                                      input[tri][0], input[tri][1], input[tri][2]));
            *l = std::max(*l, p[tri].dot({x, y, z}));
            *r = std::max(*r, PointDistanceToVector(
                              p[tri], {x * (*l), y * (*l), z * (*l)}));
        }
        model->addTriangle(p[0], p[1], p[2]);
    }
    model->endModel();
    fclose(f);
    return model;
}

fcl::BVHModel<fcl::OBBRSS>* ParseModelFromObjMeshPart(objl::Loader& Loader, int curr) {
    fcl::BVHModel<fcl::OBBRSS>* model = new fcl::BVHModel<fcl::OBBRSS>;
    objl::Mesh curMesh = Loader.LoadedMeshes[curr];
    model->beginModel();

    for (int i = 0; i < curMesh.Indices.size(); i+=3) {
        fcl::Vec3f p[3];
        p[0] = fcl::Vec3f(curMesh.Vertices[i].Position.X, curMesh.Vertices[i].Position.Z, curMesh.Vertices[i].Position.Y);
        p[1] = fcl::Vec3f(curMesh.Vertices[i+1].Position.X, curMesh.Vertices[i+1].Position.Z, curMesh.Vertices[i+1].Position.Y);
        p[2] = fcl::Vec3f(curMesh.Vertices[i+2].Position.X, curMesh.Vertices[i+2].Position.Z, curMesh.Vertices[i+2].Position.Y);
        //LOG(INFO) << p[0] << p[1] << p[2];
        model->addTriangle(p[0], p[1], p[2]);
    }
    model->endModel();
    return model;
}

fcl::BVHModel<fcl::OBBRSS> *ComputeAutmentedModel(fcl::BVHModel<fcl::OBBRSS>* model,
                                                  double scale) {

    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (unsigned int i = 0; i < model->num_vertices; ++i ) {
        sx += model->vertices[i][0];
        sy += model->vertices[i][1];
        sz += model->vertices[i][2];
    }
    sx /= (double)model->num_vertices;
    sy /= (double)model->num_vertices;
    sz /= (double)model->num_vertices;
    model->beginReplaceModel();
    for (unsigned int i = 0; i < model->num_vertices; ++i ) {
        double dx = model->vertices[i][0] - sx;
        double dy = model->vertices[i][1] - sy;
        double dz = model->vertices[i][2] - sz;

        double norm = sqrt(dx * dx + dy * dy + dz * dz);
        double padding = 0.0;

        double amx = 0, amy = 0, amz = 0;
        if (norm > 1e-6)
        {
            double fact = scale + padding/norm;
            amx = sx + dx * fact;
            amy = sy + dy * fact;
            amz = sz + dz * fact;
        }
        else
        {
            double ndx = ((dx > 0) ? dx+padding : dx-padding);
            double ndy = ((dy > 0) ? dy+padding : dy-padding);
            double ndz = ((dz > 0) ? dz+padding : dz-padding);
            amx = sx + ndx;
            amy = sy + ndy;
            amz = sz + ndz;
        }
        model->replaceVertex(fcl::Vec3f(model->vertices[i][0], amy, amz));
    }
    model->endReplaceModel(true, true);
    return model;
}

}  // namespace

FclEnvironment::~FclEnvironment() {
    if (num_collision != 0 || num_distance != 0)
        printf("C/D: %d %d \n", num_collision, num_distance);
    num_collision = 0;
    num_distance = 0;
}

FclEnvironment::FclEnvironment(const std::string& configuration) {
    boost::filesystem::path config_file_path(configuration);
    config_file_path.remove_filename();
    std::ifstream fin(configuration);
    EnvironmentConfig config_pb;
    std::string input_string((std::istreambuf_iterator<char>(fin)),
                             std::istreambuf_iterator<char>());
    bool success = google::protobuf::TextFormat::ParseFromString(
                input_string, &config_pb);
    fin.close();
    CHECK(success) << "Failed parsing f, m1, pose1, m2, pose2, DistanceRequest(true), local_result))ile: " << configuration;
    ConfigureFromPB(config_pb, config_file_path);
}

FclEnvironment::FclEnvironment(
        const EnvironmentConfig& configuration,
        const boost::filesystem::path& config_file_path) {
    ConfigureFromPB(configuration, config_file_path);
}

bool FclEnvironment::IsCollision(const std::vector<double>& q) const {
    std::lock_guard<std::mutex> guard(guard_mutex_);
    fcl::Transform3f pose;
    fcl::Matrix3f joint_angle_rotation;
    int segment = -1;
    num_collision++;
    //LOG(INFO) << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << "," << q[4] << "," << q[5];
    for (int part = 0; part < part_count_; ++part) {
        if (is_joint_start_[part]) {
            if (segment > -1)
                pose *= *dh_parameters_[segment];
            ++segment;
            joint_angle_rotation.setEulerZYX(0.0, 0.0, q[segment] * deg_to_rad());
            pose *= fcl::Transform3f(joint_angle_rotation);
        }
        parts_[part]->setTransform(pose);
        //LOG(INFO) << part << "|" << parts_[part]->getTranslation();
        for (int env_part = 0; env_part < environment_.size(); ++env_part) {
            fcl::CollisionRequest request;
            fcl::CollisionResult result;
            fcl::collide(parts_[part].get(), environment_[env_part].get(), request, result);
            if (result.isCollision()) {
                return true;
            }
        }
    }
    return false;
}

bool FclEnvironment::IsCollisionAugmented(const std::vector<double>& q) const {
    std::lock_guard<std::mutex> guard(guard_mutex_);
    fcl::Transform3f pose;
    fcl::Matrix3f joint_angle_rotation;
    int segment = -1;
    num_collision++;
    for (int part = 0; part < part_count_; ++part) {
        if (is_joint_start_[part]) {
            if (segment > -1)
                pose *= *dh_parameters_[segment];
            ++segment;
            joint_angle_rotation.setEulerZYX(0.0, 0.0, q[segment] * deg_to_rad());
            pose *= fcl::Transform3f(joint_angle_rotation);
        }
        aug_parts_[part]->setTransform(pose);
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        for (int env_part = 0; env_part < environment_.size(); ++env_part) {
            fcl::collide(parts_[part].get(), environment_[env_part].get(), request, result);
            if (result.isCollision()) {
                return true;
            }
        }
    }
    return false;
}

std::vector<fcl::Vec3f> FclEnvironment::GetJointPositions(
        const std::vector<double>& q) const {

    std::lock_guard<std::mutex> guard(guard_mutex_);
    fcl::Transform3f pose;
    fcl::Matrix3f joint_angle_rotation;
    std::vector<fcl::Vec3f> ax_P; // stores all joint positions
    std::vector<fcl::Vec3f> ax_O; // stores all axis orientations
    int segment = -1;

    fcl::Matrix3f R;
    for (int part = 0; part < part_count_ + 1; ++part) {
        if (segment > -1)
            pose *= *dh_parameters_[segment];
        ++segment;
        ax_P.push_back(pose.getTranslation());
        R = pose.getRotation();
        ax_O.push_back({R(0, 2), R(1, 2), R(2, 2)});
        joint_angle_rotation.setEulerZYX(0.0, 0.0, q[segment] * deg_to_rad());
        pose *= fcl::Transform3f(joint_angle_rotation);
    }
    return ax_P;
}

double FclEnvironment::GetDmin(const std::vector<double> &q) const
{
    std::vector<std::pair<double, std::vector<double> > > distance_profile(
                GetDistanceProfile(q));

    double distance = std::numeric_limits<double>::infinity();
    for (const auto& profile_part : distance_profile)
        distance = std::min(distance, profile_part.first);

    return distance;
}

double FclEnvironment::GetDmin(const std::vector<double>& q, std::vector<std::pair<fcl::Vec3f,
                               fcl::Vec3f> > &points) const {

    //std::vector<std::pair<fcl::Vec3f,fcl::Vec3f> > result;
    std::vector<std::pair<double, std::vector<double> > > distance_profile(
                GetDistanceProfile(q,points));

    //points = result;
    double distance = std::numeric_limits<double>::infinity();
    for (const auto& profile_part : distance_profile)
        distance = std::min(distance, profile_part.first);

    return distance;
}

EnvironmentInterface::DistanceProfile FclEnvironment::GetDistanceProfile(
        const std::vector<double>& q) const {
    std::lock_guard<std::mutex> guard(guard_mutex_);
    fcl::Transform3f pose;
    fcl::Matrix3f joint_angle_rotation;
    std::vector<fcl::Vec3f> ax_P; // stores all joint positions
    std::vector<fcl::Vec3f> ax_O; // stores all axis orientations
    EnvironmentInterface::DistanceProfile distances;
    int segment = -1;

    num_distance++;

    fcl::Vec3f p, p_prev;
    fcl::Matrix3f R;
    for (int part = 0; part < part_count_; ++part) {
        if (is_joint_start_[part]) {
            if (segment > -1)
                pose *= *dh_parameters_[segment];
            ++segment;
            ax_P.push_back(pose.getTranslation());
            R = pose.getRotation();
            ax_O.push_back({R(0, 2), R(1, 2), R(2, 2)});
            joint_angle_rotation.setEulerZYX(0.0, 0.0, q[segment] * deg_to_rad());
            pose *= fcl::Transform3f(joint_angle_rotation);
        }
        parts_[part]->setTransform(pose);
        for (int env_part = 0; env_part < environment_.size(); ++env_part) {
            fcl::DistanceRequest request(false, 1.0, variance_);
            fcl::DistanceResult result;
            fcl::distance(parts_[part].get(), environment_[env_part].get(), request, result);

            distances.emplace_back(
                        std::max(result.min_distance, 0.0), std::vector<double>(0));

            std::vector<double>* radiuses = &(distances.back().second);
            p = pose.transform(cylinders_[part].first);
            for (int seg = 0; seg <= segment; ++seg) {
                radiuses->push_back(cylinders_[part].second + std::max(
                                        PointDistanceToAxis(p - ax_P[seg], ax_O[seg]),
                                        PointDistanceToAxis(p_prev - ax_P[seg], ax_O[seg])));
            }
            p_prev = p;
        }
    }

    return distances;
}

EnvironmentInterface::DistanceProfile FclEnvironment::GetDistanceProfile(const std::vector<double> &q,
                                                                         std::vector<std::pair<fcl::Vec3f,
                                                                         fcl::Vec3f> > &points) const
{
    std::lock_guard<std::mutex> guard(guard_mutex_);
    fcl::Transform3f pose;
    fcl::Matrix3f joint_angle_rotation;
    std::vector<fcl::Vec3f> ax_P; // stores all joint positions
    std::vector<fcl::Vec3f> ax_O; // stores all axis orientations
    EnvironmentInterface::DistanceProfile distances;
    int segment = -1;

    num_distance++;

    fcl::Vec3f p, p_prev;
    fcl::Matrix3f R;
    for (int part = 0; part < part_count_; ++part) {
        if (is_joint_start_[part]) {
            if (segment > -1)
                pose *= *dh_parameters_[segment];
            ++segment;
            ax_P.push_back(pose.getTranslation());
            R = pose.getRotation();
            ax_O.push_back({R(0, 2), R(1, 2), R(2, 2)});
            joint_angle_rotation.setEulerZYX(0.0, 0.0, q[segment] * deg_to_rad());
            pose *= fcl::Transform3f(joint_angle_rotation);
        }
        parts_[part]->setTransform(pose);
        for (int env_part = 0; env_part < environment_.size(); ++env_part) {
            fcl::DistanceRequest request(true, 1.0, variance_);
            fcl::DistanceResult result;
            fcl::distance(parts_[part].get(), environment_[env_part].get(), request, result);
            distances.emplace_back(
                        std::max(result.min_distance, 0.0), std::vector<double>(0));

            points.emplace_back(
                        std::make_pair(result.nearest_points[0], result.nearest_points[1]));

            std::vector<double>* radii = &(distances.back().second);
            p = pose.transform(cylinders_[part].first);
            for (int seg = 0; seg <= segment; ++seg) {
                radii->push_back(cylinders_[part].second + std::max(
                                     PointDistanceToAxis(p - ax_P[seg], ax_O[seg]),
                                     PointDistanceToAxis(p_prev - ax_P[seg], ax_O[seg])));
            }
            p_prev = p;
        }
    }
    return distances;
}

std::vector<std::pair<double, double> > FclEnvironment::GetAngleRanges() const {
    return limits_;
}

void FclEnvironment::LoadDhAndRanges(const Robot& robot) {
    dh_inverted_.emplace_back(new fcl::Transform3f);
    for (const Segment& param : robot.segments()) {
        dh_parameters_.emplace_back(new fcl::Transform3f);
        fcl::Transform3f* t = dh_parameters_.back().get();

        fcl::Matrix3f rotations;
        rotations.setEulerZYX(0.0, 0.0, param.theta() * deg_to_rad());
        *t *= fcl::Transform3f(rotations);
        *t *= fcl::Transform3f(fcl::Vec3f(param.a(), 0.0, param.d()));
        rotations.setEulerZYX(param.alpha() * deg_to_rad(), 0.0, 0.0);
        *t *= fcl::Transform3f(rotations);

        dh_inverted_.emplace_back(new fcl::Transform3f(*t));
        fcl::Transform3f* t_inv = dh_inverted_.back().get();
        t_inv->inverse();
        *t_inv *= *dh_inverted_[dh_inverted_.size() - 2];
        CHECK (param.has_range() && param.range().has_min() &&
               param.range().has_max())
                << "Range missing from Protocol Buffer at:" << std::endl
                << param.DebugString();
        limits_.emplace_back(param.range().min(), param.range().max());
    }
}

std::vector<fcl::Vec3f> FclEnvironment::getOBBVertices(int segment) const {
    double w = obb_sizes_[segment][0];
    double h = obb_sizes_[segment][1];
    double d = obb_sizes_[segment][2];

    //LOG(INFO) << w << "," << h << "," << d;

    fcl::Vec3f center = parts_[segment]->getAABB().center();

    std::vector<fcl::Vec3f> points(8);

    points[0] = fcl::Vec3f(0.5 * w, -0.5 * h, 0.5 * d);
    points[1] = fcl::Vec3f(0.5 * w, 0.5 * h, 0.5 * d);
    points[2] = fcl::Vec3f(-0.5 * w, 0.5 * h, 0.5 * d);
    points[3] = fcl::Vec3f(-0.5 * w, -0.5 * h, 0.5 * d);
    points[4] = fcl::Vec3f(0.5 * w, -0.5 * h, -0.5 * d);
    points[5] = fcl::Vec3f(0.5 * w, 0.5 * h, -0.5 * d);
    points[6] = fcl::Vec3f(-0.5 * w, 0.5 * h, -0.5 * d);
    points[7] = fcl::Vec3f(-0.5 * w, -0.5 * h, -0.5 * d);

    for (int i = 0; i < points.size(); ++i)
        points[i] = parts_[segment]->getRotation() * points[i] + center;

    //LOG(INFO) << points.size();
    return points;

}

void FclEnvironment::ConfigureFromPB(
        const EnvironmentConfig& configuration,
        const boost::filesystem::path& config_file_path) {
    CHECK(configuration.has_robot_config() || configuration.has_robot_filename())
            << "Robot config missing from Protocol Buffer:" << std::endl
            << configuration.DebugString();
    CHECK(configuration.has_environment_filename())
            << "Environment missing from Protocol Buffer:" << std::endl
            << configuration.DebugString();
    boost::filesystem::path robot_file_path(config_file_path);
    Robot robot = configuration.robot_config();
    if (configuration.has_robot_filename()) {
        robot_file_path /= configuration.robot_filename();
        std::ifstream fin(robot_file_path.native());
        std::string input_string((std::istreambuf_iterator<char>(fin)),
                                 std::istreambuf_iterator<char>());
        bool success = google::protobuf::TextFormat::ParseFromString(
                    input_string, &robot);
        fin.close();
        CHECK(success) << "Failed parsing file: " << robot_file_path.native();
    }
    robot_file_path.remove_filename();

    std::vector<int> parts_per_joint;
    std::vector<std::string> parts;
    for (const Segment& item : robot.segments()) {
        parts_per_joint.push_back(item.parts_size());
        for (const std::string& part : item.parts())
            parts.push_back((robot_file_path / part).native());
    }

    const std::string& environment = configuration.environment_filename();
    double max_underestimate = configuration.max_underestimate();

    part_count_ = parts.size();
    variance_ = max_underestimate;
    is_joint_start_.resize(parts.size(), false);

    LoadDhAndRanges(robot);

    int part_index = 0;
    int segment_index = -1;
    is_joint_start_[0] = true;
    for (int parts_c : parts_per_joint) {
        part_index += parts_c;
        if (part_index < part_count_)
            is_joint_start_[part_index] = true;
    }
    part_index = 0;
    for (const std::string& part : parts) {
        if (is_joint_start_[part_index])
            ++segment_index;
        double l, r;
        fcl::Vec3f v = dh_parameters_[segment_index]->getTranslation();
        v.normalize();
        double x = v[0];
        double y = v[1];
        double z = v[2];
        part_meshes_.emplace_back(ParseModel(
                                      part, dh_inverted_[segment_index].get(), x, y, z, &l, &r));

        fcl::BVHModel<fcl::OBBRSS>* aug_model =
                new fcl::BVHModel<fcl::OBBRSS>(
                    *ParseModel(part, dh_inverted_[segment_index].get(), x, y, z, &l, &r));
        aug_part_meshes_.emplace_back(
                    ComputeAutmentedModel(aug_model, 3.0));
        parts_.emplace_back(new fcl::CollisionObject(
                                part_meshes_.back(), fcl::Transform3f()));
        double width = parts_.back()->getAABB().width();
        double height = parts_.back()->getAABB().height();
        double depth = parts_.back()->getAABB().depth();
        obb_sizes_.emplace_back(fcl::Vec3f(width, height, depth));
        aug_parts_.emplace_back(new fcl::CollisionObject(
                                    aug_part_meshes_.back(), fcl::Transform3f()));
        cylinders_.emplace_back(fcl::Vec3f(x * l, y * l, z * l), r);
        part_index++;
    }
    double l, r;
    objl::Loader Loader;

    Loader.LoadFile((config_file_path / environment).native());

    //LOG(INFO) << "DHinv: " << dh_inverted_[0].get()->getRotation();

    environment_mesh_.resize(Loader.LoadedMeshes.size());
    environment_.resize(Loader.LoadedMeshes.size());
    for (int i = 0; i < Loader.LoadedMeshes.size(); i++) {

        environment_mesh_[i].reset(ParseModelFromObjMeshPart(Loader, i));
        environment_[i].reset(new fcl::CollisionObject(environment_mesh_[i], fcl::Transform3f()));
    }

}

}  // namespace environment
}  // namespace bubblesmp
}  // namespace ademovic
}  // namespace com
