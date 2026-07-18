#include <ros2_control_robot_dynamics/robot_model.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <chrono>
#include <future>
#include <mutex>
#include <optional>
#include <iostream>

namespace compliant_controllers {

struct RobotModel::Impl {
  pinocchio::Model model;
  pinocchio::Data data{model};
  pinocchio::FrameIndex ee_id{0};
  std::string ee_name;
  TcpConfiguration tcp;
  bool initialized{false};
  bool last_update_ok{false};
  int nj{0};
  Eigen::VectorXd q_cache; // sized at init
  std::vector<int> controlled_q_indices;
  std::vector<int> controlled_v_indices;

  Eigen::Matrix3d tcpRotationMatrix() const {
    const Eigen::AngleAxisd roll(tcp.rotation_rpy.x(), Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(tcp.rotation_rpy.y(), Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(tcp.rotation_rpy.z(), Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
  }

  Eigen::Vector3d tcpTranslationWorldFromPlacement(const pinocchio::SE3& placement) const {
    return placement.rotation() * tcp.translation;
  }

  void applyTcpToPose(const Eigen::Vector3d& ee_position,
                      const Eigen::Quaterniond& ee_orientation,
                      Eigen::Vector3d& position,
                      Eigen::Quaterniond& orientation) const {
    if (!tcp.enabled) {
      position = ee_position;
      orientation = ee_orientation.normalized();
      return;
    }

    const Eigen::Matrix3d ee_rotation = ee_orientation.normalized().toRotationMatrix();
    const Eigen::Matrix3d tcp_rotation_world = ee_rotation * tcpRotationMatrix();
    position = ee_position + ee_rotation * tcp.translation;
    orientation = Eigen::Quaterniond(tcp_rotation_world).normalized();
  }

  void applyTcpToJacobian(const pinocchio::SE3& placement,
                          Eigen::Ref<Eigen::Matrix<double, 6, Eigen::Dynamic>> J_out) const {
    if (!tcp.enabled || tcp.translation.isZero(0.0)) {
      return;
    }

    const Eigen::Vector3d offset_world = tcpTranslationWorldFromPlacement(placement);
    const Eigen::Matrix3d skew =
      (Eigen::Matrix3d() << 0.0, -offset_world.z(), offset_world.y(),
                            offset_world.z(), 0.0, -offset_world.x(),
                           -offset_world.y(), offset_world.x(), 0.0).finished();
    J_out.topRows<3>().noalias() -= skew * J_out.bottomRows<3>();
  }

  bool build(const std::string &urdf_xml, const std::string &ee_hint,
             const std::vector<std::string> &controlled_joints) {
    try {
      pinocchio::urdf::buildModelFromXML(urdf_xml, model);
      data = pinocchio::Data(model);
      // choose ee frame
      if(!ee_hint.empty() && model.existFrame(ee_hint)) {
        ee_id = model.getFrameId(ee_hint);
        ee_name = ee_hint;
      } else {
        const std::vector<std::string> candidates{"tool0","ee_link","fr3_hand","panda_hand"};
        bool found=false;
        for(const auto & n: candidates){ if(model.existFrame(n)){ ee_id = model.getFrameId(n); ee_name = n; found=true; break; }}
        if(!found && !model.frames.empty()) { ee_id = model.frames.size()-1; ee_name = model.frames[ee_id].name; } // fallback last
      }
      controlled_q_indices.clear();
      controlled_v_indices.clear();
      if (!controlled_joints.empty()) {
        for (const auto & joint_name : controlled_joints) {
          if (!model.existJointName(joint_name)) {
            std::cerr << "RobotModel build failed: controlled joint '" << joint_name
                      << "' is not present in the URDF" << std::endl;
            initialized = false;
            return false;
          }
          const auto joint_id = model.getJointId(joint_name);
          const int nq = model.nqs[static_cast<size_t>(joint_id)];
          const int nv = model.nvs[static_cast<size_t>(joint_id)];
          if (nq != 1 || nv != 1) {
            std::cerr << "RobotModel build failed: controlled joint '" << joint_name
                      << "' has nq=" << nq << " nv=" << nv
                      << "; only 1-DoF joints are supported by this controller" << std::endl;
            initialized = false;
            return false;
          }
          controlled_q_indices.push_back(model.idx_qs[static_cast<size_t>(joint_id)]);
          controlled_v_indices.push_back(model.idx_vs[static_cast<size_t>(joint_id)]);
        }
      } else {
        controlled_q_indices.reserve(static_cast<size_t>(model.nv));
        controlled_v_indices.reserve(static_cast<size_t>(model.nv));
        for (int i = 0; i < model.nv; ++i) {
          controlled_q_indices.push_back(i);
          controlled_v_indices.push_back(i);
        }
      }
      nj = static_cast<int>(controlled_v_indices.size());
      q_cache = Eigen::VectorXd::Zero(model.nq);
      initialized = true;
      last_update_ok = false;
      return true;
    } catch(const std::exception &e) {
      std::cerr << "RobotModel build failed: " << e.what() << std::endl;
      initialized = false;
      return false;
    }
  }

  bool update(const Eigen::Ref<const Eigen::VectorXd>& q_in) {
    if(!initialized) return false;
    if (q_in.size() != static_cast<long>(controlled_q_indices.size())) return false;
    q_cache.setZero();
    for (Eigen::Index i = 0; i < q_in.size(); ++i) {
      q_cache[static_cast<Eigen::Index>(controlled_q_indices[static_cast<size_t>(i)])] = q_in[i];
    }
    try {
      pinocchio::forwardKinematics(model, data, q_cache);
      pinocchio::computeJointJacobians(model, data, q_cache);
      pinocchio::updateFramePlacements(model, data);
      last_update_ok = true;
      return true;
    } catch(const std::exception &e) {
      last_update_ok = false;
      return false;
    }
  }

  bool pose(Eigen::Vector3d &p, Eigen::Quaterniond &q) const {
    if(!(initialized && last_update_ok)) return false;
    const auto & placement = data.oMf[ee_id];
    applyTcpToPose(placement.translation(),
                   Eigen::Quaterniond(placement.rotation()),
                   p, q);
    return true;
  }

  bool jacobian(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out) {
    if(!(initialized && last_update_ok)) return false;
    // pinocchio requires non-const data reference for getFrameJacobian
    Eigen::Matrix<double,6,Eigen::Dynamic> J_full = pinocchio::getFrameJacobian(model, data, ee_id, pinocchio::LOCAL_WORLD_ALIGNED);
    const int used_cols = std::min<int>(J_out.cols(), controlled_v_indices.size());
    if (used_cols <= 0 || J_full.cols() <= 0) {
      return false;
    }
    J_out.leftCols(used_cols).setZero();
    for (int i = 0; i < used_cols; ++i) {
      const int source_col = controlled_v_indices[static_cast<size_t>(i)];
      if (source_col >= J_full.cols()) {
        return false;
      }
      J_out.col(i) = J_full.col(source_col);
    }
    applyTcpToJacobian(data.oMf[ee_id], J_out.leftCols(used_cols));
    return true;
  }

  bool jacobianAndPseudoInverse(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out,
                                Eigen::Ref<Eigen::Matrix<double,Eigen::Dynamic,6>> J_pinv_out,
                                double rcond) {
    if(!(initialized && last_update_ok)) return false;
    Eigen::Matrix<double,6,Eigen::Dynamic> J_full = pinocchio::getFrameJacobian(model, data, ee_id, pinocchio::LOCAL_WORLD_ALIGNED);
    const int used_cols = std::min<int>({static_cast<int>(J_out.cols()),
                                         static_cast<int>(J_pinv_out.rows()),
                                         static_cast<int>(controlled_v_indices.size())});
    if (used_cols <= 0 || J_full.cols() <= 0) {
      return false;
    }

    Eigen::Matrix<double,6,Eigen::Dynamic> J_used(6, used_cols);
    for (int i = 0; i < used_cols; ++i) {
      const int source_col = controlled_v_indices[static_cast<size_t>(i)];
      if (source_col >= J_full.cols()) {
        return false;
      }
      J_used.col(i) = J_full.col(source_col);
    }
    applyTcpToJacobian(data.oMf[ee_id], J_used);
    J_out.leftCols(used_cols) = J_used;

    // Compute J^+ for J_used (6 x used_cols), yielding used_cols x 6.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_used, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto & s = svd.singularValues();
    if(s.size()==0){ J_pinv_out.topRows(used_cols).setZero(); return true; }
    double tol = rcond * std::max(J_used.rows(), J_used.cols()) * s(0);
    Eigen::VectorXd inv = s.unaryExpr([&](double v){ return v > tol ? 1.0/v : 0.0; });
    J_pinv_out.topRows(used_cols).noalias() = svd.matrixV() * inv.asDiagonal() * svd.matrixU().transpose();
    return true;
  }

  bool gravity(Eigen::Ref<Eigen::VectorXd> g_out) {
    if(!(initialized && last_update_ok)) return false;
    if(g_out.size() <= 0) return false;
    // Compute generalized gravity torques using RNEA with zero velocities and accelerations
    Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd tau_g = pinocchio::rnea(model, data, q_cache, v_zero, a_zero);
    const int used = std::min<int>(static_cast<int>(g_out.size()),
                                   static_cast<int>(controlled_v_indices.size()));
    for (int i = 0; i < used; ++i) {
      const int source_index = controlled_v_indices[static_cast<size_t>(i)];
      if (source_index >= tau_g.size()) {
        return false;
      }
      g_out[i] = tau_g[source_index];
    }
    return true;
  }

  bool coriolis(const Eigen::Ref<const Eigen::VectorXd>& dq_in,
                Eigen::Ref<Eigen::VectorXd> c_out) {
    if(!(initialized && last_update_ok)) return false;
    if (dq_in.size() != static_cast<long>(controlled_v_indices.size()) || c_out.size() <= 0) {
      return false;
    }

    Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model.nv);
    for (Eigen::Index i = 0; i < dq_in.size(); ++i) {
      v_full[static_cast<Eigen::Index>(controlled_v_indices[static_cast<size_t>(i)])] = dq_in[i];
    }

    Eigen::VectorXd a_zero = Eigen::VectorXd::Zero(model.nv);
    const Eigen::VectorXd tau_non_linear = pinocchio::rnea(model, data, q_cache, v_full, a_zero);
    const Eigen::VectorXd tau_gravity = pinocchio::rnea(model, data, q_cache,
                                                        Eigen::VectorXd::Zero(model.nv), a_zero);
    const Eigen::VectorXd tau_c = tau_non_linear - tau_gravity;

    const int used = std::min<int>(static_cast<int>(c_out.size()),
                                   static_cast<int>(controlled_v_indices.size()));
    for (int i = 0; i < used; ++i) {
      const int source_index = controlled_v_indices[static_cast<size_t>(i)];
      if (source_index >= tau_c.size()) {
        return false;
      }
      c_out[i] = tau_c[source_index];
    }
    return true;
  }

  bool massMatrix(Eigen::Ref<Eigen::MatrixXd> m_out) {
    if (!(initialized && last_update_ok)) {
      return false;
    }
    if (m_out.rows() != nj || m_out.cols() != nj) {
      return false;
    }

    pinocchio::crba(model, data, q_cache);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();

    m_out.setZero();
    for (int r = 0; r < nj; ++r) {
      const int source_r = controlled_v_indices[static_cast<size_t>(r)];
      if (source_r >= data.M.rows()) {
        return false;
      }
      for (int c = 0; c < nj; ++c) {
        const int source_c = controlled_v_indices[static_cast<size_t>(c)];
        if (source_c >= data.M.cols()) {
          return false;
        }
        m_out(r, c) = data.M(source_r, source_c);
      }
    }
    return true;
  }

  bool massCoriolis(const Eigen::Ref<const Eigen::VectorXd>& dq_in,
                    const Eigen::Ref<const Eigen::VectorXd>& ddq_in,
                    Eigen::Ref<Eigen::VectorXd> mc_out) {
    if (!(initialized && last_update_ok)) return false;
    if (dq_in.size() != static_cast<long>(controlled_v_indices.size()) ||
        ddq_in.size() != static_cast<long>(controlled_v_indices.size()) ||
        mc_out.size() <= 0) {
      return false;
    }

    Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a_full = Eigen::VectorXd::Zero(model.nv);
    for (Eigen::Index i = 0; i < dq_in.size(); ++i) {
      const auto source = static_cast<Eigen::Index>(controlled_v_indices[static_cast<size_t>(i)]);
      v_full[source] = dq_in[i];
      a_full[source] = ddq_in[i];
    }

    const Eigen::VectorXd tau_full = pinocchio::rnea(model, data, q_cache, v_full, a_full);
    const Eigen::VectorXd tau_g = pinocchio::rnea(model, data, q_cache,
                                                  Eigen::VectorXd::Zero(model.nv),
                                                  Eigen::VectorXd::Zero(model.nv));
    const Eigen::VectorXd tau_mc = tau_full - tau_g;

    const int used = std::min<int>(static_cast<int>(mc_out.size()),
                                   static_cast<int>(controlled_v_indices.size()));
    for (int i = 0; i < used; ++i) {
      const int source_index = controlled_v_indices[static_cast<size_t>(i)];
      if (source_index >= tau_mc.size()) {
        return false;
      }
      mc_out[i] = tau_mc[source_index];
    }
    return true;
  }
};

RobotModel::RobotModel() : impl_(std::make_unique<Impl>()) {}
RobotModel::~RobotModel() = default;
RobotModel::RobotModel(RobotModel&&) noexcept = default;
RobotModel& RobotModel::operator=(RobotModel&&) noexcept = default;

bool RobotModel::init(const std::string &urdf_xml, const std::string &ee_hint,
                      const std::vector<std::string> &controlled_joints) {
  if(!impl_) impl_ = std::make_unique<Impl>();
  bool ok = impl_->build(urdf_xml, ee_hint, controlled_joints);
  nj = ok ? impl_->nj : 0;
  return ok;
}

bool RobotModel::update(const Eigen::Ref<const Eigen::VectorXd>& q){
  if(!impl_) return false;
  return impl_->update(q);
}

bool RobotModel::configureTcp(bool enabled,
                              const Eigen::Vector3d& translation,
                              const Eigen::Vector3d& rotation_rpy) {
  if (!impl_) {
    impl_ = std::make_unique<Impl>();
  }
  impl_->tcp.enabled = enabled;
  impl_->tcp.translation = translation;
  impl_->tcp.rotation_rpy = rotation_rpy;
  return true;
}

bool RobotModel::configureTcpFromParameters(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
  bool enabled,
  const std::string& profile_node) {
  if (node == nullptr) {
    return configureTcp(enabled);
  }

  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_rpy = Eigen::Vector3d::Zero();
  const auto read_vec3 = [](const std::vector<double>& values, Eigen::Vector3d& target) {
    if (values.size() != 3) {
      return false;
    }
    target << values[0], values[1], values[2];
    return true;
  };

  bool configured = false;
  std::vector<double> values;
  if (node->get_parameter("end_effector.tcp.translation", values) ||
      node->get_parameter("end_effector_profile.tcp.translation", values)) {
    configured = read_vec3(values, translation) || configured;
  }
  values.clear();
  if (node->get_parameter("end_effector.tcp.rotation_rpy", values) ||
      node->get_parameter("end_effector_profile.tcp.rotation_rpy", values)) {
    configured = read_vec3(values, rotation_rpy) || configured;
  }

  if (!profile_node.empty()) {
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node, profile_node);
    if (parameters_client->wait_for_service(std::chrono::seconds(1))) {
      const std::vector<std::string> names{
        "end_effector.use_tcp", "end_effector.tcp.translation", "end_effector.tcp.rotation_rpy",
        "end_effector_profile.use_tcp", "end_effector_profile.tcp.translation",
        "end_effector_profile.tcp.rotation_rpy",
      };
      auto future = parameters_client->get_parameters(names);
      if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        for (const auto& parameter : future.get()) {
          if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            continue;
          }

          const auto& name = parameter.get_name();
          const auto type = parameter.get_type();
          if (type == rclcpp::ParameterType::PARAMETER_BOOL &&
              (name == "end_effector.use_tcp" || name == "end_effector_profile.use_tcp")) {
            enabled = parameter.as_bool();
            configured = true;
          } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            const auto profile_values = parameter.as_double_array();
            if (name == "end_effector.tcp.translation" ||
                name == "end_effector_profile.tcp.translation") {
              configured = read_vec3(profile_values, translation) || configured;
            } else if (name == "end_effector.tcp.rotation_rpy" ||
                       name == "end_effector_profile.tcp.rotation_rpy") {
              configured = read_vec3(profile_values, rotation_rpy) || configured;
            }
          }
        }
      } else {
        RCLCPP_WARN(node->get_logger(),
                    "Timed out reading TCP parameters from '%s'.", profile_node.c_str());
      }
    } else {
      RCLCPP_WARN(node->get_logger(),
                  "End-effector TCP profile parameter node '%s' is not available.",
                  profile_node.c_str());
    }
  }

  configureTcp(enabled, translation, rotation_rpy);
  return configured || enabled;
}

bool RobotModel::tcpEnabled() const {
  return impl_ && impl_->tcp.enabled;
}

bool RobotModel::getPose(Eigen::Vector3d &position, Eigen::Quaterniond &orientation) const {
  if(!impl_) return false;
  return impl_->pose(position, orientation);
}

bool RobotModel::transformToTcp(Eigen::Vector3d& position, Eigen::Quaterniond& orientation) const {
  if (!impl_) return false;
  if (!impl_->tcp.enabled) {
    return true;
  }
  const Eigen::Vector3d ee_position = position;
  const Eigen::Quaterniond ee_orientation = orientation;
  impl_->applyTcpToPose(ee_position, ee_orientation, position, orientation);
  return true;
}

bool RobotModel::getJacobian(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out) {
  if(!impl_) return false;
  return impl_->jacobian(J_out);
}

bool RobotModel::getJacobianAndPseudoInverse(Eigen::Ref<Eigen::Matrix<double,6,Eigen::Dynamic>> J_out,
                                             Eigen::Ref<Eigen::Matrix<double,Eigen::Dynamic,6>> J_pinv_out,
                                             double rcond) {
  if(!impl_) return false;
  return impl_->jacobianAndPseudoInverse(J_out, J_pinv_out, rcond);
}

bool RobotModel::getGravity(Eigen::Ref<Eigen::VectorXd> g_out) {
  if(!impl_) return false;
  return impl_->gravity(g_out);
}

bool RobotModel::getCoriolis(const Eigen::Ref<const Eigen::VectorXd>& dq,
                             Eigen::Ref<Eigen::VectorXd> c_out) {
  if(!impl_) return false;
  return impl_->coriolis(dq, c_out);
}

bool RobotModel::getMassMatrix(Eigen::Ref<Eigen::MatrixXd> m_out) {
  if (!impl_) return false;
  return impl_->massMatrix(m_out);
}

bool RobotModel::getMassCoriolisTorque(const Eigen::Ref<const Eigen::VectorXd>& dq,
                                       const Eigen::Ref<const Eigen::VectorXd>& ddq,
                                       Eigen::Ref<Eigen::VectorXd> mc_out) {
  if (!impl_) return false;
  return impl_->massCoriolis(dq, ddq, mc_out);
}

bool RobotModel::valid() const {
  return impl_ && impl_->initialized && impl_->last_update_ok;
}

// Optional accessor for EE frame name
std::string RobotModel::endEffectorFrame() const {
  return impl_ ? impl_->ee_name : std::string();
}

} // namespace compliant_controllers
