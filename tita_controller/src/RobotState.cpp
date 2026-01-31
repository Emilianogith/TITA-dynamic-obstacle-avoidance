#include <RobotState.hpp>

namespace labrob {

RobotState::RobotState()
  : position(Eigen::Vector3d::Zero()),
    orientation(1.0, 0.0, 0.0, 0.0),
    linear_velocity(Eigen::Vector3d::Zero()),
    angular_velocity(Eigen::Vector3d::Zero()) {

}

} // end namespace labrob