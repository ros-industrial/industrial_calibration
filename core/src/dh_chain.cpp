#include <industrial_calibration/core/dh_chain.h>
#include <industrial_calibration/core/serialization.h>

#include <random>

namespace industrial_calibration
{
// DH Transform

DHTransform::DHTransform(const Eigen::Vector4d& params_, DHJointType type_)
  : DHTransform(params_, type_, "joint", std::random_device{}())
{
}

DHTransform::DHTransform(const Eigen::Vector4d& params_, DHJointType type_, const std::string& name_)
  : DHTransform(params_, type_, name_, std::random_device{}())
{
}

DHTransform::DHTransform(const Eigen::Vector4d& params_, DHJointType type_, const std::string& name_,
                         std::size_t random_seed_)
  : params(params_), type(type_), name(name_), random_seed(random_seed_)
{
}

double DHTransform::createRandomJointValue() const
{
  // Create a static random number generator so that this object does not get continuously re-instantiated
  static std::mt19937 mt_rand = std::mt19937(random_seed);

  // Create a new uniform distribution object with the current min and max values
  std::uniform_real_distribution<double> dist(min, max);

  return dist(mt_rand);
}

std::array<std::string, 4> DHTransform::getParamLabels() const
{
  return std::array<std::string, 4>({
      name + "_d",
      name + "_theta",
      name + "_r",
      name + "_alpha",
  });
}

// DH Chain

DHChain::DHChain(std::vector<DHTransform> transforms, const Eigen::Isometry3d& base_offset)
  : transforms_(std::move(transforms)), base_offset_(base_offset)
{
}

DHChain::DHChain(const DHChain& other, const Eigen::MatrixX4d& dh_offsets)
{
  transforms_.reserve(other.transforms_.size());
  for (std::size_t i = 0; i < other.transforms_.size(); ++i)
  {
    const DHTransform& transform = other.transforms_.at(i);
    transforms_.emplace_back(transform.params + dh_offsets.row(i).transpose(), transform.type);
  }

  base_offset_ = other.base_offset_;
}

Eigen::VectorXd DHChain::createUniformlyRandomPose() const
{
  Eigen::VectorXd joints(transforms_.size());
  for (std::size_t i = 0; i < transforms_.size(); ++i)
  {
    joints[i] = transforms_[i].createRandomJointValue();
  }
  return joints;
}

std::size_t DHChain::dof() const { return transforms_.size(); }

Eigen::MatrixX4d DHChain::getDHTable() const
{
  Eigen::MatrixX4d out(dof(), 4);
  for (std::size_t i = 0; i < transforms_.size(); ++i)
  {
    out.row(i) = transforms_[i].params.transpose();
  }
  return out;
}

std::vector<DHJointType> DHChain::getJointTypes() const
{
  std::vector<DHJointType> out;
  out.reserve(transforms_.size());
  for (const auto& t : transforms_)
  {
    out.push_back(t.type);
  }
  return out;
}

std::vector<std::array<std::string, 4>> DHChain::getParamLabels() const
{
  std::vector<std::array<std::string, 4>> out;
  for (const auto& t : transforms_)
  {
    out.push_back(t.getParamLabels());
  }
  return out;
}

Eigen::Isometry3d DHChain::getBaseOffset() const { return base_offset_; }

Eigen::Isometry3d DHChain::getRelativeTransform(int joint_index, double value) const
{
  if (joint_index < 0 || joint_index >= static_cast<int>(transforms_.size()))
    throw std::runtime_error("getRelativeTransform, Invalid joint index");

  return transforms_[joint_index].createRelativeTransform(value);
}

}  // namespace industrial_calibration

using namespace industrial_calibration;

// @cond
namespace YAML
{
bool convert<DHTransform>::decode(const Node& n, DHTransform& val)
{
  // Joint type
  switch (static_cast<DHJointType>(n["type"].as<unsigned>()))
  {
    case DHJointType::LINEAR:
      val.type = DHJointType::LINEAR;
      break;
    case DHJointType::REVOLUTE:
      val.type = DHJointType::REVOLUTE;
      break;
    default:
      throw std::runtime_error("Invalid joint type");
  }

  // DH offsets
  {
    YAML::Node d = n["d"];
    YAML::Node theta = n["theta"];
    YAML::Node r = n["r"];
    YAML::Node alpha = n["alpha"];

    Eigen::Vector4d params;
    params << d.as<double>(), theta.as<double>(), r.as<double>(), alpha.as<double>();

    val.params = params;
  }

  val.name = n["joint_name"].as<std::string>();

  return true;
}

bool convert<DHChain>::decode(const Node& n, DHChain& val)
{
  val.base_offset_ = n["base_transform"].as<Eigen::Isometry3d>();

  const YAML::Node& transforms = n["dh_transforms"];
  val.transforms_.clear();
  val.transforms_.reserve(transforms.size());
  for (std::size_t i = 0; i < transforms.size(); i++)
  {
    val.transforms_.push_back(transforms[i].as<DHTransform>());
  }

  return true;
}

}  // namespace YAML
// @endcond
