#pragma once

#include <MNN/Interpreter.hpp>
#include <eigen3/Eigen/Core>
#include <memory>

namespace math {
class MnnModel {
 public:
  MnnModel() {}
  explicit MnnModel(const std::string& model_path);
  ~MnnModel();

  Eigen::VectorXf& Inference(const Eigen::MatrixXf& observations);

 private:
  std::string model_path_ = "";
  std::shared_ptr<MNN::Interpreter> net_;

  MNN::Session* session_ = nullptr;
  MNN::Tensor* observations_tensor_ = nullptr;
  MNN::Tensor* actions_tensor_ = nullptr;
  Eigen::VectorXf actions_;
};
}  // namespace math
