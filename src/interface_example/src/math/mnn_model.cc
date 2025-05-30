#include "math/mnn_model.h"

namespace math {

MnnModel::MnnModel(const std::string& model_path) : model_path_(model_path) {
  // Constructs MNN model
  net_ = std::shared_ptr<MNN::Interpreter>(MNN::Interpreter::createFromFile(model_path_.c_str()));
  net_->setSessionMode(MNN::Interpreter::Session_Release);

  // Creates session with customized schedule config
  MNN::ScheduleConfig config;
  config.numThread = 1;
  session_ = net_->createSession(config);

  // Gets observation and actor tensor
  observations_tensor_ = net_->getSessionInput(session_, nullptr);
  actions_tensor_ = net_->getSessionOutput(session_, nullptr);

  // Allocates memory for actions
  actions_.setZero(actions_tensor_->shape().back());
}

Eigen::VectorXf& MnnModel::Inference(const Eigen::MatrixXf& observations) {
  memcpy(observations_tensor_->host<float>(), observations.data(), observations_tensor_->size());
  net_->runSession(session_);
  memcpy(actions_.data(), actions_tensor_->host<float>(), actions_tensor_->size());
  return actions_;
}

MnnModel::~MnnModel() {
  net_->releaseSession(session_);
  net_->releaseModel();
}
}  // namespace math
