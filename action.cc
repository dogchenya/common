// Copyright(C) Gaussian Robot. All rights reserved.
//
// @file clean_mode_controller.h
// @brief control device
// @author Sun Shichao<sunshichao@gs-robot.com>
// @version 1.0.0.0
// @date 2018-06-15
//

#include "common/common_inc.h"
#include "common/counter.h"
#include "common/action.h"

namespace device {
// 执行动作
void ActionManager::Start(const std::string& slot, const std::string& name, ActionPtr act_ptr) {
  auto it = act_map_.find(slot);
  if (it != act_map_.end()) {
    if (name == it->first) return;

    Stop(slot);
  }

  auto& context = act_map_[slot];
  context.first = name;
  context.second = act_ptr;

  act_ptr->Start();
  if (act_ptr->IsFinished()) act_map_.erase(slot);
}
// 暂停动作
void ActionManager::Pause(const std::string& slot) {
  auto it = act_map_.find(slot);
  if (it != act_map_.end()) {
    it->second.second->Pause();
  }
}
// 暂停动作
void ActionManager::PauseAll() {
  std::vector<ActionPtr> buffer;
  buffer.reserve(act_map_.size());

  for (auto& context : act_map_) {
    buffer.push_back(context.second.second);
  }

  for (auto& act_ptr : buffer) {
    act_ptr->Pause();
  }
}
// 恢复动作
void ActionManager::Resume(const std::string& slot) {
  auto it = act_map_.find(slot);
  if (it != act_map_.end()) {
    it->second.second->Resume();
  }
}
// 恢复动作
void ActionManager::ResumeAll() {
  std::vector<ActionPtr> buffer;
  buffer.reserve(act_map_.size());

  for (auto& context : act_map_) {
    buffer.push_back(context.second.second);
  }

  for (auto& act_ptr : buffer) {
    act_ptr->Resume();
  }
}
// 停止动作
void ActionManager::Stop(const std::string& slot) {
  auto it = act_map_.find(slot);
  if (it != act_map_.end()) {
    auto act_ptr = it->second.second;
    act_map_.erase(it);
    act_ptr->Stop();
  }
}
// 停止动作
void ActionManager::StopAll() {
  auto act_map = std::move(act_map_);
  for (auto& context : act_map) {
    context.second.second->Stop();
  }
}
// 更新动作
void ActionManager::Update(double delta_time) {
  update_buffer_.reserve(act_map_.size());

  for (auto& context : act_map_) {
    update_buffer_.emplace_back(context.first, context.second.second);
  }

  for (auto& context : update_buffer_) {
    context.second->Update(delta_time);
    if (!context.second->IsFinished()) context.second.reset();
  }

  for (auto& context : update_buffer_) {
    if (context.second) {
      auto it = act_map_.find(context.first);
      if (it != act_map_.end() && context.second == it->second.second) act_map_.erase(it);
    }
  }

  update_buffer_.clear();
}
const std::string& ActionManager::GetActionName(const std::string& slot) const {
  auto it = act_map_.find(slot);
  if (it != act_map_.end()) {
    return it->second.first;
  }

  static const std::string s_blank_;
  return s_blank_;
}
}  // namespace device
