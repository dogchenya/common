// Copyright(C) Gaussian Robot. All rights reserved.
//
// @file common_inc.h
// @brief Common include file, used by precompiling-technology.
// @author <wujiadong@gs-robot.com>
// @version 1.0.0.0
// @date 2019-01-03
//

#ifndef INCLUDE_COMMON_ACTION_H_
#define INCLUDE_COMMON_ACTION_H_

#include <boost/function.hpp>
#include <iostream>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/counter.h"

namespace device {
enum class ActionEvent : uint8_t {
  STARTED,
  PAUSED,
  RESUMED,
  FINISHED,
  CANCELED
};

class Action;
using ActionPtr = boost::intrusive_ptr<Action>;
class Action : public common::Counter {
 public:
  enum class State : uint8_t { INIT, RUNNING, PAUSED, FINISHED, CANCELED };
  using FuncOnEvent = boost::function<void(const ActionPtr&, ActionEvent)>;

 public:
  Action() = default;
  virtual ~Action() = default;

  // 开始
  virtual bool Start() {
    if (State::INIT == state_) {
      state_ = State::RUNNING;
      for (auto& on_event : on_event_vec_) on_event(this, ActionEvent::STARTED);
      Update(0);
      return true;
    }
    return false;
  }
  // 恢复
  virtual bool Resume() {
    if (State::PAUSED == state_) {
      state_ = State::RUNNING;
      for (auto& on_event : on_event_vec_) on_event(this, ActionEvent::RESUMED);
      return true;
    }
    return false;
  }
  // 更新
  virtual double Update(double delta_time) {
    UpdateState();
    return delta_time;
  }
  // 暂停
  virtual bool Pause() {
    if (pausable_ && State::RUNNING == state_) {
      state_ = State::PAUSED;
      for (auto& on_event : on_event_vec_) on_event(this, ActionEvent::PAUSED);
      return true;
    }
    return false;
  }
  // 停止
  virtual bool Stop() {
    if (State::RUNNING == state_ || State::PAUSED == state_) {
      state_ = State::CANCELED;
      for (auto& on_event : on_event_vec_)
        on_event(this, ActionEvent::CANCELED);
      return true;
    }
    return false;
  }

  void SetPausable(bool pausable) { pausable_ = pausable; }
  bool GetPausable() const { return pausable_; }

  State GetState() const { return state_; }
  // 是否初始化状态
  bool IsInit() const { return State::INIT == state_; }
  // 是否执行中
  bool IsRunning() const { return State::RUNNING == state_; }
  // 是否已完成
  bool IsFinished() const { return State::FINISHED == state_; }
  // 是否已暂停
  bool IsPaused() const { return State::PAUSED == state_; }
  // 是否已取消
  bool IsCanceled() const { return State::CANCELED == state_; }

  // 是否已寿终正寝
  bool IsEndOfLife() const { return State::FINISHED <= state_; }

  void AddOnEvent(const FuncOnEvent& on_event) {
    on_event_vec_.push_back(on_event);
  }
  void AddOnEvent(FuncOnEvent&& on_event) {
    on_event_vec_.push_back(std::move(on_event));
  }

 protected:
  // 当前动作是否执行完毕
  virtual bool IsDone() const { return true; }
  // 更新状态
  virtual void UpdateState() {
    if (IsDone()) {
      state_ = State::FINISHED;
      for (auto& on_event : on_event_vec_)
        on_event(this, ActionEvent::FINISHED);
    }
  }

 private:
  bool pausable_{true};
  State state_{State::INIT};
  std::vector<FuncOnEvent> on_event_vec_;
};

// 延迟时间动作
class ActionIncompletable : public Action {
 public:
  using Action::Action;

  // 更新
  double Update(double /*delta_time*/) override { return 0; }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override { return false; }
};
using ActionIncompletablePtr = boost::intrusive_ptr<ActionIncompletable>;

// 延迟时间动作
class ActionWaitForTime : public Action {
  using SuperType = Action;

 public:
  explicit ActionWaitForTime(double wait_time) : wait_time_(wait_time) {}
  virtual ~ActionWaitForTime() = default;

  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    is_done_ = false;
    double used = delta_time;
    if (delayed_time_ + used >= wait_time_) {
      used = wait_time_ - delayed_time_;
      is_done_ = true;
    }
    delayed_time_ += used;
    return SuperType::Update(delta_time - used);
  }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override { return is_done_; }

 private:
  const double wait_time_{0.0};
  double delayed_time_{0.0};
  bool is_done_{false};
};
using ActionWaitForTimePtr = boost::intrusive_ptr<ActionWaitForTime>;

class ActionList : public Action {
  using SuperType = Action;

 public:
  ActionList() = default;
  virtual ~ActionList() = default;

  // 恢复
  bool Resume() override {
    if (SuperType::Resume()) {
      assert(index_ < actions_vec_.size());
      actions_vec_[index_]->Resume();
      return true;
    }
    return false;
  }
  // 更新
  double Update(double delta_time) override {
    while (index_ < actions_vec_.size()) {
      auto& act = *actions_vec_[index_];
      if (act.IsInit()) {
        act.Start();
        if (act.IsRunning() && IsPaused()) act.Pause();
      }
      if (delta_time > 0 && act.IsRunning()) {
        delta_time = act.Update(delta_time);
      }

      if (act.IsFinished())
        index_++;
      else
        break;
    }
    return SuperType::Update(delta_time);
  }
  // 暂停
  bool Pause() override {
    if (SuperType::Pause()) {
      assert(index_ < actions_vec_.size());
      actions_vec_[index_]->Pause();
      return true;
    }
    return false;
  }
  // 停止
  bool Stop() override {
    if (SuperType::Stop()) {
      assert(index_ < actions_vec_.size());
      actions_vec_[index_]->Stop();
      return true;
    }
    return false;
  }

  // 添加动作
  void AddAction(const ActionPtr& act_ptr) { actions_vec_.push_back(act_ptr); }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override { return index_ >= actions_vec_.size(); }

 private:
  std::vector<ActionPtr> actions_vec_;
  size_t index_{0};
};
using ActionListPtr = boost::intrusive_ptr<ActionList>;

class ActionSetWaitAny : public Action {
  using SuperType = Action;

 public:
  ActionSetWaitAny() = default;
  virtual ~ActionSetWaitAny() = default;

  // 恢复
  bool Resume() override {
    if (SuperType::Resume()) {
      for (auto& act_ptr : actions_vec_) {
        act_ptr->Resume();
      }
      return true;
    }
    return false;
  }
  // 更新
  double Update(double delta_time) override {
    double max_left = 0;
    for (auto& act_ptr : actions_vec_) {
      auto& act = *act_ptr;
      if (act.IsInit()) {
        act.Start();
      }
      if (act.IsRunning()) {
        auto left = act.Update(delta_time);
        if (left > max_left) max_left = left;
      }
    }
    return SuperType::Update(max_left);
  }
  // 暂停
  bool Pause() override {
    if (SuperType::Pause()) {
      for (auto& act_ptr : actions_vec_) {
        act_ptr->Pause();
      }
      return true;
    }
    return false;
  }
  // 停止
  bool Stop() override {
    if (SuperType::Stop()) {
      for (auto& act_ptr : actions_vec_) {
        act_ptr->Stop();
      }
      return true;
    }
    return false;
  }

  // 添加动作
  void AddAction(const ActionPtr& act_ptr) { actions_vec_.push_back(act_ptr); }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override {
    for (auto& act_ptr : actions_vec_) {
      auto& act = *act_ptr;
      if (act.IsFinished()) return true;
    }
    return false;
  }
  // 更新状态
  void UpdateState() {
    SuperType::UpdateState();
    if (IsFinished()) {
      for (auto& act_ptr : actions_vec_) {
        if (!act_ptr->IsFinished()) act_ptr->Stop();
      }
    }
  }

 private:
  std::vector<ActionPtr> actions_vec_;
};
using ActionSetWaitAnyPtr = boost::intrusive_ptr<ActionSetWaitAny>;

class ActionSetWaitAll : public Action {
  using SuperType = Action;

 public:
  ActionSetWaitAll() = default;
  virtual ~ActionSetWaitAll() = default;

  // 恢复
  bool Resume() override {
    if (SuperType::Resume()) {
      for (auto& act_ptr : actions_vec_) {
        act_ptr->Resume();
      }
      return true;
    }
    return false;
  }  // 更新
  // 更新
  double Update(double delta_time) override {
    double min_left = delta_time;
    for (auto& act_ptr : actions_vec_) {
      auto& act = *act_ptr;
      if (act.IsInit()) {
        act.Start();
      }
      if (act.IsRunning()) {
        auto left = act.Update(delta_time);
        if (left < min_left) min_left = left;
      }
    }
    return SuperType::Update(min_left);
  }
  // 暂停
  bool Pause() override {
    if (SuperType::Pause()) {
      for (auto& act_ptr : actions_vec_) {
        act_ptr->Pause();
      }
      return true;
    }
    return false;
  }
  // 停止
  bool Stop() override {
    if (SuperType::Stop()) {
      for (auto& act_ptr : actions_vec_) {
        act_ptr->Stop();
      }
      return true;
    }
    return false;
  }

  // 添加动作
  void AddAction(const ActionPtr& act_ptr) { actions_vec_.push_back(act_ptr); }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override {
    for (auto& act_ptr : actions_vec_) {
      auto& act = *act_ptr;
      if (!act.IsFinished()) return false;
    }
    return true;
  }

 private:
  std::vector<ActionPtr> actions_vec_;
};
using ActionSetWaitAllPtr = boost::intrusive_ptr<ActionSetWaitAll>;

class ActionOperatorCb : public Action {
  using SuperType = Action;

 public:
  using FuncOnOperate = boost::function<void()>;

 public:
  explicit ActionOperatorCb(FuncOnOperate&& on_operate)
      : on_operate_(std::move(on_operate)) {}
  virtual ~ActionOperatorCb() = default;

  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    on_operate_();
    return SuperType::Update(delta_time);
  }

 private:
  FuncOnOperate on_operate_;
};
using ActionOperatorCbPtr = boost::intrusive_ptr<ActionOperatorCb>;

class ActionCheckerCb : public Action {
  using SuperType = Action;

 public:
  using FuncOnCheck = boost::function<bool()>;

 public:
  explicit ActionCheckerCb(FuncOnCheck&& on_check)
      : on_check_(std::move(on_check)) {}
  virtual ~ActionCheckerCb() = default;

  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    is_done_ = on_check_();
    return SuperType::Update(is_done_ ? delta_time : 0);
  }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override { return is_done_; }

 private:
  FuncOnCheck on_check_;
  bool is_done_{false};
};
using ActionCheckerCbPtr = boost::intrusive_ptr<ActionCheckerCb>;

// on_check满足就累加时间，直到wait_time时间
class ActionTimedCheckerCb : public Action {
  using SuperType = Action;

 public:
  using FuncOnCheck = boost::function<bool()>;

 public:
  explicit ActionTimedCheckerCb(double wait_time, FuncOnCheck&& on_check)
      : wait_time_(wait_time), on_check_(std::move(on_check)) {}
  virtual ~ActionTimedCheckerCb() = default;

  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    can_accumulate_time_ = on_check_();
    delayed_time_ += can_accumulate_time_ ? delta_time : 0;
    return SuperType::Update(can_accumulate_time_ ? delta_time : 0);
  }

  double GetRemainDuration() const {
    return delayed_time_ < wait_time_ ? wait_time_ - delayed_time_ : 0;
  }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override {
    return (delayed_time_ >= wait_time_) || is_done_;
  }

 private:
  const double wait_time_{0.0};
  double delayed_time_{0.0};
  FuncOnCheck on_check_;
  bool is_done_{false};
  bool can_accumulate_time_{false};
};
using ActionTimedCheckerCbPtr = boost::intrusive_ptr<ActionTimedCheckerCb>;

class ActionOperatorCheckerCb : public Action {
  using SuperType = Action;

 public:
  using FuncOnOperate = boost::function<void()>;
  using FuncOnCheck = boost::function<bool()>;

 public:
  ActionOperatorCheckerCb(FuncOnOperate&& on_operate, FuncOnCheck&& on_check)
      : on_operate_(std::move(on_operate)), on_check_(std::move(on_check)) {}
  virtual ~ActionOperatorCheckerCb() = default;

  // 开始
  bool Start() override {
    if (SuperType::Start()) {
      on_operate_();
      return true;
    }
    return false;
  }
  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    is_done_ = on_check_();
    return SuperType::Update(is_done_ ? delta_time : 0);
  }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override { return is_done_; }

 private:
  FuncOnOperate on_operate_;
  FuncOnCheck on_check_;
  bool is_done_{false};
};
using ActionOperatorCheckerCbPtr =
    boost::intrusive_ptr<ActionOperatorCheckerCb>;

// 动作模板：操作为修改一个值，操作即完成
template <typename OperType>
class ActionOperator : public Action {
  using SuperType = Action;

 public:
  ActionOperator(OperType& oper_ref, const OperType& oper_value)
      : oper_ref_(oper_ref), oper_value_(oper_value) {}
  virtual ~ActionOperator() = default;

  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    oper_ref_ = oper_value_;
    return SuperType::Update(delta_time);
  }

 private:
  OperType& oper_ref_;
  const OperType oper_value_;
};
template <typename OperType>
using ActionOperatorPtr = boost::intrusive_ptr<ActionOperator<OperType>>;

// 动作模板：没有操作，达到某个状态即为完成
template <typename CheckType>
class ActionChecker : public Action {
  using SuperType = Action;

 public:
  ActionChecker(const CheckType& check_ref, const CheckType& check_value)
      : check_ref_(check_ref), check_value_(check_value) {}
  virtual ~ActionChecker() = default;

  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    is_done_ = check_ref_ == check_value_;
    return SuperType::Update(is_done_ ? delta_time : 0);
  }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override { return is_done_; }

 private:
  const CheckType& check_ref_;
  const CheckType check_value_;
  bool is_done_{false};
};
template <typename OperType>
using ActionCheckerPtr = boost::intrusive_ptr<ActionChecker<OperType>>;

// 动作模板：没有操作，达到某个状态即为完成
template <typename CheckType>
class ActionTimedChecker : public Action {
  using SuperType = Action;

 public:
  ActionTimedChecker(double wait_time, const CheckType& check_ref,
                     const CheckType& check_value)
      : wait_time_(wait_time),
        check_ref_(check_ref),
        check_value_(check_value) {}
  virtual ~ActionTimedChecker() = default;

  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    is_done_ = check_ref_ == check_value_;

    delayed_time_ += is_done_ ? 0 : delta_time;
    return SuperType::Update(is_done_ ? delta_time : 0);
  }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override {
    return (delayed_time_ >= wait_time_) || is_done_;
  }

 private:
  const double wait_time_{0.0};
  double delayed_time_{0.0};
  const CheckType& check_ref_;
  const CheckType check_value_;
  bool is_done_{false};
};
template <typename OperType>
using ActionTimedCheckerPtr =
    boost::intrusive_ptr<ActionTimedChecker<OperType>>;

// 动作模板：操作为修改一个值，达到某个状态即为完成
template <typename OperType, typename CheckType>
class ActionOperatorChecker : public Action {
  using SuperType = Action;

 public:
  ActionOperatorChecker(OperType& oper_ref, const OperType& oper_value,
                        const CheckType& check_ref,    // NOLINT
                        const CheckType& check_value)  // NOLINT
      : oper_ref_(oper_ref),
        oper_value_(oper_value),
        check_ref_(check_ref),
        check_value_(check_value) {}
  virtual ~ActionOperatorChecker() = default;

  // 开始
  bool Start() override {
    if (SuperType::Start()) {
      oper_ref_ = oper_value_;
      return true;
    }
    return false;
  }
  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    is_done_ = check_ref_ == check_value_;
    return SuperType::Update(is_done_ ? delta_time : 0);
  }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override { return is_done_; }

 private:
  OperType& oper_ref_;
  const OperType oper_value_;
  const CheckType& check_ref_;
  const CheckType check_value_;
  bool is_done_{false};
};
template <typename OperType, typename CheckType>
using ActionOperatorCheckerPtr =
    boost::intrusive_ptr<ActionOperatorChecker<OperType, CheckType>>;

class ActionWrapper : public Action {
  using SuperType = Action;

 public:
  explicit ActionWrapper(ActionPtr act_ptr) : act_ptr_(act_ptr) {}

  // 开始
  bool Start() override {
    if (SuperType::Start()) {
      act_ptr_->Start();
      return true;
    }
    return false;
  }
  // 恢复
  bool Resume() override {
    if (SuperType::Resume()) {
      act_ptr_->Resume();
      return true;
    }
    return false;
  }
  // 更新
  double Update(double delta_time) override {
    if (!IsRunning()) return 0;
    delta_time = act_ptr_->Update(delta_time);
    return SuperType::Update(delta_time);
  }
  // 暂停
  bool Pause() override {
    if (SuperType::Pause()) {
      act_ptr_->Pause();
      return true;
    }
    return false;
  }
  // 停止
  bool Stop() override {
    if (SuperType::Stop()) {
      act_ptr_->Stop();
      return true;
    }
    return false;
  }

 protected:
  // 当前动作是否执行完毕
  bool IsDone() const override { return act_ptr_->IsFinished(); }

 private:
  ActionPtr act_ptr_;
};
using ActionWrapperPtr = boost::intrusive_ptr<ActionWrapper>;

class ActionNamedWrapper : public ActionWrapper {
  using SuperType = ActionWrapper;

 public:
  ActionNamedWrapper(const std::string name, ActionPtr act_ptr)
      : SuperType(act_ptr), name_(name) {}

  const std::string& GetName() const { return name_; }

 private:
  std::string name_;
};
using ActionNamedWrapperPtr = boost::intrusive_ptr<ActionNamedWrapper>;

class ActionManager {
 public:
  // 执行动作
  void Start(const std::string& slot, const std::string& name,
             ActionPtr act_ptr);
  // 暂停动作
  void Pause(const std::string& slot);
  // 暂停动作
  void PauseAll();
  // 恢复动作
  void Resume(const std::string& slot);
  // 恢复动作
  void ResumeAll();
  // 停止动作
  void Stop(const std::string& slot);
  // 停止动作
  void StopAll();
  // 更新动作
  void Update(double delta_time);
  // 获取当前动作名称
  const std::string& GetActionName(const std::string& slot) const;

 private:
  std::unordered_map<std::string, std::pair<std::string, ActionPtr>> act_map_;
  std::vector<std::pair<std::string, ActionPtr>> update_buffer_;
};
}  // namespace device
#endif  // INCLUDE_COMMON_ACTION_H_
