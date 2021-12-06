void ScrubberController::DealMistSprayDeviceStatus() {
  auto& params = dev_ctrl_->GetScrubberParameters();
  auto&& wrapper = dev_ctrl_->CreateCommandWrapper(AutoMode());
  auto& product_info = common::DeviceDB::Instance().GetProductInfo();

  //用户开启喷雾功能
  if (wrapper.mist_spray()) {
    //1.雾化箱低低液位,触发消毒液箱为空检查流程
    if (product_info.IsExistDevice("mist_box_low_low_level_sensor")) {
      if (device_status_map_["mist_box_low_low_level_sensor"] == "1" && !disinfectant_box_empty_check_flag_) {
        GS_INFO("create action:disinfectant_box_empty_check");
        //先停止原来的动作
        pausable_action_executor_.Stop("disinfectant_box_empty_check");

        ActionListPtr act_lst_ptr(new ActionList);
        act_lst_ptr->AddOnEvent(
          [this](const ActionPtr& act_ptr, ActionEvent) { disinfectant_box_empty_check_flag_ = act_ptr->IsRunning(); });

        act_lst_ptr->AddAction(new ActionWaitForTime(params.mist_box_low_low_level_duration()));
        act_lst_ptr->AddAction(new ActionOperatorCb([&] {
          if (device_status_map_["mist_box_low_low_level_sensor"] == "1") {
            GS_INFO("start disinfectant_box_empty");
            pausable_action_executor_.Stop("disinfectant_box_empty");
            // 限制：只有下发喷雾指令时，低低液位返回值才正确，于是对于流程进行以下改动
            // 消毒液空告警在触发低低液位时工作，在补充消毒液到达低液位时恢复
            disinfectant_box_empty_ = true;
            //打开警示灯和蜂鸣器直到消毒液箱非低低液位，或者用户关闭喷雾功能
            ActionSetWaitAnyPtr act_wait_ptr(new ActionSetWaitAny());
            act_wait_ptr->AddOnEvent([this](const ActionPtr& act_ptr, ActionEvent) {
              OperateDevice("mist_box_warn_light", act_ptr->IsRunning() ? "1" : "0");
              OperateDevice("mist_buzzer", act_ptr->IsRunning() ? "1" : "0");
            });
            act_wait_ptr->AddAction(new ActionChecker<bool>(mist_spray_flag_, false));
            act_wait_ptr->AddAction(
              new ActionChecker<std::string>(device_status_map_["mist_box_low_low_level_sensor"], "0"));
            pausable_action_executor_.Start("disinfectant_box_empty", "disinfectant_box_empty", act_wait_ptr);
          }
        }));

        pausable_action_executor_.Start("disinfectant_box_empty_check", "disinfectant_box_empty_check", act_lst_ptr);
      }
    }
    //2.雾化箱低液位触发补液流程
    if (product_info.IsExistDevice("mist_box_low_level_sensor") &&
        product_info.IsExistDevice("mist_box_high_level_sensor")) {
      if (disinfectant_box_empty_ && device_status_map_["mist_box_low_level_sensor"] == "0") {
        disinfectant_box_empty_ = false;
      }

      if (device_status_map_["mist_box_low_level_sensor"] == "1" && !mist_spray_supply_flag_) {
        //先停止原来的动作
        GS_INFO("start mist_spray_supply_action");
        pausable_action_executor_.Stop("mist_spray_supply_action");

        //打开消毒剂电机,直到补到雾化箱高液位，或者消毒液箱缺液体，或者用户关闭喷雾功能
        ActionSetWaitAnyPtr act_wait_ptr(new ActionSetWaitAny());
        act_wait_ptr->AddOnEvent([this](const ActionPtr& act_ptr, ActionEvent) {
          OperateDevice("disinfectant_motor", act_ptr->IsRunning() ? "1" : "0");
          mist_spray_supply_flag_ = act_ptr->IsRunning();
        });
        //act_wait_ptr->AddAction(new ActionChecker<bool>(mist_spray_flag_, false));
        act_wait_ptr->AddAction(new ActionChecker<std::string>(device_status_map_["mist_box_high_level_sensor"], "1"));
        act_wait_ptr->AddAction(new ActionTimedChecker<bool>(100.0, mist_spray_flag_, false));

        pausable_action_executor_.Start("mist_spray_supply_action", "mist_spray_supply_action", act_wait_ptr);
      }
    }
  }
}
