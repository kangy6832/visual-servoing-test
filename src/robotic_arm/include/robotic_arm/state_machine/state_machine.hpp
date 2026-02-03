#pragma once

#include <functional>
#include <map>
#include <string>

namespace robotic_arm::state_machine {

class StateMachine {
public:
  using Callback = std::function<void()>;

  void addState(const std::string &name, Callback on_enter, Callback on_tick, Callback on_exit);
  void addTransition(const std::string &from, const std::string &to, std::function<bool()> guard);

  void setInitialState(const std::string &name);
  void tick();

  const std::string &currentState() const;

private:
  struct State {
    Callback on_enter;
    Callback on_tick;
    Callback on_exit;
  };

  struct Transition {
    std::string to;
    std::function<bool()> guard;
  };

  std::map<std::string, State> states_;
  std::multimap<std::string, Transition> transitions_;
  std::string current_state_;
  bool initialized_{false};

  void changeState(const std::string &next_state);
};

}  // namespace robotic_arm::state_machine
