#include "robotic_arm/state_machine/state_machine.hpp"

#include <stdexcept>

namespace robotic_arm::state_machine {

void StateMachine::addState(const std::string &name, Callback on_enter, Callback on_tick, Callback on_exit) {
  states_[name] = State{std::move(on_enter), std::move(on_tick), std::move(on_exit)};
}

void StateMachine::addTransition(const std::string &from, const std::string &to, std::function<bool()> guard) {
  transitions_.insert({from, Transition{to, std::move(guard)}});
}

void StateMachine::setInitialState(const std::string &name) {
  if (states_.find(name) == states_.end()) {
    throw std::runtime_error("Initial state not found.");
  }
  current_state_ = name;
  initialized_ = false;
}

void StateMachine::tick() {
  if (current_state_.empty()) {
    throw std::runtime_error("State machine has no initial state.");
  }
  if (!initialized_) {
    auto &state = states_.at(current_state_);
    if (state.on_enter) {
      state.on_enter();
    }
    initialized_ = true;
  }

  auto &state = states_.at(current_state_);
  if (state.on_tick) {
    state.on_tick();
  }

  auto range = transitions_.equal_range(current_state_);
  for (auto it = range.first; it != range.second; ++it) {
    if (it->second.guard && it->second.guard()) {
      changeState(it->second.to);
      break;
    }
  }
}

const std::string &StateMachine::currentState() const { return current_state_; }

void StateMachine::changeState(const std::string &next_state) {
  if (states_.find(next_state) == states_.end()) {
    throw std::runtime_error("Next state not found.");
  }
  auto &current = states_.at(current_state_);
  if (current.on_exit) {
    current.on_exit();
  }
  current_state_ = next_state;
  initialized_ = false;
}

}  // namespace robotic_arm::state_machine
