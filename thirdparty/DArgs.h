#pragma once
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <vector>

namespace DArgs {

class DArgs {
private:
  std::vector<std::string> tokens_;
  std::vector<std::pair<std::string, std::string>> help_;
  bool FAILED_;

  template <class T>
  bool get_args(const std::set<std::string> &keys, T &result) {
    if (tokens_.size() == 0) {
      return false;
    }
    for (size_t i = 0; i < tokens_.size() - 1; ++i) {
      if (keys.count(tokens_[i])) {
        std::stringstream ss(tokens_[i + 1]);
        tokens_.erase(tokens_.begin() + i, tokens_.begin() + i + 2);
        ss >> result;
        return true;
      }
    }
    return false;
  }

public:
  DArgs(int argc, char *argv[]) {
    if (argc > 1) {
      tokens_ = std::vector<std::string>(argv + 1, argv + argc);
    }
    FAILED_ = false;
  }

  template <class T>
  T operator()(const std::string &option_flag, const std::string &desc,
               T default_val) {
    help_.push_back({option_flag, desc});
    T result = default_val;
    get_args({option_flag}, result);
    return result;
  }

  template <class T>
  T operator()(const std::string &option_flag, const std::string &short_flag,
               const std::string &desc, T default_val) {
    std::string help_key = option_flag + ", " + short_flag;
    help_.push_back({help_key, desc});
    T result = default_val;
    get_args({option_flag, short_flag}, result);
    return result;
  }

  template <class T>
  T required(const std::string &option_flag, const std::string &desc) {
    help_.push_back({option_flag, desc + " [Required]"});
    T result;
    FAILED_ |= !get_args({option_flag}, result);
    return result;
  }

  template <class T>
  T required(const std::string &option_flag, const std::string &short_flag,
             const std::string &desc) {
    std::string help_key = option_flag + ", " + short_flag;
    help_.push_back({help_key, desc + " [Required]"});
    T result;
    FAILED_ |= !get_args({option_flag, short_flag}, result);
    return result;
  }

  bool check() const { return tokens_.size() == 0 && !FAILED_; }

  void print_help() const {
    size_t ws = 0;
    for (auto [k, _] : help_) {
      ws = std::max(ws, k.size());
    }
    for (auto [k, v] : help_) {
      std::cout << std::left << std::setw(ws + 1) << k << ": " << v << "\n";
    }
  }
};
template <>
inline bool DArgs::get_args<bool>(const std::set<std::string> &keys, bool &result) {
  if (tokens_.size() == 0) {
    return false;
  }
  for (size_t i = 0; i < tokens_.size(); ++i) {
    if (keys.count(tokens_[i])) {
      if (i == tokens_.size() - 1 ||
          (tokens_[i + 1] != "true" && tokens_[i + 1] != "false")) {
        tokens_.erase(tokens_.begin() + i);
        result = true;
        return true;
      }
      result = tokens_[i + 1] == "true";
      tokens_.erase(tokens_.begin() + i, tokens_.begin() + i + 2);
      return true;
    }
  }
  return false;
}

}; // namespace DArgs
