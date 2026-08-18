// Copyright 2026 Kei Okada
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ENGINEAI_UTIL_HPP_
#define ENGINEAI_UTIL_HPP_

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <functional>

std::string compress_joint_name(const std::string & s)
{
  // split by '_'
  std::stringstream ss(s);
  std::string token;
  std::vector<std::string> parts;
  while (std::getline(ss, token, '_')) {
    parts.push_back(token);
  }

  if (parts.size() < 2) {
    return s;  // could not find '_'
  }

  // Skip prefix(J01) and shrink next toke to 4 letters
  std::string out;
  const std::string& first = parts[1];
  if (!first.empty()) {
    out = first.substr(0, std::min<size_t>(4, first.size()));
  } else {
    out = "";
  }

  // keep first lteeter for each token
  for (size_t i = 2; i < parts.size(); ++i) {
    out.push_back('_');
    if (!parts[i].empty()) {
      out.push_back(parts[i][0]);
    }
  }

  return out;
}

// Create a header line with compressed joint names.
// Example output:
// cmd:   HIP_R   KNEE_R  ...
static std::string make_joint_header_line(
    const std::vector<std::string>& joint_names,
    int width = 9,
    const std::string& prefix = "cmd: ")
{
  std::ostringstream ss;
  ss << prefix;

  for (const auto& name : joint_names) {
    // Print each compressed joint name with fixed column width
    ss << std::setw(width) << compress_joint_name(name);
  }

  return ss.str();
}


// Create a value line for a specific joint field (e.g. "/position").
// Example output:
//        0.123    -0.532   ...
static std::string make_joint_value_line(
    const std::vector<std::string>& joint_names,
    const std::function<double(const std::string&)>& get_command,
    const std::string& suffix,     // e.g. "/position"
    int width = 9,
    int precision = 3,
    const std::string& indent = "     ")
{
  std::ostringstream ss;

  // Apply fixed floating-point formatting
  ss << indent << std::fixed << std::setprecision(precision);

  for (const auto& name : joint_names) {
    // Query value using joint_name + suffix (e.g. "joint/position")
    ss << std::setw(width) << get_command(name + suffix);
  }

  return ss.str();
}

#include <sstream>
#include <unordered_map>

static std::unordered_map<std::string, double>
parse_name_value_csv(const std::string& s)
{
  std::unordered_map<std::string, double> out;
  std::stringstream ss(s);
  std::string item;

  while (std::getline(ss, item, ',')) {
    // trim spaces (simple)
    auto trim = [](std::string& x) {
      while (!x.empty() && std::isspace(x.front())) x.erase(x.begin());
      while (!x.empty() && std::isspace(x.back()))  x.pop_back();
    };
    trim(item);
    if (item.empty()) continue;

    auto pos = item.find('=');
    if (pos == std::string::npos) continue;

    std::string name = item.substr(0, pos);
    std::string val  = item.substr(pos + 1);
    trim(name);
    trim(val);

    try {
      out[name] = std::stod(val);
    } catch (...) {
      // ignore invalid
    }
  }
  return out;
}

[[maybe_unused]] inline double get_double_param(
  const std::unordered_map<std::string, std::string>& p,
  const std::string& key,
  double fallback)
{
  auto it = p.find(key);
  if (it == p.end()) return fallback;
  try { return std::stod(it->second); } catch (...) { return fallback; }
}

static std::string get_string_param(
  const std::unordered_map<std::string, std::string>& p,
  const std::string& key,
  const std::string& fallback = "")
{
  auto it = p.find(key);
  return (it == p.end()) ? fallback : it->second;
}


#endif  // ENGINEAI_UTIL_HPP_
