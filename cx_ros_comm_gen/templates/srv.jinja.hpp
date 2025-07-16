// Copyright (c) 2024-2025 Carologistics
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

/***************************************************************************
 *  {{name_camel}}.hpp
 *
 *  Automatically Generated: {{ gen_date }}
 ****************************************************************************/

// clang-format off
#ifndef CX_PLUGINS__{{name_upper}}_HPP_
#define CX_PLUGINS__{{name_upper}}_HPP_

#include <map>
#include <memory>
#include <string>

#include "cx_plugin/clips_plugin.hpp"

#include "rclcpp/rclcpp.hpp"
#include {{message_include_path}}

namespace cx {

class {{name_camel}} : public ClipsPlugin, public rclcpp::Node {
public:
  {{name_camel}}();
  ~{{name_camel}}();

  void initialize() override;
  void finalize() override;

  bool clips_env_init(std::shared_ptr<clips::Environment> &env) override;
  bool clips_env_destroyed(std::shared_ptr<clips::Environment> &env) override;

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  std::unique_ptr<rclcpp::Logger> logger_;

  std::mutex map_mtx_;

  bool stop_flag_;

  std::map<std::string,
           std::map<std::string,
                    rclcpp::Service<{{message_type}}>::SharedPtr>>
      services_;
  std::map<std::string,
           std::map<std::string,
                    rclcpp::Client<{{message_type}}>::SharedPtr>>
      clients_;
  std::unordered_map<void*, std::shared_ptr<{{message_type}}::Request>> requests_;
  std::unordered_map<void*, std::shared_ptr<{{message_type}}::Response>> responses_;

  std::unordered_set<std::string> function_names_;

{% set template_part = "declaration" %}
{% set template_type = "Request" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}
{% set template_type = "Response" %}
{% include 'get_field.jinja.cpp' with context %}
{% include 'set_field.jinja.cpp' with context %}
{% include 'create.jinja.cpp' with context %}
{% include 'destroy.jinja.cpp' with context %}

  void send_request(clips::Environment *env, {{message_type}}::Request *msg, const std::string &service_name);

  void create_new_client(clips::Environment *env, const std::string &service_name);

  void destroy_client(clips::Environment *env, const std::string &service_name);

  void create_new_service(clips::Environment *env, const std::string &service_name);

  void destroy_service(clips::Environment *env, const std::string &service_name);

  void service_callback(const std::shared_ptr<{{message_type}}::Request> request,
                        std::shared_ptr<{{message_type}}::Response> response,
                        std::string service_name, clips::Environment *env);
};

} // namespace cx
#endif // !CX_PLUGINS__{{name_upper}}_HPP_
