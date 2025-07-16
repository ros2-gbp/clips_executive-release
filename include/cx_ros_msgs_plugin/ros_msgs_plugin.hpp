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

#ifndef CX_PLUGINS__ROS_MSGS_PLUGIN_HPP_
#define CX_PLUGINS__ROS_MSGS_PLUGIN_HPP_

#include <string>

#include "cx_plugin/clips_plugin.hpp"
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <rclcpp/generic_client.hpp>

namespace cx {

class RosMsgsPlugin : public ClipsPlugin {
public:
  RosMsgsPlugin();
  ~RosMsgsPlugin();

  void initialize();
  void finalize();

  bool clips_env_init(std::shared_ptr<clips::Environment> &env) override;
  bool clips_env_destroyed(std::shared_ptr<clips::Environment> &env) override;

private:
  struct MessageInfo {
    std::shared_ptr<void> external_created_ptr;
    void *msg_ptr;
    const rosidl_typesupport_introspection_cpp::MessageMembers *members;
    bool is_sub_msg;
    rcutils_allocator_t allocator;

    MessageInfo(
        const rosidl_typesupport_introspection_cpp::MessageMembers *members,
        void *parent_ptr = nullptr)
        : msg_ptr(nullptr), members(members) {
      is_sub_msg = (parent_ptr != nullptr);
      if (parent_ptr) {
        msg_ptr = parent_ptr;
      } else {
        // Allocate memory for the message
        allocator = rcutils_get_default_allocator();
        msg_ptr =
            allocator.zero_allocate(1, members->size_of_, allocator.state);
        if (members->init_function) {
          members->init_function(
              msg_ptr,
              rosidl_runtime_cpp::MessageInitialization::ALL); // Full
                                                               // initialization
        }
      }
    }
    MessageInfo(
        const rosidl_typesupport_introspection_cpp::MessageMembers *members,
        std::shared_ptr<void> external_created_ptr)
        : external_created_ptr(external_created_ptr),
          msg_ptr(external_created_ptr.get()), members(members) {
      is_sub_msg = true;
    }

    ~MessageInfo() {
      if (!is_sub_msg) {
        // Finalize the message and free memory
        if (members && members->fini_function) {
          members->fini_function(msg_ptr);
        }
        allocator.deallocate(msg_ptr, allocator.state);
      }
    }

    // Disable copy constructor and copy assignment to prevent double freeing
    MessageInfo(const MessageInfo &) = delete;
    MessageInfo &operator=(const MessageInfo &) = delete;

    // Enable move constructor and move assignment for efficiency
    MessageInfo(MessageInfo &&other) noexcept
        : external_created_ptr(other.external_created_ptr),
          msg_ptr(other.msg_ptr), members(other.members),
          is_sub_msg(other.is_sub_msg), allocator(other.allocator) {
      RCLCPP_WARN(rclcpp::get_logger("MESSAGE INFO"), "ASSIGN");
      other.msg_ptr = nullptr;
      other.external_created_ptr.reset();
      other.members = nullptr;
      other.is_sub_msg = false;
    }
  };
  struct MessageInfoHasher {
    std::size_t operator()(const MessageInfo &key) const {
      return std::hash<void *>()(key.external_created_ptr.get()) ^
             std::hash<void *>()(key.msg_ptr) ^
             std::hash<const rosidl_typesupport_introspection_cpp::
                           MessageMembers *>()(key.members);
    }
  };

  clips::UDFValue create_message(clips::Environment *env,
                                 const std::string &type);
  void destroy_msg(void *msg);
  clips::UDFValue create_request(clips::Environment *env,
                                 const std::string &service_type);

  clips::UDFValue ros_msg_member_to_udf_value(
      clips::Environment *env, std::shared_ptr<MessageInfo> &msg_info,
      const rosidl_typesupport_introspection_cpp::MessageMember &member);
  std::shared_ptr<MessageInfo> process_nested_msg(
      void *nested_msg,
      const rosidl_typesupport_introspection_cpp::MessageMember &member);
  clips::CLIPSValue ros_to_clips_value(clips::Environment *env, void *val,
                                       uint8_t ros_type);

  clips::UDFValue get_field(clips::Environment *env, void *deserialized_msg,
                            const std::string &field, clips::UDFContext *udfc);

  void udf_value_to_ros_message_member(
      clips::Environment *env, void *deserialized_msg,
      const rosidl_typesupport_introspection_cpp::MessageMember &member,
      clips::UDFValue &val);

  void clips_to_ros_value(
      clips::Environment *env, const clips::CLIPSValue &val,
      const rosidl_typesupport_introspection_cpp::MessageMember &member,
      uint8_t *field_ptr);

  void set_field(clips::Environment *env, void *deserialized_msg,
                 const std::string &field, clips::UDFValue &val,
                 clips::UDFContext *udfc);

  //// Helper function to manage subscriptions
  void subscribe_to_topic(clips::Environment *env,
                          const std::string &topic_name,
                          const std::string &topic_type);

  std::shared_ptr<RosMsgsPlugin::MessageInfo>
  create_deserialized_msg(const std::string &topic_type);

  void topic_callback(std::shared_ptr<const rclcpp::SerializedMessage> msg,
                      const std::string &topic_name,
                      const std::string &topic_type, clips::Environment *env);
  void unsubscribe_from_topic(clips::Environment *env,
                              const std::string &topic_name);

  void create_new_publisher(clips::Environment *env,
                            const std::string &topic_name,
                            const std::string &topic_type);
  void destroy_publisher(clips::Environment *env,
                         const std::string &topic_name);
  void publish_to_topic(clips::Environment *env, void *deserialized_msg,
                        const std::string &topic_name);

  void create_new_client(clips::Environment *env,
                         const std::string &service_name,
                         const std::string &service_type);
  void destroy_client(clips::Environment *env, const std::string &service_name);
  clips::UDFValue send_request(clips::Environment *env, void *deserialized_msg,
                               const std::string &service_name);

  std::string get_msg_type(
      const rosidl_typesupport_introspection_cpp::MessageMembers *members);
  const rosidl_typesupport_introspection_cpp::MessageMembers *
  get_msg_members(const std::string &members);

  std::shared_ptr<MessageInfo>
  deserialize_msg(std::shared_ptr<const rclcpp::SerializedMessage> msg,
                  const std::string &msg_type);

  rclcpp::SerializedMessage serialize_msg(std::shared_ptr<MessageInfo> msg_info,
                                          const std::string &msg_type);

  void move_field_to_parent(
      void *parent_msg,
      const rosidl_typesupport_introspection_cpp::MessageMember *parent_member,
      void *source_msg);

  std::unique_ptr<rclcpp::Logger> logger_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;

  std::unordered_set<std::string> function_names_;

  // env -> (topic_name -> subscription)
  std::map<std::string,
           std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>>>
      subscriptions_;

  // env -> (topic_name -> publisher)
  std::map<std::string,
           std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>>>
      publishers_;

  std::unordered_map<std::string, std::shared_ptr<rcpputils::SharedLibrary>>
      libs_;

  std::mutex map_mtx_;
  bool stop_flag_;
  // MessageInfo* -> shared_ptr holding the MessageInfo*
  std::unordered_map<void *, std::shared_ptr<MessageInfo>> messages_;
  // parent msg MessageInfo* -> nested msg MessageInfo*
  std::unordered_map<void *, std::vector<std::shared_ptr<MessageInfo>>>
      sub_messages_;
  // message_type -> message_info
  std::unordered_map<std::string, const rosidl_message_type_support_t *>
      type_support_cache_;

  // env -> (service_name -> client)
  std::map<std::string,
           std::map<std::string, std::shared_ptr<rclcpp::GenericClient>>>
      clients_;
  // env -> (service_name -> service_type)
  std::map<std::string, std::unordered_map<std::string, std::string>>
      client_types_;
  std::unordered_map<std::string, const rosidl_service_type_support_t *>
      service_type_support_cache_;
};
} // namespace cx

#endif // !CX_PLUGINS__ROS_MSGS_PLUGIN_HPP_
