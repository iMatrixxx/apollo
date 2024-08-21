load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])


apollo_cc_library(
    name = "akman_utils_lib",
    srcs = ["Quaternion_Solution.cpp"],
    hdrs = ["Quaternion_Solution.h"],
    deps = [
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//cyber"
        ],
)

apollo_cc_binary(
    name = "canbus_tester",
    srcs = ["canbus_tester.cc"],
    deps = [
        "//cyber",
        "//modules/canbus:apollo_canbus",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)


apollo_cc_binary(
    name = "teleop",
    srcs = ["teleop.cc"],
    deps = [
        "//cyber",
        "//modules/canbus:apollo_canbus",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:util_tool",
    ],
)

apollo_package()
cpplint()
