load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "save_msg_proto",
    srcs = ["save_msg.proto"]
)



proto_library(
    name = "map_service_msg_proto",
    srcs = ["map_service_msg.proto"],
    deps = [
        "//modules/common_msgs/akman_msgs:occupancy_grid_proto",
    ],
)










apollo_package()
