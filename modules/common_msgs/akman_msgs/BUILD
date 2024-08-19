load("//tools/proto:proto.bzl", "proto_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "map_meta_data_proto",
    srcs = ["map_meta_data.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:geometry_proto",
    ],
)

proto_library(
    name = "occupancy_grid_proto",
    srcs = ["occupancy_grid.proto"],
    deps = [
        ":map_meta_data_proto",
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

proto_library(
    name = "entropy_proto",
    srcs = ["entropy.proto"],
)

proto_library(
    name = "pose_stamped_proto",
    srcs = ["pose_stamped.proto"],
    deps = [
        ":map_meta_data_proto",
        "//modules/common_msgs/basic_msgs:header_proto",
        "//modules/common_msgs/basic_msgs:geometry_proto",
    ]
)

proto_library(
    name = "laser_scan_proto",
    srcs = ["laser_scan.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ]
)

apollo_package()