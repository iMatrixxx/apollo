load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

GMAPPING_COPTS = ['-DMODULE_NAME=\\"slam_gmapping\\"']

apollo_component(
    name = "libslam_gmapping_component.so",
    srcs = ["slam_gmapping_component.cc",
            "utils.cc"],
    hdrs = ["slam_gmapping_component.h",
            "utils.h"],
    copts = GMAPPING_COPTS,
    deps = [
        "//cyber",
        "//modules/slam_gmapping/openslam_gmapping:apollo_openslam_gmapping",
        "//modules/common_msgs/akman_msgs:map_meta_data_cc_proto",
        "//modules/common_msgs/akman_msgs:occupancy_grid_cc_proto",
        "//modules/common_msgs/akman_msgs:entropy_cc_proto",
        "//modules/common_msgs/akman_msgs:pose_stamped_cc_proto",
        "//modules/common_msgs/akman_msgs:laser_scan_cc_proto",
        "//third_party/tf2",
        "//modules/transform:apollo_transform",
        "//modules/common/monitor_log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math",
        "//modules/slam_gmapping/common:slam_gmapping_gflags",
        "//modules/common/util:util_lib",
        "//modules/slam_gmapping/proto:slam_gmapping_conf_cc_proto",
        "@eigen",

    ],
)


filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()

cpplint()