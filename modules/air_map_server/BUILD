load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_x86_64", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

MAPSERVER_COPTS = ['-DMODULE_NAME=\\"air_map_server\\"']


apollo_cc_library(
    name = "lib_air_map_mode.so",
    srcs = ["src/map_mode.cpp"],
    hdrs = ["include/map_mode.h"],
    copts = MAPSERVER_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/akman_msgs:map_meta_data_cc_proto",
        "//modules/common_msgs/akman_msgs:occupancy_grid_cc_proto",
        "//modules/common_msgs/akman_msgs:entropy_cc_proto",
        "//modules/common_msgs/akman_msgs:pose_stamped_cc_proto",
        "//modules/common_msgs/akman_msgs:laser_scan_cc_proto",
        "//modules/air_map_server/proto:save_msg_proto",
        "//modules/air_map_server/proto:map_service_msg_proto",
        "//third_party/tf2",
        "//modules/transform:apollo_transform",
        "//modules/common/monitor_log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math",
        "//modules/slam_gmapping/common:slam_gmapping_gflags",
        "//modules/common/util:util_lib",
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)



apollo_cc_library(
    name = "lib_air_map_io.so",
    srcs = ["src/map_io.cpp"],
    hdrs = ["include/map_io.h","include/map_mode.h"],
    copts = MAPSERVER_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/akman_msgs:map_meta_data_cc_proto",
        "//modules/common_msgs/akman_msgs:occupancy_grid_cc_proto",
        "//modules/common_msgs/akman_msgs:entropy_cc_proto",
        "//modules/common_msgs/akman_msgs:pose_stamped_cc_proto",
        "//modules/common_msgs/akman_msgs:laser_scan_cc_proto",
        "//modules/air_map_server/proto:save_msg_proto",
        "//modules/air_map_server/proto:map_service_msg_proto",
        "//third_party/tf2",
        "//modules/transform:apollo_transform",
        "//modules/common/monitor_log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math",
        "//modules/slam_gmapping/common:slam_gmapping_gflags",
        "//modules/common/util:util_lib",
        "//modules/slam_gmapping/proto:slam_gmapping_conf_cc_proto",

        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",

    ],
)



apollo_component(
    name = "lib_air_map_saver.so",
    srcs = ["src/map_saver/map_saver.cpp","src/map_io.cpp","src/map_mode.cpp"],
    hdrs = ["include/map_saver.h","include/map_io.h","include/map_mode.h"],
    copts = MAPSERVER_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/akman_msgs:map_meta_data_cc_proto",
        "//modules/common_msgs/akman_msgs:occupancy_grid_cc_proto",
        "//modules/common_msgs/akman_msgs:entropy_cc_proto",
        "//modules/common_msgs/akman_msgs:pose_stamped_cc_proto",
        "//modules/common_msgs/akman_msgs:laser_scan_cc_proto",
        "//modules/air_map_server/proto:save_msg_proto",
        "//modules/air_map_server/proto:map_service_msg_proto",

        "//third_party/tf2",
        "//modules/transform:apollo_transform",
        "//modules/common/monitor_log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math",
        "//modules/slam_gmapping/common:slam_gmapping_gflags",
        "//modules/common/util:util_lib",
        "//modules/slam_gmapping/proto:slam_gmapping_conf_cc_proto",

        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    
    ],
)



apollo_component(
    name = "lib_air_map_server.so",
    srcs = ["src/map_server/map_server.cpp",
            "src/map_io.cpp",
            "src/map_mode.cpp"],

    hdrs = ["include/map_server.h",
            "include/map_io.h",
            "include/map_mode.h", 
            ],

    copts = MAPSERVER_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/akman_msgs:map_meta_data_cc_proto",
        "//modules/common_msgs/akman_msgs:occupancy_grid_cc_proto",
        "//modules/common_msgs/akman_msgs:entropy_cc_proto",
        "//modules/common_msgs/akman_msgs:pose_stamped_cc_proto",
        "//modules/common_msgs/akman_msgs:laser_scan_cc_proto",
        "//modules/air_map_server/proto:save_msg_proto",
        "//modules/air_map_server/proto:map_service_msg_proto",

        "//third_party/tf2",
        "//modules/transform:apollo_transform",
        "//modules/common/monitor_log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math",
        "//modules/slam_gmapping/common:slam_gmapping_gflags",
        "//modules/common/util:util_lib",
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)



cc_binary(
    name = "map_saver_cli",
    srcs = ["src/map_saver/map_saver_cli.cpp","include/map_io.h","include/map_mode.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/akman_msgs:map_meta_data_cc_proto",
        "//modules/common_msgs/akman_msgs:occupancy_grid_cc_proto",
        "//modules/common_msgs/akman_msgs:entropy_cc_proto",
        "//modules/common_msgs/akman_msgs:pose_stamped_cc_proto",
        "//modules/common_msgs/akman_msgs:laser_scan_cc_proto",
        "//modules/air_map_server/proto:save_msg_proto",
        "//modules/air_map_server/proto:map_service_msg_proto",

        "//third_party/tf2",
        "//modules/transform:apollo_transform",
        "//modules/common/monitor_log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math",
        "//modules/slam_gmapping/common:slam_gmapping_gflags",
        "//modules/common/util:util_lib",
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)



cc_binary(
    name = "test",
    srcs = ["src/test.cc","include/map_io.h","include/map_mode.h","include/map_server.h"],
    copts = MAPSERVER_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/akman_msgs:map_meta_data_cc_proto",
        "//modules/common_msgs/akman_msgs:occupancy_grid_cc_proto",
        "//modules/common_msgs/akman_msgs:entropy_cc_proto",
        "//modules/common_msgs/akman_msgs:pose_stamped_cc_proto",
        "//modules/common_msgs/akman_msgs:laser_scan_cc_proto",
        "//modules/air_map_server/proto:save_msg_proto",
        "//modules/air_map_server/proto:map_service_msg_proto",
        "//third_party/tf2",
        "//modules/transform:apollo_transform",
        "//modules/common/monitor_log",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math",
        "//modules/slam_gmapping/common:slam_gmapping_gflags",
        "//modules/common/util:util_lib",

        "//modules/air_map_server:lib_air_map_io.so",
        "//modules/air_map_server:lib_air_map_mode.so",
        "//modules/air_map_server:lib_air_map_server.so",
        # "//modules/air_map_server:lib_air_map_saver.so",

        "@eigen",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)


apollo_package()
cpplint()