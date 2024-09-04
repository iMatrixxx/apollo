load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_x86_64", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

AIR_VIZ_COPTS = ['-DMODULE_NAME=\\"air_viz\\"',"-std=c++17","-faligned-new",]



apollo_component(
    name = "lib_air_viz.so",
    srcs = ["src/viz.cpp"],
    hdrs = ["include/viz.h"],
    
    linkopts = ["-lGL", "-lGLU", "-lglut"],  # 链接 OpenGL 相关库
    
    copts = AIR_VIZ_COPTS,

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




apollo_package()
cpplint()