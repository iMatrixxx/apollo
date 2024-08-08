load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_x86_64", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']

apollo_component(
    name = "libcamera_component.so",
    srcs = ["camera_component.cc"],
    hdrs = ["camera_component.h"],
    copts = CAMERA_COPTS,
    linkopts = [
        "-latomic",
    ],
    deps = [
        # ":apollo_camera_util",
        ":apollo_device_listener",
        ":apollo_frame_listener",
        "//modules/drivers/camera/proto:config_cc_proto",
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "@ffmpeg//:avcodec",
        "@ffmpeg//:avformat",
        "@ffmpeg//:swscale",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

# apollo_component(
#     name = "libcamera_compress_component.so",
#     srcs = ["compress_component.cc"],
#     hdrs = ["compress_component.h"],
#     copts = CAMERA_COPTS,
#     deps = [
#         "//cyber",
#         "//modules/common_msgs/basic_msgs:error_code_cc_proto",
#         "//modules/common_msgs/basic_msgs:header_cc_proto",
#         "//modules/drivers/camera/proto:config_cc_proto",
#         "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
#         "@opencv//:core",
#         "@opencv//:highgui",
#         "@opencv//:imgcodecs",
#         "@opencv//:imgproc",
#     ],
# )

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

# apollo_cc_library(
#     name = "apollo_camera_util",
#     srcs = select(
#         {
#             "@platforms//cpu:x86_64": [
#                 "usb_cam.cc",
#                 "util.cc",
#             ],
#             "@platforms//cpu:aarch64": [
#                 "usb_cam.cc",
#             ],
#         },
#         no_match_error = "Please Build with an ARM or Linux x86_64 platform",
#     ),
#     hdrs = select(
#         {
#             "@platforms//cpu:x86_64": [
#                 "usb_cam.h",
#                 "util.h",
#             ],
#             "@platforms//cpu:aarch64": [
#                 "usb_cam.h",
#             ],
#         },
#         no_match_error = "Please Build with an ARM or Linux x86_64 platform",
#     ),
#     deps = [
#         "@boost",
#         "//cyber",
#         "//modules/drivers/camera/proto:config_cc_proto",
#         "@ffmpeg//:avcodec",
#     ] + if_x86_64(
#         ["@adv_plat"],
#     ),
# )

apollo_cc_library(
    name = "apollo_device_listener",
    srcs = ["astracamera/device_listener.cpp"],
    hdrs = ["astracamera/device_listener.h",
            "astracamera/constants.h",
            "astracamera/types.h"],
    copts = CAMERA_COPTS,
    deps = [
        "//cyber",
        "//third_party/camera_library/astracamera",
        "//modules/common/adapters:adapter_gflags"
    ],
)

apollo_cc_library(
    name = "apollo_frame_listener",
    srcs = ["astracamera/ob_frame_listener.cpp"],
    hdrs = ["astracamera/ob_frame_listener.h"],
    copts = CAMERA_COPTS,
    deps = [
        ":apollo_timer_filter",
        "//cyber",
        "//third_party/camera_library/astracamera",
        "//modules/common/adapters:adapter_gflags"
    ],
)
apollo_cc_library(
    name = "apollo_timer_filter",
    srcs = ["astracamera/ob_timer_filter.cpp"],
    hdrs = ["astracamera/ob_timer_filter.h"],
    copts = CAMERA_COPTS,
    deps = [
        ":apollo_device_listener",
        "//cyber",
        "//third_party/camera_library/astracamera",
    ],
)

apollo_package()
cpplint()
