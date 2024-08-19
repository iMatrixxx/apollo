load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_openslam_gmapping",
    srcs =glob([
        # "src/grid/*.cpp*",
        "src/gridfastslam/*.cpp*",
        "src/log/*.cpp*",
        "src/scanmatcher/*.cpp*",
        "src/sensor/sensor_base/*.cpp*",
        "src/sensor/sensor_odometry/*.cpp*",
        "src/sensor/sensor_range/*.cpp*",
        "src/utils/*.cpp*",
        
        
    ]),
    hdrs = glob([
        "include/grid/*.h*",
        "include/gridfastslam/*.h*",
        "include/log/*.h*",
        "include/particlefilter/*.h*",
        "include/scanmatcher/*.h*",
        "include/sensor/sensor_base/*.h*",
        "include/sensor/sensor_odometry/*.h*",
        "include/sensor/sensor_range/*.h*",
        "include/utils/*.h*",
        
    ]),
    deps = [
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ]
)

apollo_package()

cpplint()