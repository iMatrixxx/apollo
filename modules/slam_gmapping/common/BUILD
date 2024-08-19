load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"slam_gmapping\\"']

apollo_cc_library(
    name = "slam_gmapping_gflags",
    srcs = ["slam_gmapping_gflags.cc"],
    hdrs = ["slam_gmapping_gflags.h"],
    deps = ["@com_github_gflags_gflags//:gflags"],
)

apollo_package()

cpplint()
