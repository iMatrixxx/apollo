load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
# load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "astracamera",
    srcs = glob(["lib_x64/*.so*", "lib_x64/OpenNI2/Drivers/*.so*"]),
    hdrs = glob(["include/*.h", "include/Driver/*.h", "include/Linux-x86/*.h*"]),
    copts = [
        "-Iinclude",
    ],
    includes = [
        "include",
    ],
    linkopts = [
        "-Llib_x64",
    ],
    visibility = ["//visibility:public"],
)

install_src_files(
    name = "lib",
    src_dir = ["lib_x64", "lib_x64/OpenNI2/Drivers"],
    dest = "3rd-camera-library/lib",
    filter = "*.so*",
)

install_src_files(
    name = "headers",
    src_dir = ["include", "include/Driver", "include/Linux-x86"],
    dest = "3rd-camera-library/include",
    filter = "*",
)
# apollo_package()