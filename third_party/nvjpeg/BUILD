load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data_dest = "3rd-nvjpeg",
    data = [
        ":cyberfile.xml",
        ":3rd-nvjpeg.BUILD",
    ],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-nvjpeg/src",
    filter = "*",
)