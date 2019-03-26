load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "s32_can_client_raw",
    srcs = [
        "s32_can_client_raw.cc",
    ],
    hdrs = [
        "s32_can_client_raw.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_client",
    ],
)

cc_test(
    name = "s32_can_client_raw_test",
    size = "small",
    srcs = [
        "s32_can_client_raw_test.cc",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/canbus/can_client/s32_can:s32_can_client_raw",
        "//modules/drivers/canbus/common:canbus_common",
        "@gtest//:main",
    ],
)

cpplint()
