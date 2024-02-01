load("//build/kernel/kleaf:kernel.bzl", "ddk_headers")
load("//build/kernel/oplus:oplus_modules_define.bzl", "define_oplus_ddk_module")
load("//build/kernel/oplus:oplus_modules_dist.bzl", "ddk_copy_to_dist_dir")

def define_oplus_local_modules():

    define_oplus_ddk_module(
        name = "oplus_camera_fan53870_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/fan53870-regulator.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_wl2868c_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/wl2868c-regulator.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_wl28681c1_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/wl28681c1-regulator.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_wl28681c2_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/wl28681c2-regulator.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ak7377a",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7377a/ak7377a.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ak7377b",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7377b/ak7377b.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ak7316",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7316/ak7316.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ak7316a",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7316a/ak7316a.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s/dw9800s.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ak7375c",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7375c/ak7375c.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_afsem1217s",
        srcs = native.glob([
            "**/*.h",
            "lens/ois/adaptor-i2c.c",
            "lens/ois/afsem1217s/afsem1217s.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800sw",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800sw/dw9800sw.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw36515",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36515.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_bu24721",
        srcs = native.glob([
            "**/*.h",
            "lens/ois/adaptor-i2c.c",
            "lens/ois/bu24721/bu24721_fw.c",
            "lens/ois/bu24721/ois_bu24721.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_sem1217s",
        srcs = native.glob([
            "**/*.h",
            "lens/ois/adaptor-i2c.c",
            "lens/ois/sem1217s/sem1217s_fw.c",
            "lens/ois/sem1217s/ois_sem1217s.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_temp",
        srcs = native.glob([
            "**/*.h",
            "thermal/temp.c",
        ]),
        includes = ["."],
    )

    ddk_copy_to_dist_dir(
        name = "oplus_camera",
        module_list = [
            "oplus_camera_fan53870_regulator",
            "oplus_camera_wl2868c_regulator",
            "oplus_camera_wl28681c1_regulator",
            "oplus_camera_wl28681c2_regulator",
            "oplus_camera_ak7377a",
            "oplus_camera_ak7377b",
            "oplus_camera_ak7316",
            "oplus_camera_ak7316a",
            "oplus_camera_dw9800s",
            "oplus_camera_ak7375c",
            "oplus_camera_afsem1217s",
            "oplus_camera_dw9800sw",
            "oplus_camera_aw36515",
            "oplus_camera_bu24721",
            "oplus_camera_sem1217s",
            "oplus_camera_temp",
        ],
    )