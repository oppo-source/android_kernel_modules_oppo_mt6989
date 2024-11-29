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
        name = "oplus_camera_dio8018wl20_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/dio8018wl20-regulator.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw37004_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/aw37004-regulator.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_wl2866d_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/wl2866d-regulator.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw37004dnr_regulator",
        srcs = native.glob([
            "**/*.h",
            "regulator/aw37004dnr-regulator.c",
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
        name = "oplus_camera_dw9827c_23265",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9827c_23265/dw9827c_23265.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_23265_front",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_23265_front/dw9800s_23265_front.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_23265_tele",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_23265_tele/dw9800s_23265_tele.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_24678",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_24678/dw9800s_24678.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_24706",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_24706/dw9800s_24706.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_jd5516w_24706_front",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/jd5516w_24706_front/jd5516w_24706_front.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_gt9772b_24703",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/gt9772b_24703/gt9772b_24703.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_24051",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_24051/dw9800s_24051.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9827c_23261",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9827c_23261/dw9827c_23261.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_23261_front",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_23261_front/dw9800s_23261_front.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_jd5516w_23261_front2",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/jd5516w_23261_front2/jd5516w_23261_front2.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_gt9779_24921",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/gt9779_24921/gt9779_24921.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_jd5516w_24021",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/jd5516w_24021/jd5516w_24021.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_jd5516w_24708",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/jd5516w_24708/jd5516w_24708.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_24021",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_24021/dw9800s_24021.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_ak7316_24021",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/ak7316_24021/ak7316_24021.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_gt9764v_24021",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/gt9764v_24021/gt9764v_24021.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9827c_24023",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9827c_24023/dw9827c_24023.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_24023",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_24023/dw9800s_24023.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_gt9764v_24023",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/gt9764v_24023/gt9764v_24023.c",
        ]),
        includes = ["."],
    )
    define_oplus_ddk_module(
        name = "oplus_camera_dw9800s_24610",
        srcs = native.glob([
            "**/*.h",
            "lens/vcm/v4l2/dw9800s_24610/dw9800s_24610.c",
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
        name = "oplus_camera_aw36515_omegas2",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36515_omegas2.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw36515_omegas3",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36515_omegas3.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_sy7806e_alphah",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/sy7806e_alphah.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw36515_caymana",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36515_caymana.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw36515_caymanb",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36515_caymanb.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw36410_casio",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36410_casio.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "flashlights_dunhuang",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/flashlights_dunhuang.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw36515_milkywayc1",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36515_milkywayc1.c",
        ]),
        includes = ["."],
    )
    
    define_oplus_ddk_module(
        name = "oplus_camera_sy7806e_milkywayc2",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/sy7806e_milkywayc2.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw36515_casiox",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36515_casiox.c",
        ]),
        includes = ["."],
    )

    define_oplus_ddk_module(
        name = "oplus_camera_aw36515_2led_casiox",
        srcs = native.glob([
            "**/*.h",
            "flashlight/v4l2/aw36515_2led_casiox.c",
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
            "oplus_camera_dio8018wl20_regulator",
            "oplus_camera_wl2866d_regulator",
            "oplus_camera_aw37004dnr_regulator",
            "oplus_camera_aw37004_regulator",
            "oplus_camera_ak7377a",
            "oplus_camera_ak7377b",
            "oplus_camera_ak7316",
            "oplus_camera_ak7316a",
            "oplus_camera_dw9800s_24610",
            "oplus_camera_dw9800s",
            "oplus_camera_ak7375c",
            "oplus_camera_afsem1217s",
            "oplus_camera_dw9800sw",
            "oplus_camera_aw36515",
            "oplus_camera_bu24721",
            "oplus_camera_sem1217s",
            "oplus_camera_gt9779_24921",
            "oplus_camera_jd5516w_24708",
            "oplus_camera_temp",
        ],
    )
