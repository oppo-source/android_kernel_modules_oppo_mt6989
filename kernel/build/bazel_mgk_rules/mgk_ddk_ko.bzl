load(
    "//build/kernel/kleaf:kernel.bzl",
    "ddk_module",
)
load(
    ":mgk.bzl",
    "kernel_versions_and_projects",
)

def define_mgk_ddk_ko(
    name,
    srcs = None,
    header_deps = [],
    ko_deps = [],
    hdrs = None,
    includes = None,
    conditional_srcs = None,
    linux_includes = None,
    out = None,
    local_defines = None,
    copts = None):

    if srcs == None:
        srcs = native.glob(
            [
                "**/*.c",
                "**/*.h",
            ],
            exclude = [
                ".*",
                ".*/**",
            ],
        )
    if out == None:
        out = name + ".ko"
    for version,projects in kernel_versions_and_projects.items():
        for project in projects.split(" "):
            for build in ["eng", "userdebug", "user", "ack"]:
                ddk_module(
                    name = "{}.{}.{}.{}".format(name, project, version, build),
                    kernel_build = "//kernel_device_modules-{}:{}.{}".format(version, project, build),
                    srcs = srcs,
                    deps = [
                        "//kernel_device_modules-{}:{}_modules.{}".format(version, project, build),
                        "//kernel-{}:all_headers".format(version),
                        "//kernel_device_modules-{}:all_mgk_headers".format(version),
                    ] + ["{}.{}.{}.{}".format(m, project, version, build) for m in ko_deps] + header_deps,
                    hdrs = hdrs,
                    includes = includes,
                    conditional_srcs = conditional_srcs,
                    linux_includes = linux_includes,
                    out = out,
                    local_defines = local_defines,
                    copts = copts,
                )

