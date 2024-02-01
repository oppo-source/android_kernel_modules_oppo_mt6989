load("//build/bazel_mgk_rules:mgk_ddk_ko.bzl", "define_mgk_ddk_ko")

bazel_support_platform = "mtk"

def define_oplus_ddk_module(
    name,
    srcs = None,
    header_deps = [],
    ko_deps = [],
    hdrs = None,
    includes = None,
    conditional_srcs = None,
    conditional_defines = None,
    linux_includes = None,
    out = None,
    local_defines = None,
    copts = None):

    flattened_conditional_defines = None
    if conditional_defines:
        for config_vendor, config_defines in conditional_defines.items():
            if config_vendor == bazel_support_platform:
                if flattened_conditional_defines:
                    flattened_conditional_defines = flattened_conditional_defines + config_defines
                else:
                    flattened_conditional_defines = config_defines

    if flattened_conditional_defines:
        if local_defines:
            local_defines =  local_defines + flattened_conditional_defines
        else:
            local_defines = flattened_conditional_defines

    #fail("debug need variable {}".format(local_defines))

    define_mgk_ddk_ko(
        name,
        srcs,
        header_deps,
        ko_deps,
        hdrs,
        includes,
        conditional_srcs,
        linux_includes,
        out,
        local_defines,
        copts
    )

