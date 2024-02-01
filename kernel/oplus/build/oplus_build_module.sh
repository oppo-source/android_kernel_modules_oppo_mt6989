#!/bin/bash

source kernel/oplus/build/oplus_setup.sh $1
init_build_environment

if [ "$2" = "" ]; then
    echo "Please enter the ko directory:"
    print_module_help
    read ko_path
    echo
else
    ko_path=$2
fi

function build_in_tree_ko () {
    echo "enter build_in_tree_ko"
    cd ${TOPDIR}/kernel
    build_target=$(./tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} query //kernel_device_modules-${VERSION}:mgk_64_k61_modules.${variants_type}/${ko_path})
    if [ -n "$build_target" ]
    then
        echo "find the ko successfully!!!"
        tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} build //kernel_device_modules-${VERSION}:mgk_64_k61_modules.${variants_type}/${ko_path}
    else
        echo "the directory you input is not exit or not right, please exit and try again!!!"
    fi
    echo "exit build_in_tree_ko"
}

function build_out_of_tree_ko () {
    echo "enter build_out_of_tree_ko"
    cd ${TOPDIR}/kernel
    build_target=$(./tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} query //${ko_path}.mgk_64_k61.6.1.${variants_type})
    if [ -n "$build_target" ]
    then
        echo "find the ko successfully!!!"
        tools/bazel --output_root=${KLEAF_OBJ} --output_base=${OUTPUT_BASE} build --//build/bazel_mgk_rules:kernel_version=${VERSION} //${ko_path}.mgk_64_k61.6.1.${variants_type}
    else
        echo "the directory you input is not exit or not right, please exit and try again!!!"
    fi
    echo "exit build_out_of_tree_ko"
}

function build_ko {
    if [[ "$ko_path" == *".ko" ]]; then
        echo "-------------------------this is in tree ko-----------------------"
        build_in_tree_ko
    elif [[ "$ko_path" == *":"* ]]; then
        echo "-------------------------this is out of tree ko-------------------"
        build_out_of_tree_ko
    else
        echo "[ERROR]: Invalid parameter, please provide a path to a .ko file or a directory!!!"
        print_module_help
        exit 1
    fi
}

#print_module_help
build_ko
