# Copyright (c) 2026
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

# Locate the HPM SDK. HPM_SDK_BASE must be exported by the environment
# (see sdk_env/hpm_sdk/env.cmd); the SDK toolchain file reads it as well.
find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(pwmv2_cal_shared_delta_app)

sdk_app_src(src/pwmv2_cal_shared_delta_example.c)

generate_ide_projects()