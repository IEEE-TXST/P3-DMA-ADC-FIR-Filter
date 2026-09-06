#!/bin/sh
cmake -DCMAKE_TOOLCHAIN_FILE="../../../../../SDK_2_2_0_FRDM-KL26Z/tools/cmake_toolchain_files/armgcc.cmake" -G "Unix Makefiles" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Debug  .
make -j4
cmake -DCMAKE_TOOLCHAIN_FILE="../../../../../SDK_2_2_0_FRDM-KL26Z/tools/cmake_toolchain_files/armgcc.cmake" -G "Unix Makefiles" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release  .
make -j4
