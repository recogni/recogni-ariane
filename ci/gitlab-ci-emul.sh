#!/bin/bash
# This script emulates what the gitlab ci config does (not on public server)
# source this with a bash shell in the project root
# comment out next command if you don't want to use sudo
sudo apt install \
    gcc-7 \
    g++-7 \
    gperf \
    autoconf \
    automake \
    autotools-dev \
    libmpc-dev \
    libmpfr-dev \
    libgmp-dev \
    gawk \
    build-essential \
    bison \
    flex \
    texinfo \
    python-pexpect \
    libusb-1.0-0-dev \
    default-jdk

# customize your paths here
source ci/path-setup.sh

# install the required tools
git submodule update --init --recursive
ci/make-tmp.sh
ci/build-riscv-gcc.sh
ci/install-fesvr.sh
ci/install-verilator.sh
ci/build-riscv-tests.sh
ci/install-dtc.sh
ci/install-spike.sh
ci/get-torture.sh

# clean up and generate randomized test
make clean
make torture-gen

# run asm tests on verilator
make -j${NUM_JOBS} verilate                 verilator=$VERILATOR_ROOT/bin/verilator
make -j${NUM_JOBS} run-asm-tests-verilator  verilator=$VERILATOR_ROOT/bin/verilator
make -j${NUM_JOBS} run-benchmarks-verilator verilator=$VERILATOR_ROOT/bin/verilator
make -j${NUM_JOBS} torture-rtest-verilator  verilator=$VERILATOR_ROOT/bin/verilator

# run asm tests on questa
make -j${NUM_JOBS} build                    questa_version=$QUESTASIM_VERSION
make -j${NUM_JOBS} run-asm-tests            questa_version=$QUESTASIM_VERSION
make -j${NUM_JOBS} run-benchmarks           questa_version=$QUESTASIM_VERSION
make -j${NUM_JOBS} torture-rtest            questa_version=$QUESTASIM_VERSION
