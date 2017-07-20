PATH=${PATH}:${PWD}/toolchain/bin/
export ARCH=arm64
make CP8676_I02_defconfig ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
make -j30 ARCH=arm64 CROSS_COMPILE=aarch64-linux-android-
