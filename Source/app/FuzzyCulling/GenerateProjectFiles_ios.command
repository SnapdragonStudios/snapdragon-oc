#/bin/bash
echo "Generate Project Files for iOS"

rm build/CMakeCache.txt

ROOT_DIR="$PWD"

#build diretory
BUILD_DIR="/build"
BUILD_ABS_DIR=$ROOT_DIR/$BUILD_DIR
if [ -d $BUILD_DIR ]
then
    echo "Building folder exists"
else
    mkdir $BUILD_ABS_DIR
    echo "Create building folder"
fi

IOS_TOOLCHAIN_PATH="submodules/ios-cmake/ios.toolchain.cmake"
IOS_TOOLCHAIN_ABS_PATH="$ROOT_DIR/$IOS_TOOLCHAIN_PATH"

pushd $BUILD_ABS_DIR

# find ios toolchain
if [ -f $IOS_TOOLCHAIN_ABS_PATH ]
then
    echo "iOS Toolchain found"
else
    echo "iOS Toolchain not found"
    exit 1
fi

TARGET_PLATFORM="OS64"
if [ -n $1 ]
then
    if [ "$1" = "MAC" ]
    then
        TARGET_PLATFORM="MAC"
    fi    
fi

CMAKE_CMD="cmake $ROOT_DIR -G Xcode -DCMAKE_TOOLCHAIN_FILE=$IOS_TOOLCHAIN_ABS_PATH -DPLATFORM=$TARGET_PLATFORM"
echo $CMAKE_CMD
eval $CMAKE_CMD

popd $BUILD_ABS_DIR
