#/bin/bash
echo "Generate Project Files for Mac"
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


pushd $BUILD_ABS_DIR

    
TARGET_PLATFORM="MAC"

CMAKE_CMD="cmake $ROOT_DIR -G Xcode"
echo $CMAKE_CMD
eval $CMAKE_CMD

popd $BUILD_ABS_DIR
