#!/bin/bash

# build.sh build_dir

ZIMA_PACKAGE="base"
ZIMA_PACKAGE_UP="BASE"
PROJECT_DIR=$(pwd)
GIT_HASH_FILE=${PROJECT_DIR}/zima/zima_${ZIMA_PACKAGE}_version_hash.h

# function help() {
help() {
  echo "Usage: Place this script under project root dir and run with './build.sh build_dir'"
}

# function get_git_hash_header() {
get_git_hash_header() {
  rm ${GIT_HASH_FILE}
  GIT_HASH=$(git log -1 --pretty=format:%H)
  echo "Git hash: ${GIT_HASH}"
  echo "// This file should be auto generated, DO NOT MODIFY IT." >> ${GIT_HASH_FILE}
  echo "#ifndef ZIMA_${ZIMA_PACKAGE_UP}_VERSION_HASH_H" >> ${GIT_HASH_FILE}
  echo "#define ZIMA_${ZIMA_PACKAGE_UP}_VERSION_HASH_H" >> ${GIT_HASH_FILE}
  echo "#define ZIMA_${ZIMA_PACKAGE_UP}_VERSION_HASH \"${GIT_HASH}\"" >> ${GIT_HASH_FILE}
  echo "#endif  // ZIMA_${ZIMA_PACKAGE_UP}_VERSION_HASH_H" >> ${GIT_HASH_FILE}
}

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
  echo "Missing argument."
  help
  exit 0
fi

if [ ! -d $1 ]; then
  echo "Build dir $1 does not exist, creating."
  mkdir -p $1
  cd $1
  cmake ${PROJECT_DIR}
else
  cd $1
  if [ $# -eq 2 ] && [ $2 == "clean" ]; then
    rm -rf *
    cmake ${PROJECT_DIR}
  fi
fi

export TZ="Asia/Shanghai"
get_git_hash_header
touch ${PROJECT_DIR}/zima/zima_${ZIMA_PACKAGE}_version.h
make -j$(nproc)
ret=$?
if [ $ret -ne 0 ]; then
  echo "Build failed."
  exit $ret
fi
if [ $# -eq 2 ] && [ $2 == "no-install" ]; then
  echo "Skip installation."
  exit 0
fi
if [ $? == 0 ]; then
  sudo make install -j$(nproc)
fi