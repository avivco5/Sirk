#!/bin/sh

cd `dirname $0`
exec_dir=`pwd`

#export environment for services
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH

echo "Link path: $LD_LIBRARY_PATH"
${exec_dir}/example/payload ./config/logger.yaml ./config/payload.yaml &
${exec_dir}/example/speaker ./config/logger.yaml ./config/payload.yaml &