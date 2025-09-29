#!/bin/bash

SCRIPTDIR=$(cd $(dirname ${BASH_SOURCE[0]}) > /dev/null 2>&1 && pwd)

echo "SCRIPTDIR: ${SCRIPTDIR}"

if [[ $PATH != *"$SCRIPTDIR/bin"* ]]; then
    echo "add '$SCRIPTDIR/bin' to PATH"
    export PATH=$PATH:$SCRIPTDIR/bin:$SCRIPTDIR/tool
fi

if [[ $LD_LIBRARY_PATH != *"$SCRIPTDIR/lib"* ]]; then
    echo "add '$SCRIPTDIR/lib' to LD_LIBRARY_PATH"
    export LD_LIBRARY_PATH=$SCRIPTDIR/lib
fi
