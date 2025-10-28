#!/system/bin/sh

cd `dirname $0`
root_path=`pwd`

process_name="payload"
process_info=`ps -A | grep -v grep | grep "$process_name"`

if [ ${#process_info} -eq 0 ]; then
    echo "$process_name not running"
else
    echo "$process_name running"
fi
