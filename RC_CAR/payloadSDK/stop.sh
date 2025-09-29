#!/bin/sh

exec_path="`dirname $0`"

pids=`ps | grep "payload" | grep -v grep | awk '{print $1}'`
if ((${#pids} > 0)); then
  echo "kill process ${pids}"
  echo ${pids} | xargs kill
fi