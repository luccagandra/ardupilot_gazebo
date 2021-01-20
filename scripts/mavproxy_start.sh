#!/bin/bash
# Helper Script for launching MAVProxy with all necessary arguments.

# Exit immediatelly if a command exits with a non-zero status
set -e

# Executes a command when DEBUG signal is emitted in this script - should be after every line
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# Executes a command when ERR signal is emmitted in this script
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

PORT_OUT1=$1
PORT_OUT2=$2
PORT_MASTER=$3
PORT_SITL=$4
ID=$5
STREAMRATE=$6

# Get valid additional arguments
ADDITIONAL_ARGS=""
for arg in "${@:7:99}"
do
  if [[ ! $arg = __* ]]; then
    echo "[mavproxy_start.sh] Found valid argument: $arg"
    ADDITIONAL_ARGS="${ADDITIONAL_ARGS} ${arg}"
  fi
done

if [ -z "${PORT_OUT1}" ] ; then
  PORT_OUT1="14550"
fi

if [ -z "${PORT_OUT2}" ] ; then
  PORT_OUT2="14551"
fi

if [ -z "${PORT_MASTER}" ] ; then
  PORT_MASTER="5760"
fi

if [ -z "${PORT_SITL}" ] ; then
  PORT_OUT2="5501"
fi

if [ -z "${ID}" ] ; then
  ID="1"
fi

if [ -z "${STREAMRATE}" ] ; then
  STREAMRATE="50"
fi

# Single vehicle command
echo "Start single vehicle MAVProxy command"
echo "$HOME/.local/bin/mavproxy.py --mav10 --streamrate ${STREAMRATE} --target-system ${ID} --out 127.0.0.1:${PORT_OUT1} --out 127.0.0.1:${PORT_OUT2} --master tcp:127.0.0.1:${PORT_MASTER} --sitl 127.0.0.1:${PORT_SITL} ${ADDITIONAL_ARGS}"
$HOME/.local/bin/mavproxy.py --mav10 --streamrate ${STREAMRATE} --target-system ${ID} --out 127.0.0.1:${PORT_OUT1} --out 127.0.0.1:${PORT_OUT2} --master tcp:127.0.0.1:${PORT_MASTER} --sitl 127.0.0.1:${PORT_SITL} ${ADDITIONAL_ARGS}
echo "MAVProxy shutdown"