#!/bin/bash
# Script for launch sim_vehicle.py with all necessary arguments.

set -o errexit 

# Catch all the bash script arguments
VEHICLE_NAME=$1
VEHICLE_ID=$2
PARAM_PATH=$3
ENABLE_CONSOLE=$4
ENABLE_MAP=$5

if [ -z "${VEHICLE_NAME}" ] ; then
  VEHICLE_NAME="default"
fi

if [ -z "${VEHICLE_ID}" ] ; then
  VEHICLE_ID="1"
fi

if [ -z "${ENABLE_CONSOLE}" ] ; then
  ENABLE_CONSOLE="true"
fi

if [ -z "${ENABLE_MAP}" ] ; then
  ENABLE_MAP="false"
fi

# Navigate to the startup folder
STARTUP_DIR="$HOME/Documents/${VEHICLE_NAME}_${VEHICLE_ID}_startup"
mkdir -p $STARTUP_DIR
cd $STARTUP_DIR
echo $STARTUP_DIR

SIM_VEHICLE_ARGS=""
if [ "$ENABLE_CONSOLE" = true ] ; then
    SIM_VEHICLE_ARGS="${SIM_VEHICLE_ARGS} --console"
fi
if [ "$ENABLE_MAP" = true ] ; then
    SIM_VEHICLE_ARGS="${SIM_VEHICLE_ARGS} --map"
fi

IDENTITY_PATH=${STARTUP_DIR}/identity.parm
cat <<EOF > ${STARTUP_DIR}/identity.parm
SYSID_THISMAV ${VEHICLE_ID}
EOF

cat ${IDENTITY_PATH} >> ${PARAM_PATH}
echo "sim_vehicle.py -v ArduCopter --add-param-file=${PARAM_PATH} -f gazebo-iris -I$((${VEHICLE_ID} - 1)) -m \"--mav10 --streamrate=50 --target-system=${VEHICLE_ID}\" ${SIM_VEHICLE_ARGS}"
sim_vehicle.py -v ArduCopter --add-param-file=${PARAM_PATH} -f gazebo-iris -I$((${VEHICLE_ID} - 1)) -m "--mav10 --streamrate=50 --target-system=${VEHICLE_ID}" ${SIM_VEHICLE_ARGS}