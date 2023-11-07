#!/bin/bash

set -e
set -x

FROM=$1
NAMESPACE=$2

[ -z "${NAMESPACE}" ] && NAMESPACE="red"
[ -z "${FROM}" ] && FROM=$(rospack find ardupilot_gazebo)/config/apm_config_NAMESPACE.yaml
TO=$(rospack find ardupilot_gazebo)/config/apm_config_NAMESPACE_$NAMESPACE.yaml

sed s/NAMESPACE/$NAMESPACE/ $FROM | tee $TO

