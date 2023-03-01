#!/usr/bin/env bash

MAX_CPU_INDEX=$(cat /sys/devices/system/cpu/present | awk -F "-" '{print $2}')
CPU_CORES=$((${MAX_CPU_INDEX}+1))
ENABLE_CORES=$1
SYS_CPU=/sys/devices/system/cpu/cpu

if [ ${EUID:-${UID}} != 0 ]; then
    echo "This script must be run as root."
    exit -1
fi

if [ $# -lt 1 ]; then
    echo "usage: $0 number_of_enable_cpu_cores"
    exit -1
fi

echo "Number of CPU: ${CPU_CORES}"

if [ ${ENABLE_CORES} -lt 1 ]; then
    echo "Number of enable cpu cores must be larger than 1."
    exit -1
fi

if [ ${ENABLE_CORES} -gt ${CPU_CORES} ]; then
    echo "Number of enable cpu cores must be smaller than ${CPU_CORES}."
    exit -1
fi

echo "Number of enable cores: ${ENABLE_CORES}"

ENABLE_LAST_INDEX=$((${ENABLE_CORES}-1))
DISABLE_FIRST_INDEX=${ENABLE_CORES}

echo "Always enable CPU index 0"

for n in `seq 1 ${ENABLE_LAST_INDEX}`; do
    echo "Enable CPU index $n"
    echo 1 > ${SYS_CPU}$n/online
done

for n in `seq ${DISABLE_FIRST_INDEX} ${MAX_CPU_INDEX}`; do
    echo "Disable CPU index $n"
    echo 0 > ${SYS_CPU}$n/online
done