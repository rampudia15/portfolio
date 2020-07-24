#!/bin/sh

if [ "$#" -ne 6 ]; then
    echo "Need 6 parameters"
fi

sed -i '12s/.*/#define H_G_LOW\t\t'$1'/' ../src/image_processing.h
sed -i '13s/.*/#define H_G_HIGH\t'$2'/' ../src/image_processing.h
sed -i '14s/.*/#define S_G_LOW\t\t'$3'/' ../src/image_processing.h
sed -i '15s/.*/#define S_G_HIGH\t'$4'/' ../src/image_processing.h
sed -i '16s/.*/#define V_G_LOW\t\t'$5'/' ../src/image_processing.h
sed -i '17s/.*/#define V_G_HIGH\t'$6'/' ../src/image_processing.h
