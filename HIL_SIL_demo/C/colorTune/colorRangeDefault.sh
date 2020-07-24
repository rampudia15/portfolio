#!/bin/sh

sed -i '12s/.*/#define H_G_LOW\t\t60/' ../src/image_processing.h
sed -i '13s/.*/#define H_G_HIGH\t127/' ../src/image_processing.h
sed -i '14s/.*/#define S_G_LOW\t\t126/' ../src/image_processing.h
sed -i '15s/.*/#define S_G_HIGH\t255/' ../src/image_processing.h
sed -i '16s/.*/#define V_G_LOW\t\t73/' ../src/image_processing.h
sed -i '17s/.*/#define V_G_HIGH\t255/' ../src/image_processing.h
