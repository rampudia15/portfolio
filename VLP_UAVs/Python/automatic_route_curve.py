# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

#Modified version of the trajectory and log configuration by:
# Ricardo Ampudia <r.ampudia15@gmail.com>
# Last modification: August 2021

"""
Simple example that connects to one crazyflie, sets the initial position/yaw
and flies a trajectory.

The initial pose (x, y, z, yaw) is configured in a number of variables and
the trajectory is flown relative to this position, using the initial yaw.

This example is intended to work with any absolute positioning system.
It aims at documenting how to take off with the Crazyflie in an orientation
that is different from the standard positive X orientation and how to set the
initial position of the kalman estimator.
"""

import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'

# Change the sequence according to your setup
#             x    y    z
sequence = [
(   0.0     ,   0.0     ,   0.3     )   ,
(   0.0     ,   0.0     ,   0.6     )   ,
(   0.0     ,   0.0     ,   1.0     )   ,
(   0.0     ,   0.0     ,   1.3     )   ,
(   0.25    ,   0.0     ,   1.3     )   ,
(	0.50	,	0.00	,	1.30	)	,
(	0.49	,	0.10	,	1.40	)	,
(	0.46	,	0.19	,	1.48	)	,
(	0.41	,	0.28	,	1.53	)	,
(	0.35	,	0.36	,	1.55	)	,
(	0.27	,	0.42	,	1.53	)	,
(	0.18	,	0.47	,	1.47	)	,
(	0.08	,	0.49	,	1.38	)	,
(	-0.01	,	0.50	,	1.29	)	,
(	-0.11	,	0.49	,	1.19	)	,
(	-0.21	,	0.45	,	1.11	)	,
(	-0.29	,	0.40	,	1.06	)	,
(	-0.37	,	0.34	,	1.05	)	,
(	-0.43	,	0.26	,	1.08	)	,
(	-0.47	,	0.17	,	1.14	)	,
(	-0.49	,	0.07	,	1.23	)	,
(	-0.50	,	-0.03	,	1.33	)	,
(	-0.48	,	-0.13	,	1.42	)	,
(	-0.45	,	-0.22	,	1.50	)	,
(	-0.40	,	-0.31	,	1.54	)	,
(	-0.33	,	-0.38	,	1.55	)	,
(	-0.25	,	-0.44	,	1.51	)	,
(	-0.15	,	-0.48	,	1.45	)	,
(	-0.06	,	-0.50	,	1.36	)	,
(	0.04	,	-0.50	,	1.26	)	,
(	0.14	,	-0.48	,	1.16	)	,
(	0.23	,	-0.44	,	1.09	)	,
(	0.32	,	-0.39	,	1.05	)	,
(	0.39	,	-0.32	,	1.06	)	,
(	0.44	,	-0.23	,	1.09	)	,
(	0.48	,	-0.14	,	1.17	)	,
(	0.50	,	-0.04	,	1.26	)	,
(	0.50	,	0.00	,	1.30	)	,
(   0.25    ,   0.0     ,   1.3     )   ,
(   0.0     ,   0.0     ,   1.0     )   ,
(   0.0     ,   0.0     ,   0.6     )   ,
(   0.0     ,   0.0     ,   0.2     )   ,
]

def set_initial_position(scf, x, y, z, yaw_deg):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)

    yaw_radians = math.radians(yaw_deg)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)

def data_received_callback(timestamp, data, logconf):
    print ("[%d][%s]: %s" % (timestamp, logconf.name, data))

def logging_error(logconf, msg):
        print ("Error when logging %s" % logconf.name)

def log_pos_callback(timestamp, data, logconf):
    print(timestamp, data)
    #global position_estimate
    #position_estimate[0] = data['stateEstimate.x']
    #position_estimate[1] = data['stateEstimate.y']


def run_sequence(scf, sequence, base_x, base_y, base_z, yaw):
    cf = scf.cf
    old_pos = (0,0,0)
    k=1
    for position in sequence:
        #print('Setting position {}'.format(position))

        x = position[0] + base_x
        y = position[1] + base_y
        z = position[2] + base_z

        if k < 6 or k > 37:
            steps=10
            for i in range(steps):
                x2= round(old_pos[0] + (i+1)*(x-old_pos[0])/steps,2)
                y2= round(old_pos[1] + (i+1)*(y-old_pos[1])/steps,2)
                z2= round(old_pos[2] + (i+1)*(z-old_pos[2])/steps,2)
                cf.commander.send_position_setpoint(x2, y2, z2, yaw)
                time.sleep(0.1)
        else:
            steps=2
            for i in range(steps): 
                x2= round(old_pos[0] + (i+1)*(x-old_pos[0])/steps,2)
                y2= round(old_pos[1] + (i+1)*(y-old_pos[1])/steps,2)
                z2= round(old_pos[2] + (i+1)*(z-old_pos[2])/steps,2)
                cf.commander.send_position_setpoint(x2, y2, z2, yaw)
                time.sleep(0.2)
        k=k+1
        old_pos = position

    #cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Set these to the position and yaw based on how your Crazyflie is placed
    # on the floor
    initial_x = 0
    initial_y = 0
    initial_z = 0.0
    initial_yaw = 90  # In degrees
    # 0: positive X direction
    # 90: positive Y direction
    # 180: negative X direction
    # 270: negative Y direction

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        
        time.sleep(3)
        
        #Log ground truth at 20 Hz
        logconf1 = LogConfig(name='Ground truth', period_in_ms=50)
        logconf1.add_variable('stateEstimate.x', 'float')
        logconf1.add_variable('stateEstimate.y', 'float')
        logconf1.add_variable('stateEstimate.z', 'float')
        logconf1.add_variable('stabilizer.roll', 'float')
        logconf1.add_variable('stabilizer.pitch', 'float')
        logconf1.add_variable('stabilizer.yaw', 'float')
        
        #Log VLP data after FFT at 10 Hz
        logconf2 = LogConfig(name='VLP', period_in_ms=100)
        logconf2.add_variable('PD_ADC.Pr_1', 'float')
        logconf2.add_variable('PD_ADC.Pr_2', 'float')
        logconf2.add_variable('PD_ADC.Pr_3', 'float')
        logconf2.add_variable('PD_ADC.Pr_4', 'float')

        #Log IMU data at 50 Hz
        logconf3 = LogConfig(name='IMU', period_in_ms=20)
        logconf3.add_variable('stateEstimate.ax', 'float')
        logconf3.add_variable('stateEstimate.ay', 'float')
        logconf3.add_variable('stateEstimate.az', 'float')
        
        #Log height data at 20 Hz
        logconf4 = LogConfig(name='Height', period_in_ms=50)
        logconf4.add_variable('baro.asl', 'float')
        
        #Start log configurations
        scf.cf.log.add_config(logconf1)
        scf.cf.log.add_config(logconf2)
        scf.cf.log.add_config(logconf3)
        scf.cf.log.add_config(logconf4)
        logconf1.data_received_cb.add_callback(log_pos_callback)
        logconf2.data_received_cb.add_callback(log_pos_callback)
        logconf3.data_received_cb.add_callback(log_pos_callback)
        logconf4.data_received_cb.add_callback(log_pos_callback)
        logconf1.start()
        logconf2.start()
        logconf3.start()
        logconf4.start()
        
        #Wait 2 seconds before flight for sensor calibration
        time.sleep(2)
        
        set_initial_position(scf, initial_x, initial_y, initial_z, initial_yaw)
        run_sequence(scf, sequence,
                     initial_x, initial_y, initial_z, initial_yaw)

        #Sometimes the port from the VM takes time to print data, give it some 
        # time for the buffer to empty before ending the program
        time.sleep(15)
