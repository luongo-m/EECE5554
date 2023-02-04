#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import rospy
import serial
import utm
from gps_driver.msg import gps_msg
from std_msgs.msg import Header
import sys

# parser = argparse.ArgumentParser(
#     prog="GPS Driver ROS Publisher",
#     description="Reads serial data from a GPS puck and publishes to 'gps' topic",
#     epilog="Made by Matt Luongo not sure what goes here"
# )

# def gps_message(frameid, seq, stamp, lat, long, alt, hdop, utmeast, utmnorth, utc, zone, letter, fullstr):
#     # msg = gps_msg
#     msg.Header.frame_id = frameid
#     msg.Header.seq = seq
#
#     msg.Latitude = lat
#     msg.Longitude = long
#     msg.Altitude = alt
#     msg.HDOP = hdop
#     msg.UTM_easting = utmeast
#     msg.UTM_northing = utmnorth
#     msg.UTC = utc
#     msg.Zone = zonemsg.Header.stamp = stamp
#     msg.Letter = letter
#     msg.FullGPGGAString = fullstr

# parser.add_argument('--port', type=str)
# parser.add_argument('--baud', type=int)
# parser.add_argument('--rate', type=float)
# args = parser.parse_args()

# print("here")
if __name__ == '__main__':
    SENSOR_NAME = "GPS_puck"
    rospy.init_node('driver')
    # print("here")

    # this block sets the arguments to those specified when the file is executed in case you want to use arg parser
    # if args.port == '':
    #     serial_port = rospy.get_param('~port', '/dev/pts/3')
    # else:
    #     serial_port = args.port
    # if args.baud == '':
    #     serial_baud = rospy.get_param('~baudrate', 4800)
    # else:
    #     serial_baud = args.baud
    # if args.rate == '':
    #     sampling_rate = rospy.get_param('~sampling_rate', 5.0)
    # else:
    #     sampling_rate = args.rate
    # args = sys.argv
    # print(args)
    # serial_port = args[0]
    # serial_baud = args[1]
    # sampling_rate = args[2]
    serial_port = rospy.get_param('/driver/port', '/dev/ttyACM1')
    serial_baud = rospy.get_param('/driver/baudrate', 4800)
    sampling_rate = rospy.get_param('/driver/sampling_rate', 5.0)
    # rospy.loginfo(serial_port)

    # serial_port = '/dev/ttyACM1'
    # serial_baud = 4800
    # sampling_rate = 1.0
    rospy.loginfo("Port, baud rate, and sample rate arguments received.")

    matt_port = serial.Serial(serial_port, serial_baud, timeout=3.)  # open serial port
    matt_port.flushInput()  # flush input buffer
    matt_port.flushOutput()  # flush output buffer

    # print(matt_port.name)  # prints the name of the serial port
    # ADD ALL ROSPY.LOGDEBUG LINES HERE
    rospy.loginfo("Connected to GPS puck at port " + serial_port + " at " + str(serial_baud) + ".")

    sampling_count = int(round(1 / (sampling_rate * .007913)))
    # matt_port.write('*0100EW*0100PR='+str(sampling_count)+'\r\n')
    rospy.sleep(.2)
    buffer = matt_port.readline()
    # matt_port.write('*0100P$\r\n')

    gps_pub = rospy.Publisher('gps', gps_msg, queue_size=5)

    rospy.loginfo("Initialization complete")
    rospy.loginfo("Publishing GGA and UTM data")

    msg = gps_msg()
    temp_header = Header()
    sequence = 0
    temp_header.seq = sequence
    temp_header.frame_id = 'GPS_Frame1'
    sleep_time = 1 / sampling_rate - .025  # taken from example code

    try:  # i took the "try" syntax from the example code
        while not rospy.is_shutdown():  # test condition
            line = matt_port.readline()
            buffer = str(line, 'utf-8')
            # print(buffer[5:10])  # prints out first few chars to help matt figure out where to look
            # print(buffer)
            if line == '':
                rospy.logwarn("NO DATA DETECTED")  # debug line taken from example, outputs error if no data detected
            else:
                # if buffer.startswith('GPGGA', 5, 10):  # I think there is an opportunity
                # to compact this into a function
                print(buffer)
                print(buffer[0:6])
                if buffer.startswith('$GPGGA', 0, 6):
                    # definition, but i didn't want to
                    # print('GPGGA line detected')
                    # print(buffer)
                    GPGGA_data = buffer.split(',')  # splits buffer into pieces, each piece is a parameter
                    # print(GPGGA_data)  # debug line, proves it is splitting correctly
                    # print('Latitude is '+str(GPGGA_data[2]))  # debug for matt, making sure my indeces are correct
                    # print('Longitude is '+str(GPGGA_data[4]))  #debug for matt
                    latitude_raw = GPGGA_data[2]
                    longitude_raw = GPGGA_data[4]

                    latitude_deg = int(latitude_raw[0:2]) + float(latitude_raw[2:6]) / 60
                    if GPGGA_data[3] == 'S':
                        latitude_deg = -1 * latitude_deg

                    longitude_deg = int(longitude_raw[0:3]) + float(latitude_raw[3:7]) / 60
                    if GPGGA_data[5] == 'W':
                        longitude_deg = -1 * longitude_deg

                    UTM_data = utm.from_latlon(latitude_deg, longitude_deg)

                    # define the ros message using each custom fields:
                    # not sure if there is a better way to do this
                    # fill values from GPGGA_data (split up GPGGA string) and from UTM_data

                    UTC_time = GPGGA_data[1]
                    hourstoseconds = int(float(UTC_time[0:2])*3600)
                    minutestoseconds = int(float(UTC_time[2:4])*60)
                    seconds = int(float(UTC_time[4:6]))
                    secondsfraction = int(float(UTC_time[6:]))

                    temp_header.seq = sequence
                    sequence = sequence + 1
                    temp_header.stamp.secs = hourstoseconds + minutestoseconds + seconds
                    temp_header.stamp.nsecs = secondsfraction
                    temp_header.frame_id = 'GPS_Frame1'
                    
                    msg.Header = temp_header
                    msg.Latitude = latitude_deg
                    msg.Longitude = longitude_deg
                    msg.Altitude = float(GPGGA_data[9])
                    msg.HDOP = float(GPGGA_data[7])
                    msg.UTM_easting = UTM_data[0]
                    msg.UTM_northing = UTM_data[1]
                    msg.UTC = float(GPGGA_data[1])
                    msg.Zone = int(UTM_data[2])
                    msg.Letter = UTM_data[3]
                    msg.FullGPGGAString = buffer

                    # gps_message("GPS_Frame1", 0, GPGGA_data[1], latitude_deg, longitude_deg, GPGGA_data[9],
                    #             GPGGA_data[7], UTM_data[0], UTM_data[1], GPGGA_data[1], UTM_data[2], UTM_data[3],
                    #             buffer)
                    gps_pub.publish(msg)
                    rospy.loginfo(msg)
                rospy.sleep(sleep_time)

    except rospy.ROSInterruptException:
       matt_port.close()  # debug line taken from example code. shuts off terminal when ROSInterruptException is raised

    except serial.serialutil.SerialException:
       rospy.loginfo("Shutting down GPS driver node...")  # I took this from the example too
