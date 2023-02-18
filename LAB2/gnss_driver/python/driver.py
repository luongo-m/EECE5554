#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from gnss_driver.msg import gnss_msg
from std_msgs.msg import Header

# print("here")
if __name__ == '__main__':
    SENSOR_NAME = "EMLID_Reach"
    rospy.init_node('gnss_driver')

    serial_port = rospy.get_param('/gnss_driver/port', '/dev/ttyACM1')
    serial_baud = rospy.get_param('/gnss_driver/baudrate', 4800)
    sampling_rate = rospy.get_param('/gnss_driver/sampling_rate', 5.0)

    rospy.loginfo("Port, baud rate, and sample rate arguments received.")

    matt_port = serial.Serial(serial_port, serial_baud, timeout=3.)  # open serial port
    matt_port.flushInput()  # flush input buffer
    matt_port.flushOutput()  # flush output buffer

    rospy.loginfo("Connected to EMLID Reach at port " + serial_port + " at " + str(serial_baud) + ".")

    sampling_count = int(round(1 / (sampling_rate * .007913)))
    rospy.sleep(.2)
    buffer = matt_port.readline()

    gps_pub = rospy.Publisher('gnss', gnss_msg, queue_size=5)

    rospy.loginfo("Initialization complete")
    rospy.loginfo("Publishing GGA and UTM data")

    msg = gnss_msg()
    temp_header = Header()
    sequence = 0
    temp_header.seq = sequence
    temp_header.frame_id = 'GNSS_Frame1'
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
                # if buffer.startswith('$GNGGA', 0, 6):
                if "GNGGA" in buffer:
                    # definition, but i didn't want to
                    # print('GPGGA line detected')
                    # print(buffer)
                    GNGGA_data = buffer.split(',')  # splits buffer into pieces, each piece is a parameter
                    # print(GNGGA_data)  # debug line, proves it is splitting correctly
                    # print('Latitude is '+str(GNGGA_data[2]))  # debug for matt, making sure my indeces are correct
                    # print('Longitude is '+str(GNGGA_data[4]))  #debug for matt
                    latitude_raw = GNGGA_data[2]
                    longitude_raw = GNGGA_data[4]

                    latitude_deg = float(latitude_raw[0:2]) + float(latitude_raw[2:]) / 60
                    if GNGGA_data[3] == 'S':
                        latitude_deg = -1 * latitude_deg

                    longitude_deg = float(longitude_raw[0:3]) + float(latitude_raw[3:]) / 60
                    if GNGGA_data[5] == 'W':
                        longitude_deg = -1 * longitude_deg

                    UTM_data = utm.from_latlon(latitude_deg, longitude_deg)

                    # define the ros message using each custom fields:
                    # not sure if there is a better way to do this
                    # fill values from GNGGA_data (split up GNGGA string) and from UTM_data

                    UTC_time = GNGGA_data[1]
                    hourstoseconds = int(float(UTC_time[0:2])*3600)
                    minutestoseconds = int(float(UTC_time[2:4])*60)
                    seconds = int(float(UTC_time[4:6]))
                    secondsfraction = int(float(UTC_time[6:]))

                    temp_header.seq = sequence
                    sequence = sequence + 1
                    temp_header.stamp.secs = hourstoseconds + minutestoseconds + seconds
                    temp_header.stamp.nsecs = secondsfraction
                    temp_header.frame_id = 'GNSS_Frame1'
                    
                    msg.Header = temp_header
                    msg.Latitude = latitude_deg
                    msg.Longitude = longitude_deg
                    msg.Altitude = float(GNGGA_data[9])
                    msg.HDOP = float(GNGGA_data[8])
                    msg.UTM_easting = UTM_data[0]
                    msg.UTM_northing = UTM_data[1]
                    msg.UTC = float(GNGGA_data[1])
                    msg.Zone = int(UTM_data[2])
                    msg.Letter = UTM_data[3]
                    msg.Fix = int(GNGGA_data[6])
                    msg.FullGNGGAString = buffer

                    # gps_message("GPS_Frame1", 0, GNGGA_data[1], latitude_deg, longitude_deg, GNGGA_data[9],
                    #             GNGGA_data[7], UTM_data[0], UTM_data[1], GNGGA_data[1], UTM_data[2], UTM_data[3],
                    #             buffer)
                    gps_pub.publish(msg)
                    rospy.loginfo(msg)
                rospy.sleep(sleep_time)

    except rospy.ROSInterruptException:
       matt_port.close()  # debug line taken from example code. shuts off terminal when ROSInterruptException is raised

    except serial.serialutil.SerialException:
       rospy.loginfo("Shutting down GNSS driver node...")  # I took this from the example too
