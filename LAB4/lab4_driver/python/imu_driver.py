# #!/usr/bin/env python
# # -*- coding: utf-8 -*-

import rospy
import serial
from imu_driver.msg import Vectornav
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionRequest, convert_to_quaternionResponse
import re

rospy.Time
rospy.sleep(.5)


def replacenth(string, sub, wanted, n):
    where = [m.start() for m in re.finditer(sub, string)][n - 1]
    before = string[:where]
    after = string[where:]
    after = after.replace(sub, wanted)
    newString = before + after
    return newString


def convert_to_quaternion_client(yaw, pitch, roll):
    rospy.wait_for_service('convert_to_quaternion')
    try:
        quaternion_service = rospy.ServiceProxy('convert_to_quaternion', convert_to_quaternion)
        response = quaternion_service(roll, pitch, yaw)
        # return [response.qx, response.qy, response.qz, response.qw]
        return response
    except rospy.ServiceException as e:
        print("Service call failed: {0}".format(e))


def split_line(buffer):
    if buffer == "":
        rospy.logwarn("NO DATA DETECTED")  # debug line taken from example, outputs error if no data detected
    elif "VNYMR" in buffer:
        buffer2 = buffer.split('*')
        vnymr_data = buffer2[0].split(',')      # this function splits the line and sections into different arrays
        # rospy.loginfo("VNYMR DATA STRING: :-) !!!")
        rospy.loginfo(vnymr_data)
        eulers = [0, 0, 0]
        magxyz = [0, 0, 0]
        accelxyz = [0, 0, 0]
        gyroxyz = [0, 0, 0]

        if len(vnymr_data) > 14:
            vnymr_data = vnymr_data[0:15]

        for i in range(0, len(vnymr_data)):

            if chr(0) in vnymr_data[i]:
                # print(gyroxyz[i])
                vnymr_data[i] = vnymr_data[i].replace(chr(0), "")
                print(vnymr_data[i] + " ---> " + vnymr_data[i].replace(chr(0), ""))
                rospy.loginfo("I found an error!")

            if (vnymr_data[i].count("+") + vnymr_data[i].count("-")) > 1:
                ind_plus = vnymr_data[i].find("+")
                ind_neg = vnymr_data[i].find("-")

                vnymr_data.append(vnymr_data[len(vnymr_data)])
                vnymr_data[(i+2):-1] = vnymr_data[(i+1):-2]

                if ind_plus == -1:
                    temp2 = vnymr_data[i].split("-")
                    vnymr_data[i] = float("-" + temp2(1))
                    vnymr_data[i + 1] = float("-" + temp2(2))
                elif ind_neg == -1:
                    temp2 = vnymr_data[i].split("+")
                    vnymr_data[i] = float("+" + temp2(1))
                    vnymr_data[i + 1] = float("+" + temp2(2))
                elif ind_plus < ind_neg:
                    temp2 = vnymr_data[i].split("-")
                    vnymr_data[i] = float(temp2(0))
                    vnymr_data[i + 1] = float("-" + temp2(1))
                else:
                    temp2 = vnymr_data[i].split("+")
                    vnymr_data[i] = float(temp2(0))
                    vnymr_data[i + 1] = float("+" + temp2(1))
                rospy.loginfo("Broke up clumped data!")
                rospy.loginfo("New data: " + vnymr_data)

            if vnymr_data[i].count(".") > 1:
                vnymr_data[i] = replacenth(vnymr_data[i], ".", "", 1)

            if i > 0:
                try:
                    if i <= 3:
                        # rospy.loginfo(str(i-1))
                        eulers[i-1] = float(vnymr_data[i])
                    elif 4 <= i <= 6:
                        # rospy.loginfo(str(i - 4))
                        magxyz[i-4] = .0001 * float(vnymr_data[i])
                    elif 7 <= i <= 9:
                        # rospy.loginfo(str(i - 7))
                        accelxyz[i-7] = float(vnymr_data[i])
                    elif 10 <= i <= 12:
                        # rospy.loginfo(str(i - 10))
                        gyroxyz[i-10] = float(vnymr_data[i])
                except ValueError:
                    pass

        # eulers = list(map(float, vnymr_data[1:4]))
        # magxyz = list(map(float, vnymr_data[4:7]))
        # accelxyz = list(map(float, vnymr_data[7:10]))
        # gyroxyz = list(map(str, vnymr_data[10:]))
        # rospy.loginfo(gyroxyz)
        # gyroxyz = list(map(float, gyroxyz))

    else:
        eulers = [0, 0, 0]
        magxyz = [0, 0, 0]
        accelxyz = [0, 0, 0]
        gyroxyz = [0, 0, 0]
    return [eulers, magxyz, accelxyz, gyroxyz]      # return pieces in an array format


# builds my message
def message_compiler(seq, mags, quats, angs, lins, fullstring):
    msg = Vectornav()
    header = Header()   # temp header
    imu = Imu()         # temp imu
    mag_field = MagneticField()     # temp magfield (not sure if useful)

    now = rospy.get_rostime()
    header.seq = seq
    header.frame_id = 'IMU_Frame1'
    header.stamp.secs = now.secs
    header.stamp.nsecs = now.nsecs     # define header fields
    # quats.x, quats.y, quats.z, quats.w = quats
    # rospy.loginfo(quats)
    # rospy.loginfo(quats.qx)
    # rospy.loginfo(angs)
    imu.orientation.x = quats.qx
    imu.orientation.y = quats.qy
    imu.orientation.z = quats.qz
    imu.orientation.w = quats.qw

    imu.angular_velocity.x = angs[0]
    imu.angular_velocity.y = angs[1]
    imu.angular_velocity.z = angs[2]

    imu.linear_acceleration.x = lins[0]
    imu.linear_acceleration.y = lins[1]
    imu.linear_acceleration.z = lins[2]      # define imu fields

    mag_field.magnetic_field.x = mags[0]    # define magnetics fields
    mag_field.magnetic_field.y = mags[1]
    mag_field.magnetic_field.z = mags[2]

    msg.header = header
    msg.imu = imu
    msg.mag_field = mag_field
    msg.Full_VNYMR_String = fullstring      # build full string
    return msg


if __name__ == '__main__':
    SENSOR_NAME = "VectorNAV"
    rospy.init_node('imu_driver')   # initialize the node

    serial_port = rospy.get_param('/imu_driver/port', '/dev/ttyACM1')   # get port from parameter server (launch arg)
    # serial_baud = rospy.get_param('/imu_driver/baudrate')               # get baud rate
    # sampling_rate = rospy.get_param('/imu_driver/sampling_rate')        # get sampling rate in Hz
    serial_baud = 115200
    sampling_rate = 40.0
    sleep_time = 1 / sampling_rate - .025

    rospy.loginfo("Port, baud rate, and sample rate arguments received.")

    imu_port = serial.Serial(serial_port, serial_baud, timeout=3.)  # open serial port
    imu_port.flushInput()  # flush input buffer
    imu_port.flushOutput()  # flush output buffer

    rospy.loginfo("Connected to VectorNAV module at port " + serial_port + ".")     # debug

    imu_port.write(bytes(("$VNWRG,05," + str(int(serial_baud))) + "*XX", 'utf-8'))    # write to reg. 5 to declare baud
    rospy.loginfo("Configured serial baud rate: " + str(int(serial_baud)))
    imu_port.write(bytes("$VNWRG,06,14*XX", 'utf-8'))   # write to reg. 6 to declare output type
    rospy.loginfo("Configured output data type: VNYMR")
    imu_port.write(bytes(("$VNWRG,07," + str(int(sampling_rate)) + "*XX"), 'utf-8'))  # write to reg. 7 to
    # declare sample freq.
    rospy.loginfo("Configured async data output frequency: " + str(int(sampling_rate)) + " Hz")

    imu_pub = rospy.Publisher('imu', Vectornav, queue_size=5)   # start publishing to topic

    rospy.loginfo("Initialization complete")    # log messages taken from lab 1 example code
    rospy.loginfo("Publishing orientation, acceleration, and magnetic field data")

    sequence = 0

    try:  # i took the "try" syntax from the example code
        while not rospy.is_shutdown():  # test condition
            line = imu_port.readline()          # obtain a line from the sensor
            try:    # taken from stack exchange
                line = line.decode()    # decode into string from byte map
                # rospy.loginfo("Line decoded")
            except (UnicodeDecodeError, AttributeError):
                rospy.loginfo("Line NOT decoded!")
                pass    # if object is not a bytes-like object, just passes to next step
            # only using this ^^ because i am not sure what type the sensor will output

            # get the euler angles, magnetics, accelerations from the data here:
            eulers1, mags1, accels1, gyros1 = split_line(line)
            quats1 = convert_to_quaternion_client(eulers1[0], eulers1[1], eulers1[2])
            # call my srv file to process eulers into quaternions

            rospy.loginfo(eulers1)
            message = message_compiler(sequence, mags1, quats1, gyros1, accels1, line)
            imu_pub.publish(message)    # call message compiler function and publish
            rospy.loginfo(message)      # also send message contents to log
            sequence = sequence + 1     # increment sequence
            rospy.sleep(sleep_time)     # still needed??

    except rospy.ROSInterruptException:
        imu_port.close()  # debug line taken from example code. shuts off terminal when ROSInterruptException is raised

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down GNSS driver node...")  # I took this from the example too
