import numpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import utm

bag_openarea = bagreader('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Data/topofcolumbusparkinggarage.bag')
bag_alleyway = bagreader('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Data/alleybetweenisecandcolumbus.bag')
bag_walk = bagreader('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Data/300mstraightwalk.bag')

decode_openarea = bag_openarea.message_by_topic('gps')
print("File saved: {}".format(decode_openarea))

decode_alleyway = bag_alleyway.message_by_topic('gps')
print("File saved: {}".format(decode_alleyway))

decode_walk = bag_walk.message_by_topic('gps')
print("File saved: {}".format(decode_walk))

data_open = pd.read_csv(decode_openarea)
data_alley = pd.read_csv(decode_alleyway)
data_walk = pd.read_csv(decode_walk)

open_GPGGA_Data = data_open.FullGPGGAString
alley_GPGGA_Data = data_alley.FullGPGGAString
walk_GPGGA_Data = data_walk.FullGPGGAString

# print(open_GPGGA_Data)

i = 0
open_Time = numpy.zeros(len(open_GPGGA_Data))
open_Latitude = numpy.zeros(len(open_GPGGA_Data))
open_Longitude = numpy.zeros(len(open_GPGGA_Data))
open_Altitude = numpy.zeros(len(open_GPGGA_Data))
open_HDOP = numpy.zeros(len(open_GPGGA_Data))
open_UTM_Northing = numpy.zeros(len(open_GPGGA_Data))
open_UTM_Easting = numpy.zeros(len(open_GPGGA_Data))

open_UTM_N_Error = numpy.zeros(len(open_GPGGA_Data))
open_UTM_E_Error = numpy.zeros(len(open_GPGGA_Data))
open_Known_UTM = utm.from_latlon(42.33803838025235, -71.08644762911916)
# print(open_Known_UTM[1])
# print(type(open_Known_UTM[1]))

while i < len(open_GPGGA_Data):
    bufferline = open_GPGGA_Data[i]
    buffersplit = bufferline.split(",")

    open_Time[i] = float(buffersplit[1])
    open_Altitude[i] = float(buffersplit[9])
    open_HDOP[i] = float(buffersplit[8])

    temp_lat = buffersplit[2]
    open_Latitude[i] = float(temp_lat[0:2]) + float(temp_lat[2:])/60
    if buffersplit[3] == "S":
        open_Latitude[i] = open_Latitude[i] * -1

    temp_long = buffersplit[4]
    open_Longitude[i] = float(temp_long[0:3]) + float(temp_long[3:])/60
    if buffersplit[5] == "W":
        open_Longitude[i] = open_Longitude[i] * -1

    utm_buffer = utm.from_latlon(open_Latitude[i], open_Longitude[i])
    open_UTM_Northing[i] = utm_buffer[1]
    open_UTM_Easting[i] = utm_buffer[0]

    open_UTM_N_Error[i] = open_UTM_Northing[i] - open_Known_UTM[1]
    open_UTM_E_Error[i] = open_UTM_Easting[i] - open_Known_UTM[0]

    i = i + 1

i = 0
alley_Time = numpy.zeros(len(alley_GPGGA_Data))
alley_Latitude = numpy.zeros(len(alley_GPGGA_Data))
alley_Longitude = numpy.zeros(len(alley_GPGGA_Data))
alley_Altitude = numpy.zeros(len(alley_GPGGA_Data))
alley_HDOP = numpy.zeros(len(alley_GPGGA_Data))
alley_UTM_Northing = numpy.zeros(len(alley_GPGGA_Data))
alley_UTM_Easting = numpy.zeros(len(alley_GPGGA_Data))

alley_UTM_N_Error = numpy.zeros(len(alley_GPGGA_Data))
alley_UTM_E_Error = numpy.zeros(len(alley_GPGGA_Data))
alley_Known_UTM = utm.from_latlon(42.33775882649961, -71.08656966962846)

while i < len(alley_GPGGA_Data):
    bufferline = alley_GPGGA_Data[i]
    buffersplit = bufferline.split(",")

    alley_Time[i] = float(buffersplit[1])
    alley_Altitude[i] = float(buffersplit[9])
    alley_HDOP[i] = float(buffersplit[8])

    temp_lat = buffersplit[2]
    alley_Latitude[i] = float(temp_lat[0:2]) + float(temp_lat[2:])/60
    if buffersplit[3] == "S":
        alley_Latitude[i] = alley_Latitude[i] * -1

    temp_long = buffersplit[4]
    alley_Longitude[i] = float(temp_long[0:3]) + float(temp_long[3:])/60
    if buffersplit[5] == "W":
        alley_Longitude[i] = alley_Longitude[i] * -1

    utm_buffer = utm.from_latlon(alley_Latitude[i], alley_Longitude[i])
    alley_UTM_Northing[i] = utm_buffer[1]
    alley_UTM_Easting[i] = utm_buffer[0]

    alley_UTM_N_Error[i] = alley_UTM_Northing[i] - alley_Known_UTM[1]
    alley_UTM_E_Error[i] = alley_UTM_Easting[i] - alley_Known_UTM[0]

    i = i + 1

i = 0
walk_Time = numpy.zeros(len(walk_GPGGA_Data))
walk_Latitude = numpy.zeros(len(walk_GPGGA_Data))
walk_Longitude = numpy.zeros(len(walk_GPGGA_Data))
walk_Altitude = numpy.zeros(len(walk_GPGGA_Data))
walk_HDOP = numpy.zeros(len(walk_GPGGA_Data))
walk_UTM_Northing = numpy.zeros(len(walk_GPGGA_Data))
walk_UTM_Easting = numpy.zeros(len(walk_GPGGA_Data))

walk_UTM_N_Scaled = numpy.zeros(len(walk_GPGGA_Data))
walk_UTM_E_Scaled = numpy.zeros(len(walk_GPGGA_Data))

while i < len(walk_GPGGA_Data):
    bufferline = walk_GPGGA_Data[i]
    buffersplit = bufferline.split(",")

    walk_Time[i] = float(buffersplit[1])
    walk_Altitude[i] = float(buffersplit[9])
    walk_HDOP[i] = float(buffersplit[8])

    temp_lat = buffersplit[2]
    walk_Latitude[i] = float(temp_lat[0:2]) + float(temp_lat[2:])/60
    if buffersplit[3] == "S":
        walk_Latitude[i] = walk_Latitude[i] * -1

    temp_long = buffersplit[4]
    walk_Longitude[i] = float(temp_long[0:3]) + float(temp_long[3:])/60
    if buffersplit[5] == "W":
        walk_Longitude[i] = walk_Longitude[i] * -1

    utm_buffer = utm.from_latlon(walk_Latitude[i], walk_Longitude[i])
    walk_UTM_Northing[i] = utm_buffer[1]
    walk_UTM_Easting[i] = utm_buffer[0]

    walk_UTM_N_Scaled[i] = walk_UTM_Northing[i] - walk_UTM_Northing[0]
    walk_UTM_E_Scaled[i] = walk_UTM_Easting[i] - walk_UTM_Easting[0]

    i = i + 1

# print(open_Latitude)
# print("\n")
# print(open_Longitude)
# print("\n")
# print(open_UTM_Northing)
# print("\n")
# print(open_UTM_Easting)
# print("\n")
# print(open_UTM_N_Corrected)
# print("\n")
# print(open_UTM_E_Corrected)
# print("\n")

# print(open_UTM_N_Corrected)
# print("\n")
# print(alley_UTM_N_Corrected)
# print("\n")
# print(walk_UTM_Northing)
# print("\n")

open_UTM_N_Scaled = numpy.asarray(open_UTM_Northing - open_UTM_Northing[0])
open_UTM_E_Scaled = numpy.asarray(open_UTM_Easting - open_UTM_Easting[0])

alley_UTM_N_Scaled = numpy.asarray(alley_UTM_Northing - alley_UTM_Northing[0])
alley_UTM_E_Scaled = numpy.asarray(alley_UTM_Easting - alley_UTM_Easting[0])

walk_UTM_N_Scaled = numpy.asarray(walk_UTM_Northing - walk_UTM_Northing[0])
walk_UTM_E_Scaled = numpy.asarray(walk_UTM_Easting - walk_UTM_Easting[0])

i = 0
open_Error = numpy.zeros(len(open_UTM_N_Error))
while i < len(open_UTM_N_Error):
    open_Error[i] = (open_UTM_N_Error[i]**2 + open_UTM_E_Error[i]**2)**(1/2)
    i = i + 1

i = 0
alley_Error = numpy.zeros(len(alley_UTM_N_Error))
while i < len(alley_UTM_N_Error):
    alley_Error[i] = (alley_UTM_N_Error[i]**2 + alley_UTM_E_Error[i]**2)**(1/2)
    i = i + 1

[m, b] = np.polyfit(walk_UTM_Easting, walk_UTM_Northing, 1)
walk_BestFit = np.polyval([m, b], walk_UTM_Easting)
walk_BestFit_Scaled = numpy.asarray(walk_BestFit - walk_BestFit[0])

i = 0
walk_Error = numpy.zeros(len(walk_UTM_N_Scaled))
while i < len(walk_UTM_N_Scaled):
    walk_Error[i] = walk_UTM_N_Scaled[i] - walk_BestFit_Scaled[i]
    i = i + 1

open_AverageError = numpy.average(open_Error)
alley_AverageError = numpy.average(alley_Error)
walk_AverageError = numpy.average(walk_Error)

print('\n' + 'The average error on top of Columbus Garage was ' + str(open_AverageError) + ' m \n')
print('The average error in the alley between ISEC and Columbus was ' + str(alley_AverageError) + ' m \n')
print('The average error on the walk was ' + str(walk_AverageError) + ' m \n')

plt.figure(1)
plt.scatter(open_UTM_E_Scaled, open_UTM_N_Scaled)
plt.grid(True)
plt.title('UTM Northing vs. Easting Data on Top of Columbus Garage')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/FixedAnalysis/Garage_UTM_NorthVsEast')

plt.figure(2)
plt.scatter(alley_UTM_E_Scaled, alley_UTM_N_Scaled)
plt.grid(True)
plt.title('UTM Northing vs. Easting Data in Alley Between Columbus Garage and ISEC')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/FixedAnalysis/Alley_UTM_NorthVsEast')

plt.figure(3)
plt.scatter(walk_UTM_E_Scaled, walk_UTM_N_Scaled)
plt.plot(walk_UTM_E_Scaled, walk_BestFit_Scaled)
plt.grid(True)
plt.title('UTM Northing vs. Easting Data for a ~300 m Walk')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/FixedAnalysis/300mWalk_UTM_NorthVsEast')

plt.figure(4)
plt.scatter(open_Time, open_Altitude)
plt.ylim([0, 45])
plt.grid(True)
plt.title('Altitude vs. Time Data on Top of Columbus Garage')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/FixedAnalysis/Garage_AltitudeVsTime')

plt.figure(5)
plt.scatter(alley_Time, alley_Altitude)
plt.ylim([0, 65])
plt.grid(True)
plt.title('Altitude vs. Time Data in Alley Between Columbus Garage and ISEC')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/FixedAnalysis/Alley_AltitudeVsTime')

plt.figure(6)
plt.scatter(walk_Time, walk_Altitude)
plt.grid(True)
plt.title('Altitude vs. Time Data for a ~300 m Walk')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/FixedAnalysis/Walk_AltitudeVsTime')

plt.figure(7)
countsopen, binsopen = np.histogram(open_Error)
plt.hist(binsopen[:-1], binsopen, weights=countsopen)
plt.title('Distribution of Error on Top of Columbus Garage')
plt.xlabel('UTM Northing (m)')
plt.ylabel('UTM Easting (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/FixedAnalysis/Garage_Error_Histogram')

plt.figure(8)
countsalley, binsalley = np.histogram(alley_Error)
plt.hist(binsalley[:-1], binsalley, weights=countsalley)
plt.title('Distribution of Error in Alley Between Columbus Garage and ISEC')
plt.xlabel('UTM Northing (m)')
plt.ylabel('UTM Easting (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/FixedAnalysis/Alley_Error_Histogram')