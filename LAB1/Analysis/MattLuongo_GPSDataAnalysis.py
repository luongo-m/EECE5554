import bagpy
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

# print(data_open.UTM_northing[0])
open_Northing = numpy.asarray(data_open.UTM_northing - data_open.UTM_northing[0])
open_Easting = numpy.asarray(data_open.UTM_easting - data_open.UTM_easting[0])
open_Known_UTM = utm.from_latlon(42.33803838025235, -71.08644762911916)
open_Known = [float(open_Known_UTM[1]), float(open_Known_UTM[0])]
open_N_Error = numpy.asarray(data_open.UTM_northing - open_Known[0])
open_E_Error = numpy.asarray(data_open.UTM_easting - open_Known[1])

# print(len(open_N_Error))
# print(len(open_E_Error))

i = 0
open_Error = numpy.zeros(len(open_N_Error))
while i < len(open_N_Error):
    open_Error[i] = (open_N_Error[i]**2 + open_E_Error[i]**2)**(1/2)
    i = i + 1

# print(open_Northing)
# print(open_Easting)

alley_Northing = numpy.asarray(data_alley.UTM_northing - data_alley.UTM_northing[0])
alley_Easting = numpy.asarray(data_alley.UTM_easting - data_alley.UTM_easting[0])
alley_Known_UTM = utm.from_latlon(42.33775882649961, -71.08656966962846)
alley_Known = [float(alley_Known_UTM[1]), float(alley_Known_UTM[0])]
alley_N_Error = numpy.asarray(data_alley.UTM_northing - alley_Known[0])
alley_E_Error = numpy.asarray(data_alley.UTM_easting - alley_Known[1])

# print(data_alley.UTM_northing)
# print(alley_Known[0])
# print(data_alley.UTM_easting)
# print(alley_Known[1])

i = 0
alley_Error = numpy.zeros(len(alley_N_Error))
while i < len(alley_N_Error):
    alley_Error[i] = (alley_N_Error[i]**2 + alley_E_Error[i]**2)**(1/2)
    i = i + 1

walk_Northing = numpy.asarray(data_walk.UTM_northing - data_walk.UTM_northing[0])
walk_Easting = numpy.asarray(data_walk.UTM_easting - data_walk.UTM_easting[0])

# print(open_Error)
# print(alley_Error)

# print(walk_Northing[4:])
# print(walk_Easting[4:])
# print(data_open.UTM_easting)
# print(data_alley.UTM_northing)
# print(data_alley.UTM_easting)
# print(data_walk.UTM_easting)
# print(data_walk.UTM_easting)

plt.figure(1)
plt.scatter(open_Easting, open_Northing)
plt.title('UTM Northing vs. Easting Data on Top of Columbus Garage')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Analysis/open_UTM_scatter')

plt.figure(2)
plt.scatter(alley_Easting, alley_Northing)
plt.title('UTM Northing vs. Easting Data in Alley Between Columbus Garage and ISEC')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Analysis/alley_UTM_scatter')

bestfit_slope_num = (walk_Northing[len(walk_Northing)-1] - walk_Northing[0])
bestfit_slope_den = (walk_Easting[len(walk_Easting)-1] - walk_Easting[0])
bestfit_slope = bestfit_slope_num/bestfit_slope_den
bestfit = bestfit_slope * walk_Easting

plt.figure(3)
plt.scatter(walk_Easting, walk_Northing)
plt.plot(walk_Easting, bestfit)
plt.title('UTM Northing vs. Easting Data for a ~300 m Walk')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Analysis/walk_UTM_scatter')
# note i started holding the sensor differently at some point during the walk.

i = 0
walk_Error = numpy.zeros(len(walk_Northing))
while i < len(walk_Northing):
    walk_Error[i] = walk_Northing[i] - bestfit[i]
    i = i + 1
average_walk_error = numpy.average(walk_Error)
print(average_walk_error)


open_time = numpy.asarray(data_open.UTC - data_open.UTC[0])
open_alt = numpy.asarray(data_open.Altitude)
alley_time = numpy.asarray(data_alley.UTC - data_alley.UTC[0])
alley_alt = numpy.asarray(data_alley.Altitude)
walk_time = numpy.asarray(data_walk.UTC - data_walk.UTC[0])
walk_alt = numpy.asarray(data_walk.Altitude)

plt.figure(4)
plt.scatter(open_time, open_alt)
plt.ylim([0, 45])
plt.title('Altitude vs. Time Data on Top of Columbus Garage')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Analysis/open_alt')

plt.figure(5)
plt.scatter(alley_time, alley_alt)
plt.ylim([0, 65])
plt.title('Altitude vs. Time Data in Alley Between Columbus Garage and ISEC')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Analysis/alley_alt')

plt.figure(6)
plt.scatter(walk_time, walk_alt)
plt.title('Altitude vs. Time Data for a ~300 m Walk')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Analysis/walk_alt')

plt.figure(7)
countsopen, binsopen = np.histogram(open_Error)
plt.hist(binsopen[:-1], binsopen, weights=countsopen)
plt.title('Distribution of Error on Top of Columbus Garage')
plt.xlabel('UTM Northing (m)')
plt.ylabel('UTM Easting (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Analysis/open_hist')

plt.figure(8)
countsalley, binsalley = np.histogram(alley_Error)
plt.hist(binsalley[:-1], binsalley, weights=countsalley)
plt.title('Distribution of Error in Alley Between Columbus Garage and ISEC')
plt.xlabel('UTM Northing (m)')
plt.ylabel('UTM Easting (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/Analysis/alley_hist')




print(numpy.average(data_open.HDOP))
print(numpy.average(data_alley.HDOP))
print(numpy.average(data_walk.HDOP))
