import numpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np
import utm
import math


# find point of intersection of vector to a point and ellipse
def ellipse_dist(alph, bet, px, py):
    m = py/px                                                          # slope of line that goes thru point
    x_contact = math.copysign(math.sqrt(1/(alph + bet * m**2)), px)    # plug y = mx into a*x^2 + b*y^2 = 1
    y_contact = math.copysign((m * x_contact), py)                     # plug x back into y = mx to solve for y

    print('Evaluation complete for point at ' + str(px) + ', ' + str(py))
    # print(str(x_contact) + ', ' + str(y_contact))
    return x_contact, y_contact


# find difference between vector to point and vector to intersection (error)
def ellipse_error(px, py, x_contact, y_contact):
    d2p = math.hypot(px, py)
    d2int = math.hypot(x_contact, y_contact)
    err = abs(d2p - d2int)

    print('The error at this point is ' + str(err))
    return err


# use bagreader to open rosbags
bag_stat_ber = bagreader('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Data//stationary_behrakis.bag')
bag_stat_cent = bagreader('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Data/stationary_centennial.bag')
bag_walk_ber = bagreader('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Data/walking_behrakis.bag')
bag_walk_cent = bagreader('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Data/walking_centennial.bag')

# decode the rosbags into csv format
decode_stat_ber = bag_stat_ber.message_by_topic('/gps')
print("File saved: {}".format(decode_stat_ber))
decode_stat_cent = bag_stat_cent.message_by_topic('/gps')
print("File saved: {}".format(decode_stat_cent))
decode_walk_ber = bag_walk_ber.message_by_topic('/gps')
print("File saved: {}".format(decode_walk_ber))
decode_walk_cent = bag_walk_cent.message_by_topic('/gps')
print("File saved: {}".format(decode_walk_cent))

# read csv files into tables
data_stat_ber = pd.read_csv(decode_stat_ber)
data_stat_cent = pd.read_csv(decode_stat_cent)
data_walk_ber = pd.read_csv(decode_walk_ber)
data_walk_cent = pd.read_csv(decode_walk_cent)

# define known values
centennial_known = utm.from_latlon(42.33711180570879, -71.09047204103692)
behrakis_known = utm.from_latlon(42.33706211673956, -71.0914038329504)

# create vectors with useful values
stat_ber_Northing = data_stat_ber.UTM_northing
stat_ber_Easting = data_stat_ber.UTM_easting
stat_ber_Fix = data_stat_ber.fix_quality

stat_cent_Northing = data_stat_cent.UTM_northing
stat_cent_Easting = data_stat_cent.UTM_easting
stat_cent_Fix = data_stat_cent.fix_quality

walk_ber_Northing = data_walk_ber.UTM_northing
walk_ber_Easting = data_walk_ber.UTM_easting
walk_ber_Fix = data_walk_ber.fix_quality

walk_cent_Northing = data_walk_cent.UTM_northing
walk_cent_Easting = data_walk_cent.UTM_easting
walk_cent_Fix = data_walk_cent.fix_quality

i = 0
stat_cent_Error = numpy.zeros(len(stat_cent_Northing))
while i < len(stat_cent_Northing):
    stat_cent_Error[i] = numpy.sqrt((stat_cent_Easting[i] - centennial_known[0])**2 +
                                    (stat_cent_Northing[i] - centennial_known[1])**2)
    i = i + 1

i = 0
stat_ber_Error = numpy.zeros(len(stat_ber_Northing))
while i < len(stat_ber_Northing):
    stat_ber_Error[i] = numpy.sqrt((stat_ber_Easting[i] - behrakis_known[0])**2 +
                                   (stat_ber_Northing[i] - behrakis_known[1])**2)
    i = i + 1

x = numpy.array(walk_cent_Easting[21:-31] - walk_cent_Easting[0])
y = numpy.array(walk_cent_Northing[21:-31] - walk_cent_Northing[0])

x_m = np.mean(x)
y_m = np.mean(y)

# this block of code sets up a system of linear eqns in matrix form to solve ax^2 + by^2 = 1 (ellipse eqn)
# i use np.linalg.lstsq to find the least squares solution for a and b in the ellipse eqn
# the x and y values are the points we measured on our walk
# the purpose of this is to find a best fit ellipse to our data
Ax = x**2   # the x values of the system
Ay = y**2   # the y values of the system
A = np.transpose(np.vstack([Ax, Ay]))   # tall matrix with x values in rhs and y values in lhs
b = np.ones_like(A)     # recall the ellipse equation, the b matrix is all ones
betas = np.linalg.lstsq(A, b, rcond=None)[0].squeeze()  # found the syntax online
print('Alpha and beta are equal to:' +
      '{0:.8}x^2 + {1:.8}y^2 = 1'.format(str(betas[0, 0]), str(betas[1, 0])))   # for debug

# for some reason, it output a 2x2 matrix... i really don't know how this works so here is a workaround:
alpha = float(betas[0, 0])
beta = float(betas[1, 0])

# create a grid of values so we can plot a contour
xs = np.linspace(-30.0, 30.0, 300)
ys = np.linspace(-20.0, 20.0, 300)
X, Y = np.meshgrid(xs, ys)

# function for plotting a contour
F = alpha*X**2 + beta*Y**2 - 1
# print(F.shape)

# s_maj = math.sqrt(1/alpha)/2
# s_min = math.sqrt(1/beta)/2
# print(str(s_maj) + ', ' + str(s_min))

# find distance between point and ellipse
# notably only works since my ellipse is centered at zero and is not tilted
# instead of error being in the x or y direction, i am using the r-direction to simplify my code
# i don't know how to find regression values for error in both x and y directions
i = 0
walk_cent_Error = np.zeros_like(x)
walk_cent_SSR = np.zeros_like(x)
walk_cent_SST = np.zeros_like(x)
walk_cent_Mean = np.asarray(np.mean(np.sqrt(x**2 + y**2)))
while i < len(walk_cent_Error):
    px_i = x[i]
    py_i = y[i]
    dx, dy = ellipse_dist(alpha, beta, px_i, py_i)
    walk_cent_Error[i] = ellipse_error(px_i, py_i, dx, dy)
    walk_cent_SSR[i] = walk_cent_Error[i]**2
    walk_cent_SST[i] = (np.sqrt(x[i]**2 + y[i]**2) - walk_cent_Mean)**2
    i = i + 1

walk_cent_COD = 1 - (np.sum(walk_cent_SSR)/np.sum(walk_cent_SST))
print('\n')
print('The coefficient of determination for the Elliptical Fit is ' + str(walk_cent_COD))
print('The average error on the walk around centennial is ' + str(np.mean(walk_cent_Error)))
# print(np.sum(walk_cent_SSR))
# print(np.sum(walk_cent_SST))

[m, b] = np.polyfit(walk_ber_Easting[90:-115], walk_ber_Northing[90:-115], 1)
walk_ber_BestFit = np.polyval([m, b], walk_ber_Easting[90:-115])
walk_ber_BestFit_Scaled = np.asarray(walk_ber_BestFit - walk_ber_BestFit[0])

i = 0
walk_ber_Northing_Scaled = np.asarray(walk_ber_Northing[90:-115] - walk_ber_Northing[90])
walk_ber_Error = numpy.zeros(len(walk_ber_Northing_Scaled))
walk_ber_SSR = numpy.zeros(len(walk_ber_Northing_Scaled))
walk_ber_SST = numpy.zeros(len(walk_ber_Northing_Scaled))

while i < len(walk_ber_Northing_Scaled):
    walk_ber_Error[i] = abs(walk_ber_Northing_Scaled[i] - walk_ber_BestFit_Scaled[i])
    walk_ber_SSR[i] = walk_ber_Error[i]**2
    walk_ber_SST = (walk_ber_Northing_Scaled[i] - np.mean(walk_ber_Northing_Scaled))**2
    i = i + 1

walk_ber_COD = 1 - (np.sum(walk_ber_SSR)/np.sum(walk_ber_SST))
print('\n')
print('The coefficient of determination for the linear regression is ' + str(walk_ber_COD))
print('The average error on the walk by Behrakis is ' + str(np.mean(walk_ber_Error)))

print('\n')
print('The average error stationary on Centennial is ' + str(np.mean(stat_cent_Error)))
print('The average error stationary by Behrakis is ' + str(np.mean(stat_ber_Error)))
print('\n')

# plot scaled easting vs northing for each data set
plt.figure(0)
plt.scatter(numpy.asarray(walk_cent_Easting[21:-38] - walk_cent_Easting[0]),
            numpy.asarray(walk_cent_Northing[21:-38] - walk_cent_Northing[0]))
plt.contour(X, Y, F, [0], colors='r', linewidths=2)
plt.grid(True)
plt.title('Clipped, Fitted UTM Northing vs. Easting Data on Walk Around Centennial')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Walk_Around_Centennial_Clipped')

plt.figure(10)
plt.scatter(numpy.asarray(walk_ber_Easting[90:-115] - walk_ber_Easting[90]),
            numpy.asarray(walk_ber_Northing[90:-115] - walk_ber_Northing[90]))
plt.plot(np.asarray(walk_ber_Easting[90:-115] - walk_ber_Easting[90]),
         walk_ber_BestFit_Scaled, color='r')
plt.grid(True)
plt.title('Clipped, Fitted UTM Northing vs. Easting Data on Walk by Behrakis')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Walk_Around_Behrakis_Clipped')

plt.figure(1)
plt.scatter(numpy.asarray(walk_cent_Easting - walk_cent_Easting[0]),
            numpy.asarray(walk_cent_Northing - walk_cent_Northing[0]))
plt.grid(True)
plt.title('UTM Northing vs. Easting Data on Walk Around Centennial')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Walk_Around_Centennial')

plt.figure(2)
plt.scatter(numpy.asarray(stat_cent_Easting - stat_cent_Easting[0]),
            numpy.asarray(stat_cent_Northing - stat_cent_Northing[0]))
plt.grid(True)
plt.title('UTM Northing vs. Easting Data, Stationary on Centennial')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Stationary_Centennial')

plt.figure(3)
plt.scatter(numpy.asarray(walk_ber_Easting - walk_ber_Easting[0]),
            numpy.asarray(walk_ber_Northing - walk_ber_Northing[0]))
plt.grid(True)
plt.title('UTM Northing vs. Easting Data on Walk Around Centennial')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Walk_Around_Behrakis')

plt.figure(4)
plt.scatter(numpy.asarray(stat_ber_Easting - stat_ber_Easting[0]),
            numpy.asarray(stat_ber_Northing - stat_ber_Northing[0]))
plt.grid(True)
plt.title('UTM Northing vs. Easting Data, Stationary by Behrakis')
plt.xlabel('UTM Easting (m)')
plt.ylabel('UTM Northing (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Stationary_Behrakis')

# plot histograms of error for stationary data sets
plt.figure(5)
countsopencent, binsopencent = np.histogram(stat_cent_Error)
plt.hist(binsopencent[:-1], binsopencent, weights=countsopencent)
plt.title('Distribution of Error, Stationary on Centennial')
plt.xlabel('Error (m)')
plt.ylabel('Counts (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Stationary_Centennial_Error')

plt.figure(6)
countsopencent, binsopencent = np.histogram(stat_ber_Error)
plt.hist(binsopencent[:-1], binsopencent, weights=countsopencent)
plt.title('Distribution of Error, Stationary by Behrakis')
plt.xlabel('Error (m)')
plt.ylabel('Counts')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Stationary_Behrakis_Error')

plt.figure(7)
countswalkber, binswalkber = np.histogram(walk_ber_Error)
plt.hist(binswalkber[:-1], binswalkber, weights=countswalkber)
plt.title('Distribution of Error, Walking by Behrakis')
plt.xlabel('Error (m)')
plt.ylabel('Counts')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Walking_Behrakis_Error')

plt.figure(8)
countswalkcent, binswalkcent = np.histogram(walk_cent_Error)
plt.hist(binswalkcent[:-1], binswalkcent, weights=countswalkcent)
plt.title('Distribution of Error, Walking by Centennial')
plt.xlabel('Error (m)')
plt.ylabel('Counts (m)')
plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Walking_Centennial_Error')

# print(data_stat_ber.info())
# print(data_stat_ber.__dir__())
# print(getattr(data_stat_ber, 'UTC.secs'))

stat_ber_Time = np.asarray(getattr(data_stat_ber, 'UTC.secs') - getattr(data_stat_ber, 'UTC.secs')[0])
stat_cent_Time = np.asarray(getattr(data_stat_cent, 'UTC.secs') - getattr(data_stat_cent, 'UTC.secs')[0])
walk_ber_Time = np.asarray(getattr(data_walk_ber, 'UTC.secs') - getattr(data_walk_ber, 'UTC.secs')[0])
walk_cent_Time = np.asarray(getattr(data_walk_cent, 'UTC.secs') - getattr(data_walk_cent, 'UTC.secs')[0])

fig, axs = plt.subplots(2, 2)
fig.subplots_adjust(hspace=.5)
fig.set_size_inches(8.5, 3)

axs[0, 0].plot(stat_ber_Time, stat_ber_Fix, color='r')
axs[0, 1].plot(stat_cent_Time, stat_cent_Fix, color='b')
axs[1, 0].plot(walk_ber_Time, walk_ber_Fix, color='g')
axs[1, 1].plot(walk_cent_Time, walk_cent_Fix, color='k')

fig.legend(['Behrakis Stationary', 'Centennial Stationary', 'Behrakis Walking', 'Centennial Walking'], loc=4)

axs[0, 0].grid(True)
axs[0, 1].grid(True)
axs[1, 0].grid(True)
axs[1, 1].grid(True)

fig.suptitle('GPS Fix Qualities Over Time')

axs[0, 0].set_ylabel('GPS Fix Quality')
axs[1, 0].set_ylabel('GPS Fix Quality')

axs[0, 0].set_xlabel('Time (s)')
axs[0, 1].set_xlabel('Time (s)')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 1].set_xlabel('Time (s)')

plt.savefig('/home/mattluongo/PycharmProjects/GPSDriverAnalysis/RTK_Analysis/Plots/Fix_Value_Plot')
