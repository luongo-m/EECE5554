import bagpy
from bagpy import bagreader

bag_openarea = bagreader('~/EECE5554/LAB1/Data/topofcolumbusparkinggarage.bag')
bag_alleyway = bagreader('~/EECE5554/LAB1/Data/alleybetweenisecandcolumbus.bag')
bag_walk = bagreader('~/EECE5554/LAB1/Data/300mstraightwalk.bag')

data_openarea = bag_openarea.message_by_topic('gps')
print("File saved: {}".format(data_openarea))

data_alleyway = bag_alleyway.message_by_topic('gps')
print("File saved: {}".format(data_alleyway))

data_walk = bag_walk.message_by_topic('gps')
print("File saved: {}".format(data_walk))