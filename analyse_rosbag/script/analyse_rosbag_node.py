#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rosbag
import rospkg
import rospy
import tf

class AnalyseRosbags:
    def __init__(self):
        # determine directory of package for error messages
        rospack = rospkg.RosPack()
        pkg_dir = rospack.get_path('analyse_rosbag')
        # read rosparam and execute method
        if rospy.has_param('~rosbag_filepath'):
            self.bag_filepath = rospy.get_param('~rosbag_filepath')
            self.check_localization()
            self.plot_results()
        else:
            rospy.logerr("No filepath for rosbag was specified in launch file!\nSee %s", pkg_dir + '/launch')
    
    def read_rosbag_as_dataframe(self, topic):
        """ Reads the given rosbag and stores it as pandas dataframe
        with the column names 'time', 'topic' and 'message'."""
        bag = rosbag.Bag(self.bag_filepath)
        data_temp = [[],[],[]]
        for topic, msg, t in bag.read_messages(topics=topic):
            data_temp[0].append(t.to_nsec())
            data_temp[1].append(topic)
            data_temp[2].append(str(msg))
        bag.close()
        s_t = pd.Series(data_temp[0], name='time')
        s_topic = pd.Series(data_temp[1], name='topic')
        s_msg = pd.Series(data_temp[2], name='message')
        # concatenate the 3 series to 1 dataframe
        pf = pd.concat([s_t, s_topic, s_msg], axis=1)
        return pf

    def check_localization(self):
        """ Analyses the messages in given topics and determines,
        whether the localization yields good or bad results. """
        print('#####################################')
        print('### Localization report of rosbag ###')
        print('#####################################')
        print('---')

        topics = ['/ipa_log', '/tf']

        print('Topic: ', topics[0])
        # read the rosbag and store '/ipa_log' messages as dataframe
        df_1 = self.read_rosbag_as_dataframe(topics[0])
        # count the number of occurences of (un)trusty localizations
        count_loc_trusty = df_1['message'].str.count('Localization trusty!').sum()
        count_loc_untrusty = df_1['message'].str.count('Localization untrusty!').sum()
        # determine the average confidence of an (un)trusty localization
        vals_loc_trusty = df_1['message'].str.extractall(r'(?<="Localization trusty!\\nValue: )(.*?)(?=")')
        vals_loc_trusty[0] = vals_loc_trusty[0].astype(float, errors = 'raise')
        vals_loc_trusty_mean = vals_loc_trusty[0].mean()
        vals_loc_untrusty = df_1['message'].str.extractall(r'(?<="Localization untrusty!\\nLess than error bound: \d.\d\d\d\d\d\d\\nValue: )(.*?)(?=")')
        vals_loc_untrusty[0] = vals_loc_untrusty[0].astype(float, errors = 'raise')
        vals_loc_untrusty_mean = vals_loc_untrusty[0].mean()
        # determine error rate of localizations
        loc_error_rate = count_loc_untrusty / (count_loc_trusty + count_loc_untrusty)
        loc_error_rate_max = 0.1
        # extract the confidence values of all localizations chronologically
        self.vals_loc_all = df_1['message'].str.extractall(r'(?:.*(?:trusty).*)(?<=Value: )(.*?)(?=")')        
        self.vals_loc_all[0] = self.vals_loc_all[0].astype(float, errors = 'raise')
        # report
        print('Trusty localizations:\t{} with average confidence of \t{:.2f}'.format(count_loc_trusty, vals_loc_trusty_mean))
        print('Untrusty localizations:\t{} with average confidence of \t{:.2f}'.format(count_loc_untrusty, vals_loc_untrusty_mean))
        print('error rate:\t\t{:.0f}% (normal: < {:.0f}%)'.format(loc_error_rate*100, loc_error_rate_max*100))
        if loc_error_rate > loc_error_rate_max:
            print('--> Error rate of localization is too high to be normal!')
        print('---')

        # Reading rosbag as pandas Dataframe is inconvenient for /tf messages
        # Instead of self.read_rosbag(topics[1]) therefore this way
        
        print('Topic: ', topics[1])
        self.yaw_angle = []
        self.xy_pos = []
        # read the rosbag and directly store relevant data from /tf in lists
        bag = rosbag.Bag(self.bag_filepath)
        for _, msg, _ in bag.read_messages(topics=topics[1]):
            if msg.transforms[0].header.frame_id == "odom_combined":
                msg_rot = msg.transforms[0].transform.rotation
                msg_trans = msg.transforms[0].transform.translation
                # determine the yaw angle from a quaternion and store it
                quat_temp = (msg_rot.x, msg_rot.y, msg_rot.z, msg_rot.w)
                euler_temp = tf.transformations.euler_from_quaternion(quat_temp)
                self.yaw_angle.append(euler_temp[2])
                # store xy position
                self.xy_pos.append([msg_trans.x, msg_trans.y])
        self.xy_pos = np.array(self.xy_pos)
        print('--> See figure to investigate result')

    def plot_results(self):
        """ Plots the results determined by method check_localization():
        1st subplot: localization confidence from /ipa_log
        2nd subplot: yaw angle from /tf-->odom_combined
        3rd subplot: xy position from /tf-->odom_combined"""
        fig = plt.figure(figsize=(6, 4))
        sub1 = fig.add_subplot(311)
        sub1.set_title("Localization Confidence")
        sub1.plot(self.vals_loc_all.to_numpy())
        sub2 = fig.add_subplot(312)
        sub2.set_title("Yaw Angle (odom_combined)")
        sub2.plot(self.yaw_angle)
        sub3 = fig.add_subplot(313)
        sub3.set_title("Position (odom_combined)")
        sub3.plot(self.xy_pos[:,0], self.xy_pos[:,1])
        sub3.axis('equal')
        plt.show()

if __name__ == '__main__':
    rospy.init_node('analyse_rosbag_node', anonymous=True)
    AnalyseRosbags()
