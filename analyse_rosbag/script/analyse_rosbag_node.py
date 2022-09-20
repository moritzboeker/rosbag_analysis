#!/usr/bin/env python3

# TODO
# use ros params
# interesting topics
#### /ipa_log
#### /lts_confidence_marker
#### /tf mit frame odom_combined
#### /long_term_slam/error_code

from itertools import count
import pandas as pd
import rosbag
import rospkg
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class AnalyseRosbags:
    def __init__(self):
        rospack = rospkg.RosPack()
        pkg_dir = rospack.get_path('analyse_rosbag')
        self.bag_dir = pkg_dir + '/data/dummy_env-agv-50231.agv-2020-10-01T082312+0200_2020-10-01-11-28-17_37.bag'
        self.check_localization()
    
    def read_rosbag(self, topic):
        bag = rosbag.Bag(self.bag_dir)
        data_temp = [[],[],[]]
        for topic, msg, t in bag.read_messages(topics=topic):
            data_temp[0].append(t.to_nsec())
            data_temp[1].append(topic)
            data_temp[2].append(str(msg))
        bag.close()
        s_t = pd.Series(data_temp[0], name='time')
        s_topic = pd.Series(data_temp[1], name='topic')
        s_msg = pd.Series(data_temp[2], name='message')
        pf = pd.concat([s_t, s_topic, s_msg], axis=1)
        return pf

    def check_localization(self):
        print('### Localization report of rosbag ###')
    
        topics = ['/ipa_log', '/tf']
        df_1 = self.read_rosbag(topics[0])
        loc_trusty = df_1['message'].str.count('Localization trusty!').sum()
        loc_untrusty = df_1['message'].str.count('Localization untrusty!').sum()
        loc_error_rate = loc_untrusty / (loc_trusty + loc_untrusty)
        print('Topic {}\ntrusty localizations:\t{}\nuntrusty localizations:\t{}\nerror rate:\t\t{:.0f}%'.format(topics[0], loc_trusty, loc_untrusty, loc_error_rate*100))
        if loc_error_rate > 0.1:
            print('Error rate is too high to be normal!')

if __name__ == '__main__':
    AnalyseRosbags()
