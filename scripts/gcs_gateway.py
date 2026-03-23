#!/usr/bin/env python
# ROS Noetic - GCS Gateway
# Reads topics from config/topics.yaml and throttles them to lower frequencies
# for the Ground Control Station using topic_tools/throttle.

import rospy
import subprocess
import os
import rospkg
import yaml


def load_topics():
    rp = rospkg.RosPack()
    pkg_path = rp.get_path('onboard_dji')
    config_path = os.path.join(pkg_path, 'config', 'topics.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    return config.get('topics_to_filter', [])


def main():
    rospy.init_node('gcs_gateway', anonymous=False)

    topics = load_topics()

    if not topics:
        rospy.logwarn('gcs_gateway: no topics defined in topics.yaml')
        return

    ns = rospy.get_namespace().rstrip('/')
    processes = []

    for item in topics:
        source = '{}/{}'.format(ns, item['source'].lstrip('/'))
        target = '{}/{}'.format(ns, item['target'].lstrip('/'))
        hz = float(item['hz'])

        # rosrun topic_tools throttle messages <source> <hz> <target>
        cmd = ['rosrun', 'topic_tools', 'throttle', 'messages', source, str(hz), target]
        rospy.loginfo('gcs_gateway: {} -> {} @ {} Hz'.format(source, target, hz))
        proc = subprocess.Popen(cmd)
        processes.append(proc)

    rospy.on_shutdown(lambda: [p.terminate() for p in processes])
    rospy.spin()


if __name__ == '__main__':
    main()
