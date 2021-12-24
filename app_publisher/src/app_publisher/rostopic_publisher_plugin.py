import importlib

from app_manager import AppManagerPlugin
import rospy
from rospy_message_converter import message_converter


class RostopicPublisherPlugin(AppManagerPlugin):
    def __init__(self):
        super(RostopicPublisherPlugin, self).__init__()

    def publish_topic(self, topic, ctx):
        if 'cond' in topic:
            if ((topic['cond'] == 'success' and ctx['exit_code'] == 0)
                    or (topic['cond'] == 'failure' and ctx['exit_code'] != 0)
                    or (topic['cond'] == 'stop' and ctx['stopped'] is True)
                    or (topic['cond'] == 'timeout' and ctx['stopped'] is True
                        and ctx['timeout'] is True)):
                pass
            else:
                return
        msg = getattr(
            importlib.import_module(
                '{}.msg'.format(topic['pkg'])), topic['type'])
        pub = rospy.Publisher(topic['name'], msg, queue_size=1)
        rospy.sleep(1)
        if 'field' in topic:
            pub_msg = message_converter.convert_dictionary_to_ros_message(
                '{}/{}'.format(topic['pkg'], topic['type']),
                topic['field'])
        else:
            pub_msg = msg()
        pub.publish(pub_msg)

    def app_manager_start_plugin(self, app, ctx, plugin_args):
        if 'start_topics' not in plugin_args:
            return
        topics = plugin_args['start_topics']
        for topic in topics:
            self.publish_topic(topic, ctx)

    def app_manager_stop_plugin(self, app, ctx, plugin_args):
        if 'stop_topics' not in plugin_args:
            return
        topics = plugin_args['stop_topics']
        for topic in topics:
            self.publish_topic(topic, ctx)
