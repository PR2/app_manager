from app_manager import AppManagerPlugin
from std_msgs.msg import String
import rospy

class TestPlugin(AppManagerPlugin):
    def __init__(self):
        super(TestPlugin, self).__init__()
        self.pub = rospy.Publisher('/test_plugin', String)
        #wait for listener
        while self.pub.get_num_connections() == 0:
            rospy.logwarn('wait for subscriber...')

    def app_manager_start_plugin(self, app, ctx, plugin_args):
        self.start_time = rospy.Time.now()
        self.pub.publish("{{'start_plugin': {}}}".format(plugin_args))

    def app_manager_stop_plugin(self, app, ctx, plugin_args):
        self.pub.publish("{{'stop_plugin': {}}}".format(plugin_args))
        ctx['test_app_exit_code'] = 0
        return ctx
