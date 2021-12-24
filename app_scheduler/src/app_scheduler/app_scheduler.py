import yaml

import rospy

import schedule

from app_manager.msg import AppList
from app_manager.msg import AppStatus
from app_manager.srv import StartApp
from app_manager.srv import StartAppRequest
from app_manager.srv import StopApp
from app_manager.srv import StopAppRequest


class AppScheduler(object):

    def __init__(self, robot_name, yaml_path, duration, update_duration):
        self.robot_name = robot_name
        self.yaml_path = yaml_path
        self.running_app_names = []
        self.running_jobs = {}
        self.app_list_topic_name = '/{}/app_list'.format(self.robot_name)
        self.start_app = rospy.ServiceProxy(
            '/{}/start_app'.format(self.robot_name), StartApp)
        self.stop_app = rospy.ServiceProxy(
            '/{}/stop_app'.format(self.robot_name), StopApp)
        self.job_timer = rospy.Timer(rospy.Duration(duration), self._timer_cb)
        self.update_timer = rospy.Timer(
            rospy.Duration(update_duration), self._update_timer_cb)
        self.sub = rospy.Subscriber(
            '/{}/application/app_status'.format(self.robot_name),
            AppStatus, self._sub_cb)
        self._load_yaml()
        self._register_apps()

    def _load_yaml(self):
        with open(self.yaml_path, 'r') as yaml_f:
            self.apps = yaml.load(yaml_f)

    def _register_apps(self):
        for app in self.apps:
            rospy.loginfo(
                'register app schedule => name: {0}, app_name: {1}'.format(
                    app['name'], app['app_name']))
            self._register_app(app)

    def _register_app(self, app):
        app_schedule = app['app_schedule']
        name = app['name']
        app_name = app['app_name']
        # default app_args is []
        if 'app_args' in app:
            app_args = app['app_args']
        else:
            app_args = []
        start_job = self._create_start_job(name, app_name, app_args)  # NOQA
        try:
            eval('schedule.{}.do(start_job)'.format(app_schedule['start']))
        except (AssertionError, ValueError) as e:
            rospy.logerr(e)
            rospy.logerr('Cannot register start app')
            rospy.logerr('Please upgrade schedule module. $ pip install schedule==0.6.0 --user')  # NOQA
        if 'stop' in app_schedule:
            stop_job = self._create_stop_job(name, app_name)  # NOQA
            try:
                eval('schedule.{}.do(stop_job)'.format(app_schedule['stop']))
            except ValueError as e:
                rospy.logerr(e)
                rospy.logerr('Cannot register stop app')
                rospy.logerr('Please upgrade schedule module. $ pip install schedule==0.6.0 --user')  # NOQA

    def _create_start_job(self, name, app_name, app_args):
        def start_job():
            start_req = StartAppRequest(
                name=app_name, args=app_args)
            start_res = self.start_app(start_req)
            if not start_res.started:
                rospy.logerr('Failed to start app: {}, {}, {}'.format(
                    name, app_name, app_args))
                rospy.logerr('StartApp error code: {}'.format(
                    start_res.error_code))
                rospy.logerr('StartApp error message: {}'.format(
                    start_res.message))
            self.running_jobs[name] = {
                'app_name': app_name,
                'running': start_res.started
            }
        return start_job

    def _create_stop_job(self, name, app_name):
        def stop_job():
            if app_name in self.running_app_names:
                stop_req = StopAppRequest(name=app_name)
                stop_res = self.stop_app(stop_req)
                if not stop_res.stopped:
                    rospy.logerr('Failed to stop app: {}, {}'.format(
                        name, app_name))
                    rospy.logerr('StopApp error code: {}'.format(
                        stop_res.error_code))
                    rospy.logerr('StopApp error message: {}'.format(
                        stop_res.message))
                self.running_jobs[name] = {
                    'app_name': app_name,
                    'running': not stop_res.stopped
                }
        return stop_job

    def _update_running_app_names(self):
        try:
            msg = rospy.wait_for_message(
                self.app_list_topic_name, AppList, timeout=1)
        except Exception as e:
            rospy.logwarn(
                'Failed to subscribe {}: {}'.format(
                    self.app_list_topic_name, e))
            return
        self.running_app_names = [x.name for x in msg.running_apps]

    def _update_running_jobs(self):
        for name, job_data in self.running_jobs.items():
            if (job_data['running']
                    and job_data['app_name'] not in self.running_app_names):
                self.running_jobs[name]['running'] = False

    def _timer_cb(self, event):
        try:
            schedule.run_pending()
        except TypeError as e:
            rospy.logerr(e)
            rospy.logerr('Cannot run pending app')
            rospy.logerr('Please upgrade schedule module. $ pip install schedule==0.6.0 --user')  # NOQA

    def _update_timer_cb(self, event):
        self._update_running_app_names()
        self._update_running_jobs()

    def _sub_cb(self, msg):
        if msg.type == AppStatus.INFO:
            # INFO
            rospy.loginfo('app_scheduler: {}'.format(msg.status))
        elif msg.type == AppStatus.WARN:
            # WARN
            rospy.logwarn('app_scheduler: {}'.format(msg.status))
        else:
            # ERROR
            rospy.logerr('app_scheduler: {}'.format(msg.status))
