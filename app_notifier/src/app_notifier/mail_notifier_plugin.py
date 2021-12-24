import datetime
import subprocess

from app_manager import AppManagerPlugin
import rospy

from app_notifier.util import check_timestamp_before_start
from app_notifier.util import get_notification_json_paths
from app_notifier.util import load_notification_jsons


class MailNotifierPlugin(AppManagerPlugin):
    def __init__(self):
        super(MailNotifierPlugin, self).__init__()

    def app_manager_start_plugin(self, app, ctx, plugin_args):
        self.start_time = rospy.Time.now()

    def app_manager_stop_plugin(self, app, ctx, plugin_args):
        mail_title = plugin_args['mail_title']
        sender_address = plugin_args['sender_address']
        receiver_address = plugin_args['receiver_address']
        use_timestamp_title = False
        if 'use_timestamp_title' in plugin_args:
            use_timestamp_title = plugin_args['use_timestamp_title']

        if use_timestamp_title:
            timestamp = '{0:%Y/%m/%d (%H:%M:%S)}'.format(
                datetime.datetime.now())
            mail_title += ': {}'.format(timestamp)

        display_name = app.display_name
        mail_content = "Hi, \\n"
        if ctx['exit_code'] == 0 and not ctx['stopped']:
            mail_content += "I succeeded in doing {}.\\n".format(display_name)
        elif ctx['stopped']:
            mail_content += "I stopped doing {}.\\n".format(display_name)
        else:
            mail_content += "I failed to do {}.\\n".format(display_name)
        if 'upload_successes' in ctx:
            if all(ctx['upload_successes']):
                mail_content += "I succeeded to upload data.\\n"
            else:
                mail_content += "I failed to upload data.\\n"
            mail_content += "\\n"
            for success, file_url in zip(
                    ctx['upload_successes'], ctx['upload_file_urls']):
                if success:
                    mail_content += "URL: {}\\n".format(file_url)
        mail_content += "\\n"

        json_paths = get_notification_json_paths()
        notification = load_notification_jsons(json_paths)
        for n_type in notification.keys():
            mail_content += "Following {} is reported.\\n".format(n_type)
            for event in notification[n_type]:
                if check_timestamp_before_start(
                        event['date'], self.start_time):
                    continue
                if event['location'] == "":
                    mail_content += " - At {}, {}.\\n".format(
                        event['date'], event['message'])
                else:
                    mail_content += " - At {}, {} in {}.\\n".format(
                        event['date'], event['message'], event['location'])
            mail_content += "\\n"

        cmd = "LC_CTYPE=en_US.UTF-8 /bin/echo -e \"{}\"".format(mail_content)
        cmd += " | /usr/bin/mail -s \"{}\" -r {} {}".format(
            mail_title, sender_address, receiver_address)
        exit_code = subprocess.call(cmd, shell=True)
        rospy.loginfo('Title: {}'.format(mail_title))
        if exit_code > 0:
            rospy.logerr(
                'Failed to send e-mail:  {} -> {}'.format(
                    sender_address, receiver_address))
            rospy.logerr("You may need to do '$ sudo apt install mailutils'")
        else:
            rospy.loginfo(
                'Succeeded to send e-mail: {} -> {}'.format(
                    sender_address, receiver_address))
        ctx['mail_notifier_exit_code'] = exit_code
        return ctx
