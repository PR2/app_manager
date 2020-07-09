class AppManagerPlugin(object):

    """Base class for app_manager plugin

    This is a base class for app_manager plugin.
    app_manager plugin have two class methods;
    app_manager_start_plugin and app_manager_stop_plugin.
    app_manager_start_plugin runs before app starts,
    and app_manager_stop_plugin runs after app stops.

    app_manager plugin is defined in yaml format as below;
    - name: app_recorder/rosbag_recorder_plugin  # plugin name
      launch: app_recorder/rosbag_recorder.launch  # plugin launch name
      module: app_recorder.rosbag_recorder_plugin.RosbagRecorderPlugin
      # plugin module name

    Also, app_manager plugin yaml file is exported in package.xml as below;
     <export>
       <app_manager plugin="${prefix}/app_recorder_plugin.yaml" />
     </export>

    In app file, you can add plugin to your app as below;
    - plugins
      - name: mail_notifier_plugin  # name to identify this plugin
        type: app_notifier/mail_notifier_plugin  # plugin type
        launch_args:  # arguments for plugin launch file
          - foo: bar
        plugin_args:  # arguments for plugin function arguments
          - hoge: fuga

    Both class methods have 3 arguments;
    App definition, app context and app plugin arguments.
    App definition is the definition of app.
    App context is the shared information about app (app context)
    between plugins, such as app results and plugin results.
    App plugin arguments are the arguments for the module defined in app file
    and written as below;
    """

    def __init__(self):
        pass

    @classmethod
    def app_manager_start_plugin(cls, app, ctx, plugin_args):
        """Start plugin for app_manager

        Args:
            app (app_manager.AppDefinition): app definition
            ctx (dict): app context shared between plugins
            plugin_args (dict): arguments for plugin defined in app file
        """

        return ctx

    @classmethod
    def app_manager_stop_plugin(cls, app, ctx, plugin_args):
        """Stop plugin for app_manager

        Args:
            app (app_manager.AppDefinition): app definition
            ctx (dict): app context shared between plugins
            plugin_args (dict): arguments for plugin defined in app file
        """

        return ctx
