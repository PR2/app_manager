# this node shows app information
display: Test app plugin
platform: all
launch: app_manager/sample_node.xml
interface: app_manager/test_plugin.interface
timeout: 0.5
plugins:
  - name: test_plugin  # name to identify this plugin
    type: test_plugin/test_plugin  # plugin type, defiend in plugin.yml: name
    launch_args:  # arguments for plugin launch file, defined in plugin.yml: launch
      param1: hello
      param2: world
    plugin_args:  # default arguments for plugin function
      hoge: 10
    start_plugin_args:  # arguments for start plugin function
      hoge: 100  # arguments for start plugin function arguments (it overwrites plugin_args hoge: 10 -> 100)
      fuga: 300  # arguments for start plugin function arguments (it overwrites plugin_args hoge: 10 -> 100)
    stop_plugin_args:  # arguments for stop plugin function
      hoge: 1000  # arguments for start plugin function arguments (it overwrites plugin_args fuga: 300 -> 1000)
      fuga: 3000  # arguments for start plugin function arguments (it overwrites plugin_args fuga: 300 -> 1000)
