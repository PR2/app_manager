#! /usr/bin/env python

"""
usage: %prog [args]
"""

import roslib
roslib.load_manifest('app_manager')
import rospy

import os, sys, string
from optparse import OptionParser

import app_manager

def main(argv, stdout, environ):

  parser = OptionParser(__doc__.strip())
  (options, args) = parser.parse_args()

  try:
    a = app_manager.App(args[0])
    rospy.logwarn(a.yaml)
  except app_manager.TaskException as e:
    rospy.logerr(e)

if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
