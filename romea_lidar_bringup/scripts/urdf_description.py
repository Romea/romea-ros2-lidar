#!/usr/bin/env python3
import lidar_bringup
import sys

if __name__ == "__main__":

  argv = sys.argv

  parameters = {}
  for argument in argv[1:]:
       name, value = argument.split(':')
       parameters[name] = value

  if not parameters["robot_namespace"] :
    prefix="";
  else:
    prefix=parameters["robot_namespace"]+"_";

  description_yaml_file=parameters["description_yaml_file"]

  print(lidar_bringup.urdf_description(prefix,description_yaml_file))
