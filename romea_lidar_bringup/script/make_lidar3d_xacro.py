#!/usr/bin/env python3

import xml.etree.cElementTree as ET
import os
import sys
import glob

if __name__ == "__main__":

   argv = sys.argv

   options = {}
   for argument in argv[1:]:
        name, value = argument.split('=')
        options[name] = value

   xacro_element = ET.Element("robot")
   xacro_element.set("name",options['name'])
   xacro_element.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")

   ET.SubElement(
      xacro_element,
      "xacro:include",
      filename="$(find romea_lidar_description)/urdf/lidar3d.xacro",
   )

   lidar_xacro_element = ET.SubElement(xacro_element, "xacro:lidar3d_sensor")
   lidar_xacro_element.set("xyz", options["xyz"].strip('[]').replace(',', ' '))
   lidar_xacro_element.set("rpy", options["rpy"].strip('[]').replace(',', ' '))
   lidar_xacro_element.set("parent_link", options["parent_link"])
   lidar_xacro_element.set("rate", options["rate"])
   lidar_xacro_element.set("type", options["type"])
   lidar_xacro_element.set("model", options["model"])
   lidar_xacro_element.set("name", options["name"])
   lidar_xacro_element.set("organize_cloud","false")
   lidar_xacro_element.set("gpu","false")

   xacro_directory = os.path.expanduser(options["xacro_directory"])
   if not os.path.exists(xacro_directory) :
      os.makedirs(xacro_directory)

   xacro_filename = options['name']+".xacro"
   xacro_filename = os.path.join(xacro_directory,xacro_filename)

   xacro_tree = ET.ElementTree(xacro_element)
   xacro_tree.write(xacro_filename, xml_declaration=True, encoding="utf-8", method="xml")

   print(ET.tostring(xacro_element, encoding='unicode'))
