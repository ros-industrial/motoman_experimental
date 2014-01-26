#!/usr/bin/env python

# Package.xml macro
# package - name of package
# model - robot model (typically uppercase)
# author - authors name
# author_email - authors email
def package_xacro(package, model, author, author_email):
  return """
<package xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:include filename="$(find motoman_package_generator)/xacro/package_macro.xacro" />
  <xacro:package_xml package=\"""" + package + """\" model=\"""" + model + """\" author=\"""" + author + """\" author_email=\"""" + author_email + """\"/>
</package>"""

# load_<model>.launch macro
# package - name of package
# model - robot model
LOAD_LAUNCH_MACRO="""
<launch>
	<param name="robot_description" command="$(find xacro)/xacro.py '$(find ${package})/urdf/${model}.xacro'" />
</launch>
"""
def load_launch(package, model):
  rtn = LOAD_LAUNCH_MACRO.replace("${package}", package)
  rtn = rtn.replace("${model}", model)
  return rtn

# robot_interface_streaming_<model>.launch macro
# package - name of package
# model - robot model
STREAMING_LAUNCH_MACRO="""
<!--
  Manipulator specific version of 'robot_interface_streaming.launch'.

  Defaults provided for ${model}:
   - 7 joints

  Usage:
    robot_interface_streaming_${model}.launch robot_ip:=<value> controller:=<fs100|dx100>
-->
<launch>
	<arg name="robot_ip" />
	
  <!-- controller: Controller name (fs100 or dx100) -->
  <arg name="controller"/>

	<rosparam command="load" file="$(find ${package})/config/joint_names_${model}.yaml" />

	<include file="$(find motoman_driver)/launch/robot_interface_streaming_$(arg controller).launch">
		<arg name="robot_ip"   value="$(arg robot_ip)" />
	</include>
</launch>
"""
def streaming_launch(package, model):
  rtn = STREAMING_LAUNCH_MACRO.replace("${package}", package)
  rtn = rtn.replace("${model}", model)
  return rtn
