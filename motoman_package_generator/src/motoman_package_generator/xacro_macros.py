#!/usr/bin/env python

# CMakeList.txt macro
# package - name of the package
CMAKELIST_MACRO="""
cmake_minimum_required(VERSION 2.8.3)

project(${package})

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)
roslaunch_add_file_check(test/launch_test.xml)

foreach(dir config launch meshes urdf)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
"""

def cmakelist(package):
  rtn = CMAKELIST_MACRO.replace("${package}", package)
  return rtn


# Package.xml macro
# package - name of package
# model - robot model (typically uppercase)
# author - authors name
# author_email - authors email
PACKAGE_XML_MACRO="""
<?xml version="1.0" ?>
<package xmlns:xacro="http://ros.org/wiki/xacro">
 <name>${package}</name>
 <version>0.1.0</version>
 <description>
  <p>
      ROS Industrial support for the Motoman ${model} (and variants).
    </p>
  <p>
      This package contains configuration data, 3D models and launch files
      for Motoman ${model} manipulators. This currently includes on variants.
    </p>
  <p>
   <b>Specifications</b>
  </p>
  <ul>
   <li>${model} - Default</li>
  </ul>
  <p>
      Joint limits and maximum joint velocities are based on the information 
      found in the online 
      http://www.motoman.com/datasheets/${model}.pdf
      All urdfs are based on the default motion and joint velocity limits, 
      unless noted otherwise.
    </p>
  <p>
      Before using any of the configuration files and / or meshes included
      in this package, be sure to check they are correct for the particular
      robot model and configuration you intend to use them with.
    </p>
 </description>
 <author>${author}</author>
 <maintainer email="${author_email}"/>
 <license>BSD</license>
 <url type="website">http://ros.org/wiki/${package}</url>
 <url type="bugtracker">https://github.com/ros-industrial/motoman/issues</url>
 <url type="repository">https://github.com/ros-industrial/motoman</url>
 <buildtool_depend>catkin</buildtool_depend>
 <run_depend>motoman_driver</run_depend>
 <run_depend>robot_state_publisher</run_depend>
 <run_depend>rviz</run_depend>
 <run_depend>joint_state_publisher</run_depend>
 <export>
  <architecture_independent/>
 </export>
</package>
"""
def package_xml(package, model, author, author_email):
  rtn = PACKAGE_XML_MACRO.replace("${package}", package)
  rtn = rtn.replace("${model}", model)
  rtn = rtn.replace("${author}", author)
  rtn = rtn.replace("${author_email}", author_email)
  return rtn



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


# robot_state_visualize_<model>.launch macro
# package - name of package
# model - robot model
STATE_VIS_LAUNCH_MACRO="""
<!--
  Manipulator specific version of the state visualizer.

  Defaults provided for ${model}:
   - 7 joints

  Usage:
    robot_state_visualize_sia20d.launch robot_ip:=<value> controller:=<fs100|dx100>
-->
<launch>
	<arg name="robot_ip" />

  <!-- controller: Controller name (fs100 or dx100) -->
  <arg name="controller" />

	<rosparam command="load" file="$(find ${package})/config/joint_names_${model}.yaml" />

	<include file="$(find motoman_driver)/launch/robot_state_$(arg controller).launch">
		<arg name="robot_ip"   value="$(arg robot_ip)" />
	</include>

	<node name="robot_state_publisher" pkg="robot_state_publisher" 
		type="robot_state_publisher" />

	<include file="$(find ${package})/launch/load_${model}.launch" />

	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find industrial_robot_client)/config/robot_state_visualize.rviz" required="true" />
</launch>
"""
def state_vis_launch(package, model):
  rtn = STATE_VIS_LAUNCH_MACRO.replace("${package}", package)
  rtn = rtn.replace("${model}", model)
  return rtn

# test_<model>.launch macro
# package - name of package
# model - robot model
TEST_MODEL_LAUNCH_MACRO="""
<launch>
	<include file="$(find ${package})/launch/load_${model}.launch" />
	<param name="use_gui" value="true" />
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find industrial_robot_client)/config/robot_state_visualize.rviz" required="true" />
</launch>
"""
def test_model_launch(package, model):
  rtn = TEST_MODEL_LAUNCH_MACRO.replace("${package}", package)
  rtn = rtn.replace("${model}", model)
  return rtn

# launch_test.xml macro
# package - name of package
# model - robot model
LAUNCH_TEST_MACRO="""
<!--
  launch_test.xml - ROSlaunch tests

  Usage Instructions:

  1. Add the following to your CMakeLists.txt:
    
    find_package(roslaunch)
    roslaunch_add_file_check(test/launch_test.xml)

  2. Create a test directory under your package
  3. Add the "launch_text.xml" file and fill out the test below.  Use the
     following conventions:
    a. Encapsulate each launch file test in it's own namespace.  By
       convention the namespace should have the same name as the launch
       file (minus ".launch" extension)
    b. Create tests for each possible combination of parameters.  Utilize
       sub-namespaces under the main namespace.

  Notes:

  1. XML extension is used in order to avoid beinging included
  in roslaunch auto-complete.

  2. Group tags with namespaces are used to avoid node name collisions when
  testing multpile launch files
-->
<launch>
	<arg name="req_arg" value="default"/>
  <arg name="dx100" value="dx100"/>
  <arg name="fs100" value="fs100" />

  <group ns="load_${model}">
	  <include file="$(find ${package})/launch/load_${model}.launch"/>
  </group>

  <group ns="test_${model}">
	  <include file="$(find ${package})/launch/test_${model}.launch"/>
  </group>


  <group ns="robot_interface_streaming_${model}">
    <group ns="dx100" >
	    <include file="$(find ${package})/launch/robot_interface_streaming_${model}.launch">
		    <arg name="robot_ip"   value="$(arg req_arg)" />
        <arg name="controller" value="$(arg dx100)"/>
	    </include>
    </group>

    <group ns="fs100" >
	    <include file="$(find ${package})/launch/robot_interface_streaming_${model}.launch">
		    <arg name="robot_ip"   value="$(arg req_arg)" />
        <arg name="controller" value="$(arg fs100)"/>
	    </include>
    </group>
  </group>

  <group ns="robot_state_visualize_${model}">
    <group ns="dx100" >
	    <include file="$(find ${package})/launch/robot_state_visualize_${model}.launch">
		    <arg name="robot_ip"   value="$(arg req_arg)" />
        <arg name="controller" value="$(arg dx100)"/>
	    </include>
    </group>

    <group ns="fs100" >
	    <include file="$(find ${package})/launch/robot_state_visualize_${model}.launch">
		    <arg name="robot_ip"   value="$(arg req_arg)" />
        <arg name="controller" value="$(arg fs100)"/>
	    </include>
    </group>
  </group>

</launch>
"""

def launch_test(package, model):
  rtn = LAUNCH_TEST_MACRO.replace("${package}", package)
  rtn = rtn.replace("${model}", model)
  return rtn
