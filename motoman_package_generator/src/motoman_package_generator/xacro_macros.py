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
