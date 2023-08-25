<package format="2">
  <name>canbus-vehicle-%(car_type_lower)s</name>
  <version>local</version>
  <description>
    Dynamic loading for canbus %(car_type_lower)s vehicle module
  </description>

  <maintainer email="apollo-support@baidu.com">Apollo</maintainer>
  <license>Apache License 2.0</license>
  <url type="website">https://www.apollo.auto/</url>
  <url type="repository">https://github.com/ApolloAuto/apollo</url>
  <url type="bugtracker">https://github.com/ApolloAuto/apollo/issues</url>

  <type>module</type>
  <src_path url="https://github.com/ApolloAuto/apollo">//modules/canbus_vehicle/%(car_type_lower)s</src_path>

  <depend type="binary" repo_name="cyber">cyber</depend>
  <depend type="binary" repo_name="common" lib_names="common">common</depend>
  <depend type="binary" repo_name="canbus" lib_names="canbus">canbus</depend>
  <depend type="binary" repo_name="common-msgs" lib_names="common-msgs">common-msgs</depend>
  <depend type="binary" repo_name="drivers-canbus" lib_names="drivers-canbus">drivers-canbus</depend>
  <depend repo_name="com_google_googletest" lib_names="gtest,gtest_main">3rd-gtest</depend>

</package>