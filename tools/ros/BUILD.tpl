package(default_visibility = ["//visibility:public"])

%{ros_distro_gen_rules}

%{copy_rules}

%{ws_copy_rules}

%{cc_libraries}

%{ws_cc_libraries}

%{ros_interface}