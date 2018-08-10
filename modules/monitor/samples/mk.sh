g++ -std=c++11 -fPIC -g -c -Wall plugin_test.cc
gcc -std=c++11 -shared -Wl,-soname,libplugin_test.so.1 -o libplugin_test.so.1.0.1 plugin_test.o -lc
g++ -std=c++11 -fPIC -g -Wall plugin_manager.cc -ldl
