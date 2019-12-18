sudo update-alternatives --remove-all gcc
sudo update-alternatives --remove-all g++

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 50
sudo update-alternatives --install /usr/bin/cc cc /usr/bin/gcc-4.8 60
sudo update-alternatives --set cc /usr/bin/gcc
sudo update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++-4.8 60
sudo update-alternatives --set c++ /usr/bin/g++
