DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "${DIR}/install_gcc.sh"

ln -s /usr/bin/cc /usr/bin/cc1

chmod +x ./NVIDIA-Linux-x86_64-375.51.run
sudo ./NVIDIA-Linux-x86_64-375.51.run --no-opengl-files -a -s

source "${DIR}/recover_gcc.sh"
