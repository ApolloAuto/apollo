# Installers to Build Docker Image

The installers should be standalone, which means it can finish successfully with
minimum assumption of the base image.

## Minimum assumption

The only thing you can assume is that we have run the `pre_install.sh`. All
other dependencies should be installed by your own installer.

## Test

Run the following command to test if an installer works well during image
building process.

```bash
# Test single or multiple installers.
./test_installer.sh <install_xxx.sh> [install_yyy.sh ...]

# Test all installers.
./test_installer.sh install_*.sh
```
