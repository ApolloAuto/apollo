# How to Save and Load Apollo Docker Images

Considering that Apollo docker images are usually more than serveral Gigabytes,
it is better to download them over WIFI or other stable internet connections And
then copy the downloaded image to your vehicle.

## Save the Docker Image

After generating or downloading your docker image, you can save it as a local
tarball using the following command:

```bash
# docker save -o <path/to/saved/image/tar> <repo:tag>
docker save -o apollo_dev.tar apolloauto/apollo:dev-x86_64-18.04-20200823_0534
```

## Load the Docker Image

After copying the saved taball to your vehicle, you can load Apollo docker image
from it by running:

```bash
# docker load -i <path/to/docker/tarball>
docker load -i apollo_dev.tar
```
