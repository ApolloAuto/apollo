# How to save and load docker image
Considering the docker image is more than one Gigabyte, it is better to generate or download the docker image in the WIFI environment and then copy the image to cars.

## Save Docker Image

After generating or downloading a docker image, you can save the docker image into a local tar file.

e.g.

```
# docker save -o <save image to path> <image name>
docker save -o apollo_img.tar apolloauto/apollo:dev-latest
```

## Load Docker Image

After copying the tar file to the car, you need to load the docker image from the tar file.

e.g.

```
# docker load -i <path to image tar file>
docker load -i apollo_img.tar
```
