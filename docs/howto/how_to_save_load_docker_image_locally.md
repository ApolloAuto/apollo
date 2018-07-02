# How to save and load your docker image
Considering the docker image is more than one Gigabyte, it is better to generate or download the docker image using WiFi or a very stable internet connection, and then copy the image to your car.

## Save the Docker Image

After generating or downloading a docker image, you can save the docker image into a local tar file.

e.g.

```
# docker save -o <save image to path> <image name>
docker save -o apollo_img.tar apolloauto/apollo:release-latest
```

## Load the Docker Image

After copying the .tar file to the car, you need to load the docker image from the .tar file as shown below:

```
# docker load -i <path to image tar file>
docker load -i apollo_img.tar
```
