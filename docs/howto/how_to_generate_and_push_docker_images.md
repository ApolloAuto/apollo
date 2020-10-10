## How to Generate and Push Docker Images

 This How to will walk you through the basic steps of generating and pushing images through to your docker hub.

### Generate release image
 First, you would have to exit the Docker container by typing ```exit```

 Then, to create a new Docker image:

```bash
bash apollo_docker.sh gen
```
This will create a new docker image with the release directory. The release image will be named as *release-yyyymmdd_hhmm*. Meanwhile, your most recent built image will be tagged as *release-latest*.

### Push docker images
By default, the images will be pushed to Apolloauto/apollo Docker hub when you run the following command:
```bash
bash apollo_docker.sh push
```
You would need to upload it to your own docker hub, otherwise you would see an error
```bash
denied: requested access to resource is denied.
```
One way to solve this issue is by running the following command:
```bash
docker tag apolloauto/apollo:TAG_NAME YOUR_REPO:YOUR_TAGNAME
```
Now, login to gain access to your repository by following the process mentioned [here](https://docs.docker.com/engine/reference/commandline/login/#options)

Then, push your images to your own repository on Docker hub. You can refer to [this page](https://ropenscilabs.github.io/r-docker-tutorial/04-Dockerhub.html) for additional support.
