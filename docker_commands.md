# Docker Commands


## IMAGES
- Downloading Images
```console
$ docker pull <image name>
```
- List all images
```console
$ docker images -a
```
- Delete single Image
```console
$ docker rmi <image id>
```
- All images delete
```console
$ docker rmi $(docker images  -q)
```
- Dangling images delete
```console
$ docker rmi $(docker images --filter "dangling=true" -q --no-trunc)
```



## CONTAINERS
- Creating a interactive container  from image
```console
$ docker run -it <image name>
```
- Giving Name to a container while creating
```console
$ docker run --name <container name> <image name>
```
- Start a stopped Container
```console
$ docker start (container_id)
```
- Stop all containers
```console
$ sudo docker kill $(sudo docker ps -a)
```
- Connect shell to running container
```console
$ docker exec -it (container_id) bash
```
- Connect shell to running container into workspace
```console
$ docker exec -w /home/<username> <container_name_or_id> <command_to_execute>
```
- Delete single Container
```console
docker rm <container id or container name>
```
- Delete all containers
```console
$ docker rm $(docker ps -a -q)
```
## Building Image from Docker File
- Terminal from same directory
```console
$ docker built -t <image name > .
```

## GUI arguments
### Windows

## Complete Examples
- Running a container with GUI enabled for Windows
```console
$ docker run -it --name r2_pathplanning_container -e DISPLAY=host.docker.internal:0.0 -e haiderabbasi333/ros2-pathplanning-course:1 bash

```

## Mistakes
- Mixing options
```console
$ docker run --name -it <container name> <image name>
```
here you should have provided name for **--name** before giving another option **-it**

