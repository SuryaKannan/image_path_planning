#!/bin/bash
# Script for building and working in a docker container

attach_to_container() 
{
    # Allow docker windows to show on our current X Server
    xhost + >> /dev/null

    # Start the container in case it's stopped
    docker start $CONTAINER_NAME

    # Attach a terminal into the container
    exec docker exec -it $CONTAINER_NAME bash -c /home/guest/colcon_ws/src/image_path_planning/docker/tmux-start.sh
}

run_without_gpu()
{
    docker run -e DISPLAY -e TERM \
        --privileged \
        -v "/dev:/dev:rw" \
        -v "$(pwd):/home/guest/colcon_ws/src/image_path_planning:rw" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --net=host \
        --name $CONTAINER_NAME \
        --entrypoint /ros_entrypoint.sh \
        -d $IMAGE_NAME /usr/bin/tail -f /dev/null
}

build_image() 
{
    echo "Building docker image $IMAGE_NAME from $DOCKER_FILE"
    docker build . -t $IMAGE_NAME -f $DOCKER_FILE
}


CONTAINER_NAME=image_path_planning
IMAGE_NAME=image_path_planning
DOCKER_FILE=docker/Dockerfile

case "$1" in
"build")
    build_image
    ;;
"rm")
    docker rm -f $CONTAINER_NAME
    echo "Removed container"
    ;;
"--help")
    echo "Usage: docker/run.sh [command]
Available commands:
    run.sh
        Attach a new terminal to the container (pulling/building, creating and starting it if necessary)
    run.sh build
        Build a new image from the Dockerfile in the current directory
    run.sh rm
        Remove the current container
    run.sh --help
        Show this help message    
    "
    ;;
*) # Attach a new terminal to the container (pulling, creating and starting it if necessary)
    if [ -z "$(docker images -f reference=$IMAGE_NAME -q)" ]; then # if the image does not yet exist, build it
        build_image
    fi
    if [ -z "$(docker ps -qa -f name=$CONTAINER_NAME)" ]; then # if container has not yet been created, create it
        echo "Initialising without GPU support"
        run_without_gpu
    fi
    attach_to_container
    ;;
esac
