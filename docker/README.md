You can build the docker image by downloading the `Dockerfile` in the folder and then build it:
```
mkdir dismesh_docker
cd dismesh_docker
wget https://raw.githubusercontent.com/StructuresComp/dismech-rods/refs/heads/main/docker/Dockerfile
docker build -t dismesh .
```
After this you can run the example using
```
xhost +
docker run -it --rm --env DISPLAY --ipc=host --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --volume=/dev/dri:/dev/dri:rw dismech /dismech-rods/examples/spider_case/spiderExample
```

Or run the python version using
```
xhost +
docker run -it --rm --env DISPLAY --ipc=host --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --volume=/dev/dri:/dev/dri:rw dismech python3 /dismech-rods/py_examples/spider_case/spiderExample.py
```


# TODO:
* fix version of dependencies
