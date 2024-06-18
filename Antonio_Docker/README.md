This example Dockerfile is from the following tutorial: xxxx.

It borrows heavily from [Allison Thackston's Dockerfile repo](https://github.com/athackst/dockerfiles) and the [OSRF Docker images](https://github.com/osrf/docker_images) were also used for inspiration.

# docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ACM0 -b 115200