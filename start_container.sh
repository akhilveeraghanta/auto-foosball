#######################################################################
#                           Start Container                           #
#######################################################################

# mount some extra stuff for display forwarding
docker container run --rm -it \
    --user $(id -u):$(id -g) \
    --env="DISPLAY" \
    --volume="$(pwd):/app:rw" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    autofoosball/roscore:latest \
    /bin/bash
