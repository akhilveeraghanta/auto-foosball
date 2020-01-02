# run the container in detatched mode so that we can attatch and run our program
# eventually the container will start and just run the program, this is just a stop gap
# until the docker containers are setup correctly
docker container stop autofooscontainer
docker container rm autofooscontainer
docker run -it --name autofooscontainer -d autofoosball/roscore /bin/bash
docker exec -it autofooscontainer /bin/bash

