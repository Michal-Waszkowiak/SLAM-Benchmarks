xhost +local:root

docker run --init -it -d \
  --name liosam \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  # --volume="$HOME/Studia/Autonomiczne_Samochody/Share:/Share" \ <--- Tutaj były trzymane rosbagi
  --runtime=nvidia --gpus all \
  waszini/lio-sam:v1 \
  bash

