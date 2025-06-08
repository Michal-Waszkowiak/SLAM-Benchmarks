xhost +local:root

# BUILD THE IMAGE
ROS_IMAGE="waszini/orbslam3:v1"
if [ -z "$1" ]
	then
		echo "RUNNING WITH 1 WORKER"
		WORKERS=1
	else
		echo "RUNNING WITH $1 WORKERS"
		WORKERS=$1
fi
docker build --no-cache -f Dockerfile --build-arg parallel_workers=$WORKERS -t $ROS_IMAGE ./..
