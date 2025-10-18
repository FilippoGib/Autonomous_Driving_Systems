xhost +local:docker

docker compose up -d --build

#docker compose exec pcl_app /bin/bash

echo "-------------------------------------------------------------------"
echo "INFO: Exited container shell. The container remains running in the background."
echo "      Use 'docker compose stop' to stop it."
