#!/bin/bash
export $(grep -v '^#' .env | xargs -d '\n')

docker run --name postgres -v `pwd`/init:/docker-entrypoint-initdb.d -e POSTGRES_PASSWORD=$DB_PASS -p 127.0.0.1:5432:5432/tcp -d postgis/postgis:15-3.3-alpine

ln -s `pwd`/.env ./ros_ws # Symlink .env file to ros_ws for easier use of ros2 run

pip install -r requirements.txt
sudo apt install postgresql-client
sudo apt install libeccodes0

# ROS2 (Not done)
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions

echo $DB_PASS > ~/.pgpass
export PGPASSWORD=$DB_PASS

psql -d geomdata -f ./enableGIS.sql -h localhost -p 5432 -U postgres