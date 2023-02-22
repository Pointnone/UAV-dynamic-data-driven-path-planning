#!/bin/bash
export $(grep -v '^#' .env | xargs -d '\n')

docker run --name postgres -v `pwd`/init:/docker-entrypoint-initdb.d -e POSTGRES_PASSWORD=$DB_PASSWORD -p 127.0.0.1:5432:5432/tcp -d postgis/postgis:15-3.3-alpine

pip3 install -r requirements.txt
sudo apt install postgresql-client
sudo apt install libeccodes0

echo $DB_PASSWORD > ~/.pgpass
export PGPASSWORD=$DB_PASSWORD

psql -d geomdata -f ./enableGIS.sql -h localhost -p 5432 -U postgres