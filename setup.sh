#!/bin/bash
export $(grep -v '^#' .env | xargs -d '\n')

docker run --name postgres -v `pwd`/init:/docker-entrypoint-initdb.d -e POSTGRES_PASSWORD=$DB_PASSWORD -p 127.0.0.1:5432:5432/tcp -d postgres:15.2-alpine

pip3 install -r requirements.txt
#python3 ./setup.py