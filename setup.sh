#!/bin/bash
export $(grep -v '^#' .env | xargs -d '\n')

docker run --name postgres -e POSTGRES_PASSWORD=$DB_PASSWORD -p 127.0.0.1:5432:5432/tcp -d postgres:15.2-alpine