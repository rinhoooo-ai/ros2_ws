#!/bin/bash

while true
do
    git add .
    git commit -m "auto commit"
    git push
    sleep 60
done