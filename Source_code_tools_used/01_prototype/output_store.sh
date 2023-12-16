#!/bin/bash

#foldername="data_$(date +%Y_%m_%d_%H_%M_%S)"
foldername="data_$1"
echo $foldername

mkdir -p ./data/$foldername

mv ./output/* ./data/$foldername
echo "data stored"
