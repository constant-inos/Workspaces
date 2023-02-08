#!/bin/bash
my_models_folder='./gazebo_models_using'
gazebo_models_folder='/usr/share/gazebo-11/models/'

for object in $my_models_folder/*;
do
cp -r $object $gazebo_models_folder;
f="$(basename -- $object)"
sudo chmod -R +rwx $gazebo_models_folder/$f
done