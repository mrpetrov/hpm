#!/bin/bash
service_name=hpm
src_dir=/home/pi/$service_name

service $service_name stop
cp $src_dir/$service_name /usr/sbin
chown root:root /usr/sbin/$service_name
chmod a+x /usr/sbin/$service_name
sleep 6
service $service_name start

