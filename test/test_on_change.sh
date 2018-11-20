#!/bin/sh

# apt-get install inotify-tools
while inotifywait -e close_write -r ../src .; do make && ./fusion_ekf; done

