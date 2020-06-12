#!/bin/bash
start_client() {
./../bin/demo  \
    --data_path ./files.list_video \
    --src_frame_rate 10   \
    --wait_time 0 \
    --rtsp=false \
    --loop=false \
    --config_fname "client.json" \
    --alsologtostderr \
    --perf_db_dir="./perf_client"
}

start_server() {
./../bin/demo  \
    --data_path ./files.list_video \
    --src_frame_rate 10   \
    --wait_time 0 \
    --rtsp=false \
    --loop=false \
    --config_fname "server.json" \
    --alsologtostderr \
    --perf_db_dir="./perf_server"
}

source env.sh
mkdir -p output
start_client & start_server
wait
