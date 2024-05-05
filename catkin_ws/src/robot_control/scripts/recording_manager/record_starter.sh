#!/bin/bash
# make fifo
set -e
FIFO_FILE="./in.tmpfifo"
cleanup() {
  echo "save recording"
  echo "record-pause" > $FIFO_FILE
  echo "record-save" > $FIFO_FILE
  sleep 1
  echo "recording saved"
  echo "quit" > $FIFO_FILE
  echo "Removing FIFO_FILE"
  rm  -r $FIFO_FILE
}

trap cleanup 2
if  [ -p $FIFO_FILE ]; then
  rm -r $FIFO_FILE
fi
mkfifo $FIFO_FILE

# start recording
tail -f $FIFO_FILE | simplescreenrecorder --start-recording --start-hidden &> ./screenrecording_log.txt &

wait
