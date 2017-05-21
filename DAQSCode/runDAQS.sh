#!/bin/bash
export DISPLAY=:0
NAME="daqs"
LOG_FILE="/home/pi/Desktop/$NAME.log"
#LOG_FILE="/dev/null"
PID_FILE="/home/pi/Desktop/$NAME.pid"
CMD="/usr/bin/python2 /home/pi/Desktop/Code/DAQSToFirebase.py"




function startnotification {
VAR=`ps -ef | grep "$CMD" | grep -v grep | wc -l`
if [ $VAR -gt 0 ]; then
echo "$NAME already running..."
else
nohup $CMD > $LOG_FILE 2>&1 &
echo $! > $PID_FILE
echo "$NAME listener is started..."
fi
}

function stopnotification {
kill `cat $PID_FILE`
rm -f $PID_FILE
echo "$NAME listener stopped."
}
case $1 in
start) startnotification;;
stop)  stopnotification;;
restart)
stopnotification
startnotification;;
*)
echo "usage: $NAME {start|stop}" ;;
esac
exit 0
