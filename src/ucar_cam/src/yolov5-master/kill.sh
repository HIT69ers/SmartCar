pid=`ps -ef | grep "roslaunch" | grep -v 'grep' | awk '{print $2}'`
if [ -n "$pid" ]
then
  kill -9 $pid
fi
pid=`ps -ef | grep "python" | grep -v 'grep' | awk '{print $2}'`
if [ -n "$pid" ]
then
  kill -9 $pid
fi
