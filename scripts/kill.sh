echo "kill ${1}"
ps uax | grep ${1} | grep -v grep | awk '{print$2}'|xargs kill -9

