python auto-test/collision_detect.py &
python auto-test/fuzzer.py &
LASTPID=$!   
sleep 10; kill $LASTPID

