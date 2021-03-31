python auto-test/collision_detect.py &
python auto-test/fuzzer.py &
LASTPID=$!   
sleep 30; kill $LASTPID
python auto-test/metamporphic.py

