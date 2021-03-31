trap "kill 0" EXIT

python auto-test/collision_detect.py &
DETECTPID=$!
python auto-test/fuzzer.py &
FUZZERPID=$!   
sleep 30; kill $DETECTPID; kill $FUZZERPID
python auto-test/metamorphic.py

