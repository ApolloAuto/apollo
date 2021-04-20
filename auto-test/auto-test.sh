trap "kill 0" EXIT # kill all background process on exit

# run collision_detect.py and fuzzer.py in parallel 
python auto-test/routing_pub.py
python auto-test/collision_detect.py src &
DETECTPID=$!
python auto-test/fuzzer.py &
FUZZERPID=$!

# let the 2 scripts run for certain time
sleep 30; kill $DETECTPID; kill $FUZZERPID

python auto-test/routing_pub.py
python auto-test/collision_detect.py follow &
DETECTPID=$!
python auto-test/metamorphic.py &

# let the 2 scripts run for certain time
sleep 30; kill $DETECTPID
# remove the old csv files
rm /apollo/auto-test/data/obstacles.csv /apollo/auto-test/data/collision.csv /apollo/auto-test/data/collision_new.csv

