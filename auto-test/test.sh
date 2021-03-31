python auto-test/test.py &
LASTPID=$!        
sleep 5; kill $LASTPID
