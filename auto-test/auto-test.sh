# trap "kill 0" EXIT # kill all background process on exit

# run collision_detect.py and fuzzer.py in parallel 
round=50
curr_round=1

while [ $curr_round -le $round ]
do
    echo "Test round: $curr_round"
    echo "Source test case generating..."
    python auto-test/collision_detect.py src &
    python auto-test/fuzzer.py 

    # wait -n
    pkill -P $$

    echo "Follow-up test case generating..."
    python auto-test/collision_detect.py follow &
    python auto-test/metamorphic.py 

    # wait -n
    pkill -P $$
    
    # remove the old csv files
    rm -f /apollo/auto-test/data/obstacles.csv /apollo/auto-test/data/collision.csv /apollo/auto-test/data/collision_new.csv
    curr_round=$(( curr_round+1 ))
done
