if [ -z $1 ] || [ -z $2 ]; then
    echo "[Usage] replay.sh target_binary testcase_folder"
fi
for file in $2/*
do
    $1 "$file"
done

