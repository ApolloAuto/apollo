# Apollo HMI (Human Machine Interface)


### Prerequisite
Make sure you have installed the required python packages by:
```bash
# All bash commands should be run from Apollo root directory.
sudo pip install modules/tools/py27_requirements.txt
```


### Start the server
Start the server with default settings by:
```bash
bash scripts/hmi.sh
```

Experts may want to have more control:
```bash
source scripts/apollo_base.sh
python modules/hmi/web/hmi_main.py \
    --conf=<your/conf/file> \
    --host=0.0.0.0 --port=8899
```

To get more help information:
```bash
source scripts/apollo_base.sh
python modules/hmi/web/hmi_main.py --help|--helpfull
```


### View HMI
Point your web browser to http://localhost:8887 (or your customized address) to
see the home page.
