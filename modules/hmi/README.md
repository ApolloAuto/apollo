# Apollo HMI (Human Machine Interface)

### Deprecation Notice
HMI and Dreamview will be combined to one module soon. So you may see some
out-of-date as well as redundency code in these two modules during the
migration.

### Prerequisite
Make sure you are in the docker container, and running bash commands from Apollo
root directory.

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
Point your web browser to http://127.0.0.1:8887 (or your customized address) to
see the home page.

If you enabled HTTPS, the url should be like https://apollo.hmi:8887, which
varies according to your config.
