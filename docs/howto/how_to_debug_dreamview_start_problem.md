## How to Debug a Dreamview Start Problem

### Steps to start Dreamview

If you encounter problems when starting Dreamview in the `docker/scripts/dev` sequence, first check if you are using the correct commands as shown below.

```bash
$ bash docker/scripts/dev_start.sh
$ bash docker/scripts/dev_into.sh
$ cd /apollo
$ bash apollo.sh build
$ bash scripts/dreamview.sh
```
### Dreamview Fails to Start

If Dreamview fails to start, use the script below to check the Dreamview startup log and restart Dreamview.

```bash
# check dreamview startup log
$ cat data/log/dreamview.out
terminate called after throwing an instance of 'CivetException'
  what():  null context when constructing CivetServer. Possible problem binding to port.

$ sudo apt-get install psmisc

# to check if dreamview is running from other terminal
$ sudo lsof -i :8888

# kill other running/pending dreamview
$ sudo fuser -k 8888/tcp

# restart dreamview again
$ bash scripts/dreamview.sh
```
