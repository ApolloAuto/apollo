# Data Uploader

This is a tool to upload Apollo data from a folder to one or mutiple safe
storage, such as HDFS, NFS or a simple server with large disks. It also indexes
the data into a MongoDB which serves as
[Apollo Warehouse](https://github.com/ApolloAuto/apollo/tree/master/modules/data/warehouse).

## Working Process

The uploader works with following steps.

### Data Finding

We assume that all the tasks are recorded with Apollo standard recording tool,
which was triggered through Apollo Dreamview. Then all the tasks should be with
pattern `/data/bag/<YYYY-mm-DD-HH-MM-SS>` on the portable disk. If you record
data in other ways, make sure it's organized as we do.

This step finally generates a list of TODO tasks for the following steps.

### Data Cleanning

1. Index bags which are not indexed.

1. Remove useless tasks which are too small in size or short in time.

This step passes a list of *good condition* TODO tasks for the following steps.

### Additional Processing

Some users may want to process the data before or during uploading, such as
making a copy that filtered off all sensor data to make it much smaller.

### Data Uploading

We'll try to support as many storage types as possible. For example:

1. Local, or mounted NFS

   We call `cp` to copy data to a local path. You can leverage tools like
   [GlusterFS](https://docs.gluster.org) to setup your own distributed NFS.

1. Simple server

   We call `rsync` to transfer data to a remote server.

1. HDFS

   We call `hadoop fs -put` to upload data to an HDFS.

1. MongoDB

   We call `/apollo/modules/data/warehouse/importer/import_task.py` to index a
   task into MongoDB to serve queries.
