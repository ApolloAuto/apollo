# Data Warehouse

We choose document based data warehousing, which performs well on composite
search criterias.

We defined the document as a proto, saying apollo::data::Task. It's easy to
convert proto to Json/Bson and vice versa.

## Setup MongoDB

You need to setup your own MongoDB. Let's start a local instance for testing:

```bash
docker run --name mongo -p 27017:27017 -d mongo
```

Now you can run all tools without any special configuration.

Advanced users should pass proper value for flags defined in mongo_util.py:

```bash
python <tool> \
    --mongo_host=x.x.x.x \
    --mongo_port=xxxx \
    --mongo_db_name=apollo \
    --mongo_user=xxxxxx \
    --mongo_pass=xxxxxx \
    --collection=xxxxxx \
    <other arguments>
```

## Import Task

```bash
python importer/import_task.py \
    --mongo_host=x.x.x.x \
    --mongo_port=x \
    --mongo_db_name=apollo \
    --mongo_user=xxxxxx \
    --mongo_pass=xxxxxx \
    --collection=xxxxxx \
    <task_dir>
```

## Serve Data

```bash
python web_server/main.py \
    --mongo_host=x.x.x.x \
    --mongo_port=x \
    --mongo_db_name=apollo \
    --mongo_user=xxxxxx \
    --mongo_pass=xxxxxx \
    --collection=xxxxxx \
    --gmap_api_key=xxxxxx
```
