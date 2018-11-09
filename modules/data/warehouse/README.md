# Data Warehouse

We choose document based data warehousing, which performs well on composite
search criterias.

We defined the document as a proto, saying apollo::data::Record. It's easy to
convert proto to Json/Bson and vice versa.

## Setup MongoDB

For local testing, simply bring up a docker container:

```bash
docker run --name mongo -p 27017:27017 -d mongo
```

Advanced users should setup credentials and pass flags to mongo_util.py:

```bash
python <tool> \
    --mongo_host=x.x.x.x \
    --mongo_port=xxxx \
    --mongo_db_name=apollo \
    --mongo_user=xxxxxx \
    --mongo_pass=xxxxxx \
    <other arguments>
```

## Import Record

```bash
python tools/import_record.py <record_file>
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
