# Docker Scripts for MacOS

**Hackers only!** Note that we always recommend to run Apollo on Ubuntu. Other
platforms may be not full featured and lack of testing.

## Prerequisites

1. Make sure you have installed docker-machine, docker and virtualbox.

```bash
brew install docker docker-machine
brew cask install virtualbox
```

2. Start a Linux based docker machine.

```bash
docker-machine create --driver virtualbox --virtualbox-memory 4096 apollo
```

You should allocate at least 4GB memory. But, of course, the more resources, the
better. For example:

```bash
docker-machine create --driver virtualbox \
    --virtualbox-cpu-count 2 \
    --virtualbox-memory 8192 \
    apollo
```

3. Bring up the container and start to work!

```bash
bash docker/scripts/mac/dev_start.sh
bash docker/scripts/mac/dev_into.sh

[Inside container] bash apollo.sh build
[Inside container] ...
```

## Known Hacks

1. You docker machine might go down in some situations such as suspending your
MacOS. You should bring it up again:

```bash
docker-machine restart apollo
```

2. Your docker machine is generally running behind a NAT. You can get the
address with:

```bash
docker-machine ip apollo
```

Remember to use it to access Apollo infras such as Dreamview, while not
"localhost" or "127.0.0.1".
