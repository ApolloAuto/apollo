# Voice Detection

## Usage

1. Start the system.

```bash
./scripts/bootstrap.sh start
```

2. Open dreamview at http://localhost:8888, and you'll see "Voice Command"
button on bottom left, which is disabled by default. Simply activate it and say
the commands. Currently we support:
```text
"Apollo, setup."
"Apollo, auto-mode."
"Apollo, disengage."
```

## Suported browsers

Chrome has strict security control. Only use it if you are accessing via
"localhost" or enabled HTTPS.

Firefox should be fine but you need to grand microphone permission everytime.

## Implementation

We capture audio from the frontend, a.k.a, Dreamview, and send the data to
backend in every 100ms.

The backend uses Snowboy hotword detector. Please refer to
https://github.com/Kitt-AI/snowboy for more information.
