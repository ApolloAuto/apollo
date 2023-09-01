import { webSocket } from 'rxjs/webSocket';
import ws from 'ws';
import { tap, catchError } from 'rxjs/operators';
import { EMPTY } from 'rxjs';
import fs from 'fs';
import * as path from 'path';
import { program } from 'commander';


function parseList(value) {
    return value.split(',');
}

program
    .version('0.0.1')
    .description('Record and replay binary messages from WebSocket connections.')
    .option('-pre, --prefix <prefix>', 'Prefix of binary files to replay', 'sensor_rgb')
    .option('-h, --host <host>', 'Host IP to record. Default is 127.0.0.1.', '127.0.0.1')
    .option('-p, --port <port>', 'Port number to record. Default is 8888.', '8888')
    .requiredOption('-s, --sockets <sockets>', 'Comma-separated list of socket paths to record.', parseList)
    .option('-d, --dir <dir>', 'Directory to store the recorded binary messages.', './recordings')
    .helpOption('-H, --HELP', 'display help for command')
    .parse(process.argv);

const { host, port, sockets, dir, prefix } = program.opts();

// Check if the directory exists, if not, create it
if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir);
}

sockets.forEach((socketPath) => {
    const url = `ws://${host}:${port}${socketPath}`;
    const wsSubject = webSocket({
        url,
        WebSocketCtor: ws,
        deserializer: (e) => e.data,
    });

    wsSubject.pipe(
        tap((data) => {
            const timestamp = Date.now();
            const filename = path.join(dir, `${prefix}-${socketPath.replace('/', '')}-${timestamp}.bin`);
            fs.writeFileSync(filename, data);
        }),
        catchError((error) => {
            console.error(`Error on socket ${socketPath}:`, error);
            return EMPTY;
        })
    ).subscribe();

    console.log(`Listening on ${url}. You can start sending data to this WebSocket.`);
});

// 命令：yarn workspace @dreamview/dreamview-mock dreamview-record -p 8888 \
// -s /hmistatus,/map,/pointcloud,/camera,/simworld \
// -d ./recordings

