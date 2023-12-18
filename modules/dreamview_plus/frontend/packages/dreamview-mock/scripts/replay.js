const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');
const { from, interval, repeat } = require('rxjs');
const { concatMap, take, tap } = require('rxjs/operators');
const { program } = require('commander');

function parseList(value) {
    return value.split(',');
}

program
    .version('0.0.1')
    .description('Record and replay binary messages from WebSocket connections.')
    .option('-pre, --prefix <prefix>', 'Prefix of binary files to replay', 'sensor_rgb')
    .option('-h, --host <host>', 'Host IP to record. Default is 127.0.0.1.', '127.0.0.1')
    .option('-p, --port <port>', 'Port number to record. Default is 8888.', '8889')
    .requiredOption('-s, --sockets <sockets>', 'Comma-separated list of socket paths to record.', parseList)
    .option('-d, --dir <dir>', 'Directory to read binary files from', './recordings')
    .option('-f, --frequency <frequency>', 'Frequency to replay at in Hz', parseFloat, 10)
    .helpOption('-H, --HELP', 'display help for command')
    .parse(process.argv);

const { prefix, dir, port, host, sockets, frequency } = program.opts();

sockets.forEach((socket) => {
    const files = fs
        .readdirSync(dir)
        .filter((filename) => filename.startsWith(`${prefix}${socket.replace('/', '-')}`))
        .map((filename) => path.join(dir, filename))
        .sort();

    const wss = new WebSocket.Server({
        host,
        path: socket,
        port,
    });

    wss.on('connection', (ws) => {
        console.log('Client connected');

        // Create an observable from the array of files
        const file$ = from(files);

        // Create an interval observable that emits every 1/frequency seconds
        const interval$ = interval(1000 / frequency);

        // Combine the two observables such that for each tick of the interval,
        // one file is read and sent over the WebSocket
        interval$
            .pipe(
                take(files.length), // Take only as many ticks as there are files
                concatMap(() => file$), // For each tick, map to a file
                tap((filename) => {
                    // For each file, read it and send its contents over the WebSocket
                    const data = fs.readFileSync(filename);
                    ws.send(data);
                }),
                repeat(), // repeat sending when all files are sent
            )
            .subscribe({});

        ws.on('close', () => {
            console.log('Client disconnected');
        });
    });

    console.log(`WebSocket path ${socket} is running`);
});
console.log(`WebSocket server is running on ws://localhost:${port}`);

// 命令: yarn workspace @dreamview/dreamview-mock dreamview-replay -s /camera
