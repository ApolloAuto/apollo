const path = require('path');
const { exec } = require('child_process');
const fs = require('fs');

// Resolve the directories
const ROOT_DIR = path.resolve(__dirname, '../../../../../..');
const MODULES_DIR = path.resolve(__dirname, '../../../../..');
const DREAMVIEW_DIR = path.resolve(__dirname, '../../../..');

const DREAMVIEW_PROTO_BUNDLE_PATH = path.resolve(__dirname, '../lib/dreamview_proto_bundle.js');
const DECLARATION_BUNDLE_PATH = path.resolve(__dirname, '../lib/dreamview_proto_bundle.d.ts');

// Define the command to be executed
const generateProto = `
${'../node_modules/.bin/pbjs'} -t static-module -w commonjs \
${DREAMVIEW_DIR}/proto/data_handler.proto \
-o ${DREAMVIEW_PROTO_BUNDLE_PATH}
`;

const generateDeclaration = `
${'../node_modules/.bin/pbts'} ${DREAMVIEW_PROTO_BUNDLE_PATH} -o ${DECLARATION_BUNDLE_PATH}
`;

// Execute the command
exec(generateProto, (err, stdout, stderr) => {
    if (err) {
        console.error(`generateProto Error: ${err}`);
        return;
    }

    if (stderr) {
        console.error(`generateProto STDERR: ${stderr}`);
        return;
    }

    console.log(`generateProto STDOUT: ${stdout}`);

    // eslint-disable-next-line no-shadow
    exec(generateDeclaration, (err, stdout, stderr) => {
        if (err) {
            console.error(`generateDeclaration Error: ${err}`);
            return;
        }

        if (stderr) {
            console.error(`generateDeclaration STDERR: ${stderr}`);
            return;
        }

        console.log(`generateDeclaration STDOUT: ${stdout}`);

        // Add patching step after the protobuf files are generated
        // fs.readFile(DREAMVIEW_PROTO_BUNDLE_PATH, 'utf-8', (readFileErr, data) => {
        //     if (readFileErr) {
        //         throw readFileErr;
        //     }
        //
        //     // Add 'const JSObject = Object;'
        //     let patchedData = `const JSObject = Object;\n${data}`;
        //
        //     // Replace all Object.keys with JSObject.keys
        //     patchedData = patchedData.replaceAll('Object.keys(', 'JSObject.keys(');
        //
        //     // Write back to the file
        //     fs.writeFile(DREAMVIEW_PROTO_BUNDLE_PATH, patchedData, 'utf-8', (writeFileErr) => {
        //         if (writeFileErr) {
        //             throw writeFileErr;
        //         }
        //         console.log('Patching is complete!');
        //     });
        // });
    });
});
