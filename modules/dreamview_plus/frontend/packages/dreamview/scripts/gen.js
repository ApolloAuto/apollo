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
${ROOT_DIR}/cyber/proto/record.proto \
${DREAMVIEW_DIR}/proto/simulation_world.proto \
${DREAMVIEW_DIR}/proto/chart.proto \
${DREAMVIEW_DIR}/proto/camera_update.proto \
${DREAMVIEW_DIR}/proto/data_handler.proto \
${DREAMVIEW_DIR}/proto/obstacle.proto \
${MODULES_DIR}/common/proto/*.proto \
${MODULES_DIR}/common/configs/proto/vehicle_config.proto \
${MODULES_DIR}/localization/proto/localization.proto \
${MODULES_DIR}/localization/proto/pose.proto \
${MODULES_DIR}/localization/proto/localization_status.proto \
${MODULES_DIR}/canbus/proto/chassis.proto \
${MODULES_DIR}/planning/proto/*.proto \
${MODULES_DIR}/planning/proto/math/*.proto \
${MODULES_DIR}/perception/proto/traffic_light_detection.proto \
${MODULES_DIR}/perception/proto/perception_obstacle.proto \
${MODULES_DIR}/common/monitor_log/proto/monitor_log.proto \
${MODULES_DIR}/routing/proto/routing.proto \
${MODULES_DIR}/map/proto/*.proto \
${MODULES_DIR}/prediction/proto/feature.proto \
${MODULES_DIR}/prediction/proto/lane_graph.proto \
${MODULES_DIR}/prediction/proto/prediction_point.proto \
${MODULES_DIR}/prediction/proto/prediction_obstacle.proto \
${MODULES_DIR}/prediction/proto/scenario.proto \
${MODULES_DIR}/map/relative_map/proto/*.proto \
${MODULES_DIR}/audio/proto/audio_common.proto \
${MODULES_DIR}/audio/proto/audio_event.proto \
${MODULES_DIR}/common_msgs/*/*.proto \
${DREAMVIEW_DIR}/proto/point_cloud.proto \
${DREAMVIEW_DIR}/proto/point_cloud.proto \
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
        fs.readFile(DREAMVIEW_PROTO_BUNDLE_PATH, 'utf-8', (readFileErr, data) => {
            if (readFileErr) {
                throw readFileErr;
            }

            // Add 'const JSObject = Object;'
            let patchedData = `const JSObject = Object;\n${data}`;

            // Replace all Object.keys with JSObject.keys
            patchedData = patchedData.replaceAll('Object.keys(', 'JSObject.keys(');

            // Write back to the file
            fs.writeFile(DREAMVIEW_PROTO_BUNDLE_PATH, patchedData, 'utf-8', (writeFileErr) => {
                if (writeFileErr) {
                    throw writeFileErr;
                }
                console.log('Patching is complete!');
            });
        });
    });
});
