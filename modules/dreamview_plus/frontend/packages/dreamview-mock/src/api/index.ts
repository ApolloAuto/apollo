import MockSocketServer from '../base/mockSocket';
import { PluginApiNames } from '@dreamview/dreamview-core/src/services/api/types';

function initMockSocketServer() {
    const server = new MockSocketServer(8888);

    server.addMock({
        dataName: 'simworld',
        protoPath: '',
        requestName: 'simworld',
        websocketInfo: {
            websocketName: 'simworld',
            websocketPipe: 'simworld',
        },
        subscribe: true,
    });

    server.addMock({
        dataName: 'camera',
        protoPath: '',
        requestName: 'camera',
        channels: [
            '/apollo/sensor/camera/front_12mm/image',
            '/apollo/sensor/camera/front_6mm/image',
        ],
        differentForChannels: true,
        websocketInfo: {
            websocketName: 'camera',
            websocketPipe: 'camera',
        },
        subscribe: true,
    });
    server.addMock({
        dataName: 'pointcloud',
        protoPath: '',
        requestName: 'pointcloud',
        channels: [
            "/apollo/sensor/velodyne64/compensator/PointCloud2"
        ],
        differentForChannels: true,
        websocketInfo: {
            websocketName: 'pointcloud',
            websocketPipe: 'pointcloud',
        },
        subscribe: true,
    });

    server.addMock({
        dataName: 'map',
        protoPath: '',
        requestName: 'map',
        websocketInfo: {
            websocketName: 'map',
            websocketPipe: 'map',
        },
        subscribe: true,
    });

    server.addMock({
        mockType: 'PluginRequest',
        mockName: PluginApiNames.CheckCertStatus,
        response: async (data) => JSON.stringify(data),
    })
}

export default initMockSocketServer;
