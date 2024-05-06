/* eslint-disable import/no-webpack-loader-syntax */
// @ts-ignore
import DecoderWorker from 'worker-loader!./decoder.worker';
import Logger from '@dreamview/log';
import { MetadataItem } from '@dreamview/dreamview-core/src/services/WebSocketManager/type';
import { AbstractWorkerWrapper } from './type';

const logger = Logger.getInstance('DecoderWorkerClass');
class DecoderWorkerClass extends AbstractWorkerWrapper {
    constructor() {
        super();
        this.worker = new DecoderWorker();
        logger.debug('DecoderWorkerClass created');
    }

    /**
     * 同步metadata用于反序列化数据
     */
    syncMetadata(metadata: MetadataItem[]) {
        this.postMessage({ type: 'SOCKET_METADATA', payload: metadata });
        return this;
    }
}

export default DecoderWorkerClass;
