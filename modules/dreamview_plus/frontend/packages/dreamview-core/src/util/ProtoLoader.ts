import * as protobuf from 'protobufjs';
import Logger from '@dreamview/log';
import { indexedDBStorage } from './indexedDB/IndexedDBStorage';

export enum ProtoStatus {
    Loading = 'loading',
    Loaded = 'loaded',
    Error = 'error',
    Idle = 'idle',
}

const Root = protobuf.Root;

const logger = Logger.getInstance('ProtoLoader');

interface ProtoLoadedMsg {
    [key: string]: any;
}

export class ProtoLoader {
    private loadedProtoFiles: ProtoLoadedMsg;

    // origin: 这个参数是当前正在解析的 .proto 文件的路径。
    // target: 这个参数是在 .proto 文件中使用 import 语句指定的路径
    private resolvePath = function (_origin: string, target: string) {
        if (target.startsWith('modules')) {
            return `proto/${target}`;
        }
        return target;
    };

    constructor() {
        this.loadedProtoFiles = {};
    }

    private jsonDescriptor: any = {};

    private rootCache: { [key: string]: protobuf.Root } = {};

    async loadAndCacheProto(
        protoPath: string,
        config?: {
            dataName: string;
            channelName?: string;
        },
    ): Promise<protobuf.Root> {
        try {
            if (this.rootCache[protoPath]) {
                return this.rootCache[protoPath];
            }
            const theProtoLoaderStatus = await ProtoLoader.getProtoCache(protoPath);

            if (theProtoLoaderStatus && theProtoLoaderStatus === ProtoStatus.Loading) {
                // 延长等待时间，等待正在加载的 proto 加载完成
                await ProtoLoader.setProtoCache(protoPath, ProtoStatus.Loading, 500);
                logger.warn(`${protoPath} is loading,this load will be ignored`);
                return null;
            }

            // 尝试从 IndexedDB 缓存中获取 Proto 对象
            const cachedProto = await this.getProtoDescriptor(config?.dataName, config?.channelName);
            // @ts-ignore
            if (cachedProto && cachedProto.nested) {
                this.rootCache[protoPath] = Root.fromJSON(cachedProto);
                return this.rootCache[protoPath];
            }

            await ProtoLoader.setProtoCache(protoPath, ProtoStatus.Loading, 500);
            // 否则，从文件加载
            const root = new protobuf.Root();
            root.resolvePath = this.resolvePath;
            const pRoot = await root.load(protoPath);
            if (config?.dataName) {
                const descriptor = pRoot.toJSON();
                if (descriptor && descriptor.nested) {
                    await ProtoLoader.setProtoCache(protoPath, ProtoStatus.Loaded, 500);
                    await this.setProtoDescriptor(descriptor, config);
                }
            }
            this.rootCache[protoPath] = root;
            return root;
        } catch (error) {
            logger.error(`Error loading or caching proto ${protoPath}: ${error}`);
            throw error;
        }
    }

    async loadRecursive(fileName: any, rootFileName: any) {
        try {
            // proto file have been loaded, return;
            if (this.loadedProtoFiles[fileName]) {
                return;
            }

            let loadFileName = fileName;
            if (fileName.startsWith('modules')) {
                loadFileName = `proto/${fileName}`;
            }

            // first to set map, default belive it can be loaded.
            this.loadedProtoFiles[fileName] = new protobuf.Root();

            const dependFiles: string[] = [];
            await fetch(loadFileName)
                .then((response) => response.text())
                .then((protoContent) => {
                    const root = protobuf.parse(protoContent);
                    this.loadedProtoFiles[fileName] = root;
                    root.imports.forEach((item) => {
                        dependFiles.push(item);
                    });
                });

            await Promise.all(dependFiles.map((file) => this.loadRecursive(file, rootFileName)));
        } catch (error) {
            logger.error(`Error loading or caching proto ${fileName}: ${error}`);
            throw error;
        }
    }

    async loadProto(fileName: any) {
        await this.loadRecursive(fileName, fileName);
    }

    async setProtoDescriptor(descriptor: any, config: { dataName: string; channelName?: string }) {
        const { dataName, channelName } = config;
        const key = `${dataName}${channelName ? `-${channelName}` : ''}`;
        const storeManager = await indexedDBStorage.getStoreManager('ProtoDescriptor');
        return storeManager.setItem(key, descriptor, 1000 * 60 * 60 * 24);
    }

    async getProtoDescriptor(dataName: string, channelName?: string) {
        const key = `${dataName}${channelName ? `-${channelName}` : ''}`;
        if (this.jsonDescriptor[key]) {
            return this.jsonDescriptor[key];
        }
        const storeManager = await indexedDBStorage.getStoreManager('ProtoDescriptor');
        this.jsonDescriptor[key] = storeManager.getItem(key);
        return this.jsonDescriptor[key];
    }

    static async setProtoCache(path: string, descriptor: any, timeout?: number) {
        const storeManager = await indexedDBStorage.getStoreManager('ProtoCache');
        return storeManager.setItem(path, descriptor, timeout ?? 1000 * 60 * 60 * 24);
    }

    static async getProtoCache(path: string) {
        const storeManager = await indexedDBStorage.getStoreManager('ProtoCache');
        return storeManager.getItem(path);
    }
}
