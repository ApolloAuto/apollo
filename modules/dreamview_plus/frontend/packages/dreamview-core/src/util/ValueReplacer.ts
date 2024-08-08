import { LRUCache } from 'lru-cache';
import Logger from '@dreamview/log';
import { JSONPath } from 'jsonpath-plus';

const logger = Logger.getInstance('ValueReplacer');

export type ValueTransformer = (value: any) => any;

interface TransformerEntry {
    transformer: ValueTransformer;
    path: string;
}

class ValueReplacer {
    private transformers: Map<string, Map<string, TransformerEntry>> = new Map();

    private cache: LRUCache<string, any>;

    constructor(maxCacheSize = 1000) {
        this.cache = new LRUCache<string, any>({ max: maxCacheSize });
    }

    public registerTransformer(msgType: string, paths: string | string[], transformer: ValueTransformer): void {
        const pathArray = Array.isArray(paths) ? paths : [paths];
        if (!this.transformers.has(msgType)) {
            this.transformers.set(msgType, new Map());
        }
        pathArray.forEach((path) => {
            this.transformers.get(msgType)!.set(path, { transformer, path });
        });
        logger.info(`Transformer registered for msgType: ${msgType} with paths: ${pathArray.join(', ')}`);
    }

    public applyTransformations(obj: any, msgType: string): void {
        const typeTransformers = this.transformers.get(msgType);
        if (!typeTransformers) {
            logger.error(`No transformers registered for msgType: ${msgType}`);
            return;
        }
        typeTransformers.forEach((entry) => {
            const paths = JSONPath({ path: entry.path, json: obj, resultType: 'path' });
            paths.forEach((path: string) => {
                const value = JSONPath({ path, json: obj, resultType: 'value' })[0];
                const cacheKey = `${msgType}:${entry.path}:${JSON.stringify(value)}`;
                let transformedValue = this.cache.get(cacheKey);

                if (!transformedValue) {
                    transformedValue = entry.transformer(value);
                    this.cache.set(cacheKey, transformedValue);
                }
                this.setValueByPath(obj, path, transformedValue);
            });
        });
    }

    private setValueByPath(obj: any, path: string, value: any): void {
        JSONPath({
            path,
            json: obj,
            callback(payload) {
                // payload 是当前路径匹配到的元素，parent 是其父对象，parentProperty 是该元素在父对象中的键名
                if (payload && payload.parent && payload.parentProperty !== undefined) {
                    payload.parent[payload.parentProperty] = value;
                }
            },
            resultType: 'all',
        });
    }

    public clearCache(): void {
        this.cache.clear();
        logger.info('Cache cleared.');
    }
}

export default ValueReplacer;
