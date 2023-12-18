/* eslint-disable no-continue */
import get from 'lodash/get';

const protoMsgType: Record<string, boolean> = [
    'double',
    'float',
    'int32',
    'int64',
    'uint32',
    'uint64',
    'sint32',
    'sint64',
    'fixed32',
    'fixed64',
    'sfixed32',
    'sfixed64',
    'bool',
    'string',
    'bytes',
    'enum',
].reduce((result, key) => ({ ...result, [key]: true }), {});

const isNumberOrStringOrBool = (type: string) => protoMsgType[type];
const isNumberOrStringOrBoolArray = (value: any) => value.rule === 'repeated' && isNumberOrStringOrBool(value.type);
const isObjectArray = (value: any) => value.rule === 'repeated' && !isNumberOrStringOrBool(value.type);

export default function jsonDescribeToJson(json: any, fieldPath?: string) {
    const result: string[] = [];
    function reduce(prop: any, prefix: string, path: string) {
        const keys = Object.keys(prop || {});
        for (let i = 0; i < keys.length; i += 1) {
            const key = keys[i];
            const value = prop[key];

            // value.keyType === 'string'时，key无法枚举，忽略掉
            if (value.keyType === 'string') {
                continue;
            }
            // oneofs 的作用是描述了属于某几个类型的其中之一，不作为类型描述存在，忽略。
            if (key === 'oneofs') {
                continue;
            }
            if (key === 'nested') {
                reduce(value, prefix, path);
                continue;
            }
            if (key === 'fields') {
                reduce(value, prefix, path);
                continue;
            }
            // 有type说明已经到叶子结点了
            if (value.type) {
                if (isNumberOrStringOrBoolArray(value)) {
                    // 普通数据数组类型结束遍历，不保存节点路径
                } else if (isNumberOrStringOrBool(value.type)) {
                    // 普通数据类型结束遍历，保存节点路径
                    result.push(`${prefix}/${key}`.replace(/^\//, '').replace(/\//g, '.'));
                } else {
                    // type为其他定义的类型时需要继续遍历
                    let subValue;
                    let subValuePath;

                    if (/^apollo/.test(value.type) || /^google/.test(value.type)) {
                        // 类型1: apollo.common.GaussianInfo，
                        // 类型1类型需要从根目录遍历寻找apollo.common.GaussianInfo =》nested.apollo.nested.common.nested.GaussianInfo
                        const subPath = `nested.${value.type.split('.').join('.nested.')}`;
                        subValue = get(json, subPath);
                        subValuePath = value.type;
                    } else {
                        // 类型2: Chassis.DrivingMode 或 DrivingMode
                        // Chassis.DrivingMode需要根据path递归向上查找.
                        const pathArr = path.replace(/^\./, '').split('.');
                        const type = /\./.test(value.type) ? `${value.type.split('.').join('.nested.')}` : value.type;
                        while (pathArr.length) {
                            const subPath = `nested.${pathArr.join('.nested.')}.nested`;
                            const val = get(json, `${subPath}.${type}`);
                            if (val) {
                                subValue = val;
                                subValuePath = `${pathArr.join('.')}.${value.type}`;
                                break;
                            } else {
                                pathArr.pop();
                            }
                        }
                    }

                    const isEnum = !!subValue.values;

                    // 如果subValue是枚举类型结束遍历，保存节点路径
                    if (isEnum) {
                        result.push(`${prefix}/${key}`.replace(/^\//, '').replace(/\//g, '.'));
                    } else if (isObjectArray(value)) {
                        // 数组中如果子项内容又出现了数组则忽略
                        if (/\[0\]/.test(prefix)) {
                            break;
                        }
                        reduce(subValue, `${prefix}/${key}/[0]`, subValuePath);
                    } else {
                        reduce(subValue, `${prefix}/${key}`, subValuePath);
                    }
                }
                continue;
            }
            const isEnum = !!value.values;
            if (!isEnum && value) {
                reduce(value, `${prefix}/${key}`, `${path}.${key}`);
            }
        }
    }
    if (fieldPath) {
        reduce(get(json, `nested.${fieldPath.split('.').join('.nested.')}`), '', fieldPath);
    } else {
        reduce(json, '', '');
    }
    return result;
}
