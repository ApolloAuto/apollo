import { stringify, parse } from 'flatted';

export const visualizeFlatted = (data: object) => {
    const flattedData = stringify(data);
    const parsedData = parse(flattedData);

    // 创建一个深拷贝，以避免修改原始数据
    const visualizedData = JSON.parse(JSON.stringify(parsedData));

    // 遍历数据并替换引用
    Object.keys(visualizedData).forEach((key) => {
        const value = visualizedData[key];

        if (typeof value === 'string' && !Number.isNaN(Number(value))) {
            // 这是一个 flatted 引用，用实际值替换它
            visualizedData[key] = parsedData[parseInt(value, 10)];
        }
    });

    return JSON.stringify(visualizedData, null, '');
};
