import flatted from 'flatted';
export default function visualizeFlatted(data: object) {
    const flattedData = flatted.stringify(data);
    const parsedData = flatted.parse(flattedData);

    // 创建一个深拷贝，以避免修改原始数据
    const visualizedData = JSON.parse(JSON.stringify(parsedData));

    // 遍历数据并替换引用
    for (const key in visualizedData) {
        if (Object.prototype.hasOwnProperty.call(visualizedData, key)) {
            const value = visualizedData[key];

            if (typeof value === 'string' && !isNaN(Number(value))) {
                // 这是一个 flatted 引用，用实际值替换它
                visualizedData[key] = parsedData[parseInt(value, 10)];
            }
        }
    }

    return JSON.stringify(visualizedData, null, 2);
}
