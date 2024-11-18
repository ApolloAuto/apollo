import { message } from '@dreamview/dreamview-ui';

/**
 * 下载文本文件
 *
 * @param filename 文件名称
 * @param text 文件内容，类型为字符串
 */
export function downloadTextFile(filename: string, text: string) {
    // 创建 Blob 对象，包含文件内容
    const blob = new Blob([text], { type: 'text/plain' });
    // 创建一个 URL 对象
    const url = URL.createObjectURL(blob);

    // 创建一个 <a> 标签
    const a = document.createElement('a');
    a.href = url;
    a.download = filename; // 指定下载的文件名

    // 触发点击事件，开始下载
    document.body.appendChild(a); // 添加到 DOM 中
    a.click(); // 触发点击

    // 移除 <a> 标签，清理 URL 对象
    document.body.removeChild(a);
    URL.revokeObjectURL(url); // 释放 URL 对象
}

/**
 * 一键复制函数，将传入的文本复制到剪切板中。
 *
 * @param text 需要复制的文本字符串。
 * @returns {Promise<void>} 返回一个 Promise，resolve 时表示复制成功，reject 时表示复制失败或浏览器不支持一键复制属性。
 */
export default function clipboard(text: string) {
    return new Promise((resovle, reject) => {
        if (!navigator.clipboard) {
            message({
                type: 'warning',
                content: `浏览器不支持一键复制属性，请手动复制！${text}`,
            });
            reject();
        } else {
            navigator.clipboard
                .writeText(text)
                .then(() => {
                    resovle(undefined);
                })
                .catch(() => {
                    message({
                        type: 'warning',
                        content: `浏览器不支持一键复制属性，请手动复制！${text}`,
                    });
                    reject();
                });
        }
    });
}
