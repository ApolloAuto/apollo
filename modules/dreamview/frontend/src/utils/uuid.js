/**
 * 获取一个唯一标识符UUID。
 *
 * @returns {string} 返回生成的唯一标识符字符串。
 */
export function getUUID() {
  let d = new Date().getTime();
  const uuid = 'xxxxxxxx-xxxx-xxxx'.replace(/[xy]/g, function (c) {
    const r = (d + Math.random() * 16) % 16 | 0;
    d = Math.floor(d / 16);
    return (c === 'x' ? r : (r & 0x3 | 0x8)).toString(16);
  });
}
