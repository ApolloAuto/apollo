// 安全的json字符串转换对象
export function safeParseJSON(json) {
  try {
    return JSON.parse(json);
  } catch (e) {
    console.error(`Failed to parse JSON: ${json}`);
    return null;
  }
}
