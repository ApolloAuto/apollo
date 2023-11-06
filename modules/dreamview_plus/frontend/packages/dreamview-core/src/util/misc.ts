/**
 * 补充0至指定长度
 * @param val
 * @param length
 * @returns
 * fill0(1, 3) => 001
 * fill0(21, 3) => 021
 */
export function fill0(val: number | string, length = 2, addInFront = true): string {
    const number = Number(val);
    const max = 10 ** (length - 1);
    if (number > max) {
        return String(number);
    }
    const prefFix = '0'.repeat(length - String(number).length);
    if (typeof number !== 'number') {
        throw new Error('fill0 recived an invidate value');
    }
    if (addInFront) {
        return `${prefFix}${number}`;
    }
    return `${number}${prefFix}`;
}
/**
 *
 * @param timestampMs
 * @param showMilliseconds
 * @returns 12:33:31:054 or 12:33:31
 */
export function timestampMsToTimeString(timestampMs: number | string, showMilliseconds = false) {
    const date = new Date(timestampMs);
    const hours = fill0(date.getHours());
    const minutes = fill0(date.getMinutes());
    const seconds = fill0(date.getSeconds());
    const milliseconds = fill0(date.getMilliseconds(), 3);

    let timeString = `${hours}:${minutes}:${seconds}`;

    if (showMilliseconds) {
        timeString += `:${milliseconds}`;
    }

    return timeString;
}

const MILLISECONDS_IN_A_SECOND = 1000;
const MILLISECONDS_IN_A_MINUTE = 1000 * 60;

/**
 *
 * @param duration
 * @returns 00:00.000
 */
export function millisecondsToTime(duration: number) {
    const milliseconds = fill0(Math.floor(duration % 1000), 3);
    const seconds = fill0(Math.floor((duration / MILLISECONDS_IN_A_SECOND) % 60));
    const minutes = fill0(Math.floor(duration / MILLISECONDS_IN_A_MINUTE));

    return `${minutes}:${seconds}.${milliseconds}`;
}
