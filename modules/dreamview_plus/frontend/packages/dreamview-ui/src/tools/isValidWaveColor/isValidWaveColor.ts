export function isNotGrey(color: string) {
    // eslint-disable-next-line no-useless-escape
    const match = (color || '').match(/rgba?\((\d*), (\d*), (\d*)(, [\d.]*)?\)/);
    if (match && match[1] && match[2] && match[3]) {
        return !(match[1] === match[2] && match[2] === match[3]);
    }
    return true;
}

export function isValidWaveColor(color: string) {
    return (
        color &&
        color !== '#fff' &&
        color !== '#ffffff' &&
        color !== 'rgb(255, 255, 255)' &&
        color !== 'rgba(255, 255, 255, 1)' &&
        isNotGrey(color) &&
        !/rgba\((?:\d*, ){3}0\)/.test(color) && // any transparent rgba color
        color !== 'transparent'
    );
}
