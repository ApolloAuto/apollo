import shortUUID from 'short-uuid';

export const genereateRequestId = (type: string) => {
    const replaceUid = type.replace(/!.*$/, '');
    return `${replaceUid}!${shortUUID.generate()}`;
};
