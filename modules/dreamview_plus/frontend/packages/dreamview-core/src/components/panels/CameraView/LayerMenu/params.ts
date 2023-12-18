export interface subMenuParams {
    [key: string]: {
        defaultVisible: boolean;
        currentVisible: boolean;
        vizKey: string;
    };
}

export const layerMenuParams: { [key: string]: subMenuParams } = {
    '': {
        boundingbox: {
            defaultVisible: false,
            currentVisible: false,
            vizKey: 'Boundingbox',
        },
    },
};
