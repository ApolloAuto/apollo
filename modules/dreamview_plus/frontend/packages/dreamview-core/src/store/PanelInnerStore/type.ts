export type FullScreenFnRef = {
    enterFullScreen: () => Promise<void>;
    exitFullScreen: () => Promise<void>;
};
