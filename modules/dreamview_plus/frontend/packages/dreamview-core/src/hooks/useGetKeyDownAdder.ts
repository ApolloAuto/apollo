import useGetPublicFn from './useGetPublicFn';

export default function useGetKeyDownAdder(panelId: string) {
    return useGetPublicFn(`keydown-add:${panelId}`);
}
