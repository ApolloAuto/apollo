import { KeyHandlers } from '../components/panels/base/KeyListener';
import useCustomSubcribe from './useCustomSubcribe';

export default function useKeyDownAddHandler(panelId: string, keyAddHandler: (handlers: KeyHandlers[]) => void) {
    useCustomSubcribe(`keydown-add:${panelId}`, keyAddHandler);
}
