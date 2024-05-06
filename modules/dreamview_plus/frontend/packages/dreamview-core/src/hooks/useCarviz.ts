import { useEffect, useState } from 'react';
import { Carviz, RoutingEditor } from '@dreamview/dreamview-carviz/src/index';
import { useThemeContext } from '@dreamview/dreamview-theme';
import shortUUID from 'short-uuid';

export default function useCarViz(): [Carviz, string] {
    const { tokens } = useThemeContext();
    const [uid] = useState(shortUUID.generate);
    const [carviz] = useState(() => new Carviz(uid));

    useEffect(() => {
        try {
            carviz?.updateColors(tokens.components.carViz);
            carviz.resetScence();
        } catch (err) {
            //
        }
    }, [tokens]);

    return [carviz, uid];
}

export function useRoutingEditor(): [RoutingEditor, string] {
    const { tokens } = useThemeContext();
    const [uid] = useState(shortUUID.generate);
    const [carviz] = useState(() => new RoutingEditor(uid));

    useEffect(() => {
        try {
            carviz?.updateColors(tokens.components.carViz);
            carviz.resetScence();
        } catch (err) {
            //
        }
    }, [tokens]);

    return [carviz, uid];
}
