import { useMemo } from 'react';
import { useThemeContext } from '@dreamview/dreamview-theme';
import { IImageName } from './type';
import lightImg from './light';
import drakImg from './drak';

export function useImagePrak(name: IImageName | IImageName[]) {
    const theme = useThemeContext()?.theme;

    return useMemo(() => {
        const imgRaw = theme === 'drak' ? drakImg : lightImg;
        if (typeof name === 'string') {
            return imgRaw[name];
        }
        return name.map((item) => imgRaw[item as IImageName]);
    }, [theme]);
}
