import { IDefaultTheme as TypeIDefaultTheme } from './tokens/baseToken';

declare module '@dreamview/dreamview-theme' {
    export interface IDefaultTheme extends TypeIDefaultTheme {}
}

export * from './createStyles';
