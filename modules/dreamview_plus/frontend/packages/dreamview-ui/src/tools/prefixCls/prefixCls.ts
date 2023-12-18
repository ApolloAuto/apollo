const GLOBAL_PREFIX_CLS = 'dreamview';

export const getPrefixCls = (suffixCls?: string, customizePrefixCls?: string): string => {
    if (customizePrefixCls) return customizePrefixCls;
    return suffixCls ? `${GLOBAL_PREFIX_CLS}-${suffixCls}` : GLOBAL_PREFIX_CLS;
};
