import {describe, expect, test} from '@jest/globals';
import { getPrefixCls } from './prefixCls';

describe('prefixClsTest', () => {
    test('prefixClsCustomTest', () => {
        expect(getPrefixCls('btn', 'customPrefix')).toBe('customPrefix');
    });

    test('prefixClsSuffixTest', () => {
        expect(getPrefixCls('btn')).toBe('dreamview-btn');
    });
});