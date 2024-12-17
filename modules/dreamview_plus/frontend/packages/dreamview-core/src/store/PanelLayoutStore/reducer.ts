import { produce } from 'immer';
import { MosaicNode } from 'react-mosaic-component';
import set from 'lodash/set';
import get from 'lodash/get';
import shortUUID from 'short-uuid';
import { LocalStorage } from '@dreamview/dreamview-core/src/util/storageManager';
import { completionPanelId, genereatePanelId, genereateNewMosaicNode } from '@dreamview/dreamview-core/src/util/layout';
import { isEmpty } from 'lodash';
import {
    // EXPAND_MODE_LAYOUT_RELATION,
    UPDATE,
    ADD_PANEL_FROM_OUTSIDE,
    REFRESH_PANEL,
    RESET_LAYOUT,
    INIT_PANEL_LAYOUT,
} from './actionTypes';
import { CombineAction, AddPanelFromOutsidePayload, IResetLayoutByMode, RefreshPanelPayload } from './actions';

export type ILayout = {
    [key: string]: {
        mosaicId: string;
        layout: MosaicNode<string>;
    };
};

export type IInitState = {
    layout: ILayout;
    initLatout: ILayout;
};

export const mosaicId = (() => {
    const uid = shortUUID.generate();
    const storageManager = new LocalStorage('mosaicId');
    if (storageManager.get()) {
        return storageManager.get();
    }
    storageManager.set(uid);
    return uid;
})();

/**
 * @description
 * 从外部添加面板，并将其添加到布局中。
 *
 * @param {IInitState} state - 初始化状态对象
 * @param {AddPanelFromOutsidePayload} payload - 包含路径、位置、来源面板配置、面板ID和模式等信息的载荷对象
 * @param {string} payload.path - 路径，如果为空则表示是一级节点
 * @param {number} payload.position - 位置，可以是 'left'、'right'、'top'、'bottom' 之一
 * @param {PanelConfig} payload.originPanelConfig - 来源面板配置对象
 * @param {string} [payload.panelId] - 面板ID，如果未提供则使用生成的面板ID
 * @param {string} mode - 模式，可以是 'edit' 或 'preview'
 *
 * @returns {void} 无返回值
 */
function addPanelFromOutside(state: IInitState, payload: AddPanelFromOutsidePayload) {
    const { path, position, originPanelConfig, panelId, mode } = payload;
    const newPanelType = panelId || genereatePanelId(originPanelConfig.type);

    const isLeve1 = !path || path.length === 0;

    if (!state.layout[mode]) {
        state.layout[mode] = {
            mosaicId,
            layout: '',
        };
    }

    const hasLayout = !!state.layout[mode].layout;

    const newNode = genereateNewMosaicNode(state.layout[mode].layout, path, position, newPanelType);
    if (!hasLayout) {
        state.layout[mode].layout = newPanelType;
    } else if (isLeve1) {
        state.layout[mode].layout = newNode;
    } else {
        set(state.layout[mode].layout as object, path, newNode) as MosaicNode<string>;
    }
}

function refreshPanel(draftState: IInitState, payload: RefreshPanelPayload) {
    const { path, mode } = payload;
    const isLeve1 = path.length === 0;
    if (isLeve1) {
        draftState.layout[mode].layout = genereatePanelId(draftState.layout[mode].layout as string);
    } else {
        const newPanelType = genereatePanelId(get(draftState.layout[mode].layout, path));
        set(draftState.layout[mode].layout as object, path, newPanelType) as MosaicNode<string>;
    }
}

/**
 * @description 根据模式重置布局的处理函数
 * @param {IInitState} draftState 初始状态对象，包含 layout、initLatout 等属性
 * @param {IResetLayoutByMode} payload 包含 mode（字符串类型，表示模式）的对象
 * @returns {void} 无返回值
 */
function resetLayoutByModeHandler(draftState: IInitState, payload: IResetLayoutByMode) {
    draftState.layout[payload.mode].layout = draftState.initLatout[payload.mode].layout;
}

/**
 * @description
 * 将一个对象转换为布局JSON格式。
 * 如果传入的值是空对象或者undefined，则返回空字符串；否则，返回一个包含布局信息的JSON对象。
 * 布局JSON格式：
 * - direction：布局方向，可选值为"vertical"、"horizontal"。默认为"vertical"。
 * - splitPercentage：分割比例，可选值为0到1之间的数字，默认为0.5。
 * - first：第一个子节点的布局信息，类型为布局JSON对象。
 * - second：第二个子节点的布局信息，类型为布局JSON对象。
 *
 * @param value {any} 需要转换的对象，类型为任意类型。
 * @returns {string} 返回一个字符串，表示布局JSON格式。如果传入的值是空对象或者undefined，则返回空字符串。
 */
function toLayoutJson(value: any) {
    if (isEmpty(value)) {
        return '';
    }
    const isLefaeNode = !!value.type;
    if (isLefaeNode) {
        return completionPanelId(value.type);
    }
    const result: any = {};
    Object.entries(value).forEach(([key, subValue]) => {
        if (key === 'direction' || key === 'splitPercentage') {
            result[key] = subValue;
            return;
        }
        if (key === 'first' || key === 'second') {
            result[key] = toLayoutJson(subValue);
        }
    });
    return result;
}

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case UPDATE:
                if (!action.payload.layout) {
                    break;
                }
                if (!draftState.layout[action.payload.mode]) {
                    draftState.layout[action.payload.mode] = {
                        mosaicId,
                        layout: action.payload.layout,
                    };
                } else {
                    draftState.layout[action.payload.mode].layout = action.payload.layout;
                }
                break;
            case INIT_PANEL_LAYOUT:
                draftState.initLatout[action.payload.mode] = {
                    mosaicId,
                    layout: toLayoutJson(action.payload.initLatout),
                };
                draftState.layout[action.payload.mode] = {
                    mosaicId,
                    layout: toLayoutJson(action.payload.currentLayout),
                };
                break;
            case REFRESH_PANEL:
                refreshPanel(draftState, action.payload);
                break;
            case ADD_PANEL_FROM_OUTSIDE:
                addPanelFromOutside(draftState, action.payload);
                break;
            case RESET_LAYOUT:
                resetLayoutByModeHandler(draftState, action.payload);
                break;
            default:
                break;
        }
    });
