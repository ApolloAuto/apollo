export enum MouseEventType {
    // 鼠标移动获取世界坐标，入参[x, y]
    CURRENT_COORDINATES = 'CURRENT_COORDINATES',
    // 鼠标拖动距离上一点长度，入参length
    CURRENT_LENGTH = 'CURRENT_LENGTH',
    // 隐藏鼠标移动获取世界坐标
    HIDE_CURRENT_COORDINATES = 'HIDE_CURRENT_COORDINATES',
}
