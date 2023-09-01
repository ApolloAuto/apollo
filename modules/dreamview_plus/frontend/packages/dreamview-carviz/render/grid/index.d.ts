import * as THREE from 'three';
interface Grid {
    size: number;
    divisions: number;
    colorCenterLine: THREE.Color;
    colorGrid: THREE.Color;
}
/**
 * 绘制网格的类
 * @class
 * @classdesc 绘制网格的类
 */
declare class Grid {
    private scene;
    constructor(scene: THREE.Scene);
    /**
     * 绘制网格
     * @param {number} size 坐标格尺寸
     * @param {number} divisions 坐标格细分次数
     * @param {THREE.Color} colorCenterLine 中线颜色
     * @param {THREE.Color} colorGrid 坐标网格线颜色
     * @returns THREE.gridHelper 返回辅助网格对象
     */
    drawGrid(gidAttr: Grid): void;
    /**
     * 删除网格
     */
    removeGrid(): void;
}
export default Grid;
