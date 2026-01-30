import * as THREE from 'three';
import { type CSS2DRenderer } from 'three/examples/jsm/renderers/CSS2DRenderer.js';
import type Point from './Point';
import type Polyline from './Polyline';

export interface IThreeContext {
    scene: THREE.Scene;
    renderer: THREE.WebGLRenderer;
    camera: THREE.PerspectiveCamera;
    CSS2DRenderer?: CSS2DRenderer;
}

export interface IMarker {
    coordinate: THREE.Vector3;
    direction?: number;
    instance?: Point;
}

export interface IPolyMarker {
    coordinates: THREE.Vector3[];
    instance?: Polyline;
}

export type PointData = {
    x: number;
    y: number;
    z?: number;
    heading?: number;
};

export type CreatePathwayMarkerCallbackRes = {
    lastPosition: PointData;
    allPositions: PointData[];
    origin?: string;
};

export type FunctionalOperation = 'init' | 'edit' | 'undo' | 'reset';
