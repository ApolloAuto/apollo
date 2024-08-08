import WebGL from 'three/addons/capabilities/WebGL.js';


export function checkWebGLSupport() {
    try {
        return WebGL.isWebGLAvailable();
    } catch (e) {
        return false;
    }
}

// 检查webgpu
export function checkWebGPUSupport() {
    return 'gpu' in navigator;
}
