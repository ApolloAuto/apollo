import * as THREE from "three";
import MTLLoader from "three/examples/js/loaders/MTLLoader.js";
import OBJLoader from "three-obj-loader";

OBJLoader(THREE);

// The two loaders for material and object files, respectively.
const mtlLoader = new THREE.MTLLoader();
const objLoader = new THREE.OBJLoader();
const textureLoader = new THREE.TextureLoader();

export function loadObject(materialFile, objectFile, scale, callback) {
    mtlLoader.load(materialFile, materials => {
        materials.preload();
        objLoader.setMaterials(materials);
        objLoader.load(objectFile, loaded => {
            const object = loaded.clone();
            object.name = objectFile;
            object.scale.set(scale.x, scale.y, scale.z);
            callback(object);
        });
    });
}

export function loadTexture(textureFile, callback) {
    textureLoader.load(textureFile, callback);
}
