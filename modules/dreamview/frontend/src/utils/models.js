import * as THREE from "three";
import OBJLoader from "three/examples/js/loaders/OBJLoader.js";
import MTLLoader from "three/examples/js/loaders/MTLLoader.js";

// The two loaders for material and object files, respectively.
const mtlLoader = new THREE.MTLLoader();
const textureLoader = new THREE.TextureLoader();

THREE.TextureLoader.prototype.crossOrigin = '';

const loadedMaterialAndObject = {};

export function loadObject(materialFile, objectFile, scale, callback) {
    function placeMtlAndObj(loaded) {
        if (callback) {
            const object = loaded.clone();
            callback(object);
        }
    }

    if (loadedMaterialAndObject[objectFile]) {
        placeMtlAndObj(loadedMaterialAndObject[objectFile]);
    } else {
        mtlLoader.load(materialFile, materials => {
            const objLoader = new THREE.OBJLoader();
            materials.preload();
            objLoader.setMaterials(materials);

            objLoader.load(objectFile, loaded => {
                loaded.name = objectFile;
                loaded.scale.set(scale.x, scale.y, scale.z);
                loadedMaterialAndObject[objectFile] = loaded;
                placeMtlAndObj(loaded);
            });
        });
    }
}

export function loadTexture(textureFile, callback) {
    textureLoader.load(textureFile, callback);
}
