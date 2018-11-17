import * as THREE from "three";

const _ = require('lodash');

const fonts = {};
let fontsLoaded = false;
const loader = new THREE.FontLoader();
const fontPath = "fonts/gentilis_bold.typeface.json";
loader.load(fontPath, font => {
        fonts['gentilis_bold'] = font;
        fontsLoaded = true;
    },
    function (xhr) {
        console.log(fontPath + (xhr.loaded / xhr.total * 100) + '% loaded');
    },
    function (xhr) {
        console.log( 'An error happened when loading ' + fontPath );
});

export default class Text3D {
    constructor() {
        // The meshes for each ASCII char, created and reused when needed.
        // e.g. {65: [mesh('a'), mesh('a')], 66: [mesh('b')]}
        // These meshes will not be deleted even when not in use,
        // as the construction is expensive.
        this.charMeshes = {};
        // Mapping from each ASCII char to the index of the mesh used
        // e.g. {65: 1, 66: 0}
        this.charPointers = {};
    }

    reset() {
        this.charPointers = {};
    }

    composeText(text) {
        if (!fontsLoaded) {
            return null;
        }
        // 32 is the ASCII code for white space.
        const charIndices = _.map(text, l => l.charCodeAt(0) - 32);
        const letterOffset = 0.4;
        const textMesh = new THREE.Object3D();
        for (let j = 0; j < charIndices.length; j++) {
            const idx = charIndices[j];
            let pIdx = this.charPointers[idx];
            if (pIdx === undefined) {
                pIdx = 0;
                this.charPointers[idx] = pIdx;
            }
            if (this.charMeshes[idx] === undefined) {
                this.charMeshes[idx] = [];
            }
            let mesh = this.charMeshes[idx][pIdx];
            if (mesh === undefined) {
                if (this.charMeshes[idx].length > 0) {
                    mesh = this.charMeshes[idx][0].clone();
                } else {
                    mesh = this.drawChar3D(text[j]);
                }
                this.charMeshes[idx].push(mesh);
            }

            let additionalOffset = 0;
            switch (text[j]) {
                case ',':
                    additionalOffset = 0.35;
                    break;
                case '/':
                    additionalOffset = 0.15;
                    break;
            }
            mesh.position.set((j - charIndices.length / 2) * letterOffset + additionalOffset, 0, 0);
            this.charPointers[idx]++;
            textMesh.add(mesh);
        }
        return textMesh;
    }

    drawChar3D(char, font = fonts['gentilis_bold'], size = 0.6, height = 0,
            color = 0xFFEA00) {
        const charGeo = new THREE.TextGeometry(char, {
            font: font,
            size: size,
            height: height});
        const charMaterial = new THREE.MeshBasicMaterial({color: color});
        const charMesh = new THREE.Mesh(charGeo, charMaterial);
        return charMesh;
    }
}