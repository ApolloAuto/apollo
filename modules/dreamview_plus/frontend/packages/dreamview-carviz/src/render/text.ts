import * as THREE from 'three';
import { FontLoader } from 'three/examples/jsm/loaders/FontLoader';
import { TextGeometry } from 'three/examples/jsm/geometries/TextGeometry';
import { map } from 'lodash';
import fontJson from '../../assets/fonts/gentilis_bold.typeface.json';

const fonts: any = {};
let fontsLoaded = false;
const loader = new FontLoader();
const font = loader.parse(fontJson);
fonts.gentilis_bold = font;
fontsLoaded = true;

export const TEXT_ALIGN = {
    CENTER: 'center',
    LEFT: 'left',
};

const LETTER_OFFSET = 0.05;

export default class Text3D {
    private charMeshes;

    private charWidths;

    private charPointers;

    private camera;

    private meshBasicMaterialMemo = new Map();

    private chartMemo = new Map();

    constructor(camera) {
        this.charMeshes = {};
        this.charPointers = {};
        this.charWidths = {};
        this.camera = camera;
    }

    getText(char, size, height) {
        const key = `${char}_${size}_${height}`;
        const text = this.chartMemo.get(key);
        if (text) {
            return text;
        }
        this.chartMemo.set(
            key,
            new TextGeometry(char, {
                font,
                size,
                height,
                curveSegments: 1,
            }),
        );
        return this.chartMemo.get(key);
    }

    getMeshBasicMaterial(color) {
        const material = this.meshBasicMaterialMemo.get(color);
        if (material) {
            return material;
        }
        this.meshBasicMaterialMemo.set(color, new THREE.MeshBasicMaterial({ color, side: THREE.FrontSide }));
        return this.meshBasicMaterialMemo.get(color);
    }

    reset() {
        this.charPointers = {};
    }

    drawText(text, color, position) {
        const textMesh = this.composeText(text, color);
        if (textMesh === null) {
            return null;
        }
        if (this.camera !== undefined) {
            textMesh.quaternion.copy(this.camera.quaternion);
        }
        // eslint-disable-next-line no-return-assign
        textMesh.children.forEach((c) => (c.visible = true));
        textMesh.visible = true;
        textMesh.position.set(position.x, position.y, position.z || 0);
        return textMesh;
    }

    composeText(text, color) {
        if (!fontsLoaded) {
            return null;
        }
        // 32 is the ASCII code for white space.
        const charIndices = map(text, (l) => l.charCodeAt(0) - 32);
        const textMesh = new THREE.Object3D();
        let offsetSum = 0;

        for (let j = 0; j < charIndices.length; j += 1) {
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
                    const { charMesh, charWidth } = this.drawChar3D(text[j], color);
                    mesh = charMesh;
                    this.charWidths[idx] = Number.isFinite(charWidth) ? charWidth : 0.2;
                }
                this.charMeshes[idx].push(mesh);
            }

            mesh.position.set(offsetSum, 0, 0);
            offsetSum = offsetSum + this.charWidths[idx] + LETTER_OFFSET;
            this.charPointers[idx] += 1;
            textMesh.add(mesh);
        }

        const offset = offsetSum / 2;
        textMesh.children.forEach((child) => {
            child.position.setX(child.position.x - offset);
        });

        return textMesh;
    }

    drawChar3D(char, color, font = fonts.gentilis_bold, size = 0.6, height = 0) {
        const charGeo = this.getText(char, size, height);
        const charMaterial = this.getMeshBasicMaterial(color);
        const charMesh = new THREE.Mesh(charGeo, charMaterial);

        charGeo.computeBoundingBox();
        const { max, min } = charGeo.boundingBox;

        return { charMesh, charWidth: max.x - min.x };
    }
}
