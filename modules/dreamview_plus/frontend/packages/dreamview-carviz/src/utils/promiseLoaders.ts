import { Texture, TextureLoader } from 'three';

export function promiseTextureLoader(texturePath: string): Promise<Texture> {
    return new Promise((resolve, reject) => {
        new TextureLoader().load(
            texturePath,
            (texture) => {
                resolve(texture);
            },
            undefined,
            (error) => {
                reject(error);
            },
        );
    });
}
