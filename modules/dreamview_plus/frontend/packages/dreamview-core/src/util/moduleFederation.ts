export function loadRemoteEntry(remoteUrl: string, scope: string): Promise<void> {
    return new Promise((resolve, reject) => {
        // 避免重复加载同一远程应用
        if (window[scope]) {
            return resolve();
        }
        const script = document.createElement('script');
        script.src = remoteUrl;
        script.type = 'text/javascript';
        script.async = true;

        script.onload = () => {
            resolve();
        };

        script.onerror = () => {
            reject(new Error(`Failed to load remote entry script: ${remoteUrl}`));
        };

        document.head.appendChild(script);
    });
}
export async function loadComponent(scope: string, module: string): Promise<any> {
    // @ts-ignore
    await __webpack_init_sharing__('default');
    // @ts-ignore
    const container = window[scope];
    if (!container) {
        throw new Error(`Container not found for scope ${scope}`);
    }
    // @ts-ignore
    await container.init(__webpack_share_scopes__.default);

    const factory = await container.get(module);
    return factory();
}
