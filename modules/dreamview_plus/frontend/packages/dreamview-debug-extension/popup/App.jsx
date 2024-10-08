import React, { useEffect, useState } from 'react';

function App() {
    const [version, setVersion] = useState('');

    useEffect(() => {
        // 监听updateIcon事件，获取版本号
        // eslint-disable-next-line no-undef
        chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
            if (request.type === 'updateIcon') {
                console.log('updateIcon', request.payload?.version);
                const theVersion = request.payload?.version;
                setVersion(theVersion);
            }
        });
    }, []);

    return (
        <div>
            <h1>Dreamview Plus</h1>
            <pre>
                version:
                {version}
            </pre>
        </div>
    );
}

export default App;
