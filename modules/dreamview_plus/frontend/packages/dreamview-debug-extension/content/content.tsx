import React from 'react';
import ReactDOM from 'react-dom/client';

function initial() {
    // // Create a new div element and append it to the document's body
    // const rootDiv = document.createElement('div')
    // rootDiv.id = 'extension-root'
    // document.body.appendChild(rootDiv)
    //
    // // Use `createRoot` to create a root, then render the <App /> component
    // // Note that `createRoot` takes the container DOM node, not the React element
    // const root = ReactDOM.createRoot(rootDiv)
    // root.render(<Root />)

    // 判断页面id为dreamviewVersion的元素是否存在
    const isDebuggable = !!document.getElementById('dreamviewVersion');
    // @ts-ignore
    const version = window.dreamviewVersion;
    console.log('isDebuggable:', isDebuggable, 'version:', version);
    chrome.runtime.sendMessage({ type: 'updateIcon', enabled: isDebuggable, payload: { version } });

    document.addEventListener('dreamview-analysis', (event) => {
        // 发送前判断是否存在监听器
        chrome.runtime.sendMessage({
            type: 'updateData',
            // @ts-ignore
            payload: event.detail,
        });
    });
}

// tab切换触发content script
chrome.runtime.onMessage.addListener((message, sender, sendResponse) => {
    if (message.type === 'tabActivated') {
        initial();
    }
});

setTimeout(initial, 1000);
