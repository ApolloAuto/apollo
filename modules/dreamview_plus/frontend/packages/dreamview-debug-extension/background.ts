chrome.runtime.onMessage.addListener(function (message, sender, sendResponse) {
    if (message.type === "updateIcon") {
        console.log("Received message", message, sender, sendResponse);
        if (message.enabled) {
            chrome.action.setIcon({
                path: {
                    "16": "public/icon/active_icon_16.png",
                    "48": "public/icon/active_icon_48.png"
                }
            })
            chrome.action.enable(sender?.tab?.id);
        } else {
            chrome.action.setIcon({
                path: {
                    "16": "public/icon/icon_16.png",
                    "48": "public/icon/icon_48.png"
                }
            })
            chrome.action.disable(sender?.tab?.id);
        }
    }
});

// tab切换触发content script
chrome.tabs.onActivated.addListener((activeInfo) => {
    chrome.tabs.sendMessage(activeInfo.tabId, {type: "tabActivated"});
});

chrome.commands.onCommand.addListener((command) => {
    if (command === "open-debug-window") {
        console.log("Opening debug window");
        // @ts-ignore
        chrome.windows.create({
            url: "pages/debug.html",
            type: "popup",
            focused: true,
            width: 600,
            height: 800
        });
    }
});
