import { useEffect, useState } from 'react';
import './App.scss';
import ReactJson from 'react-json-view';

function App() {
    const [data, setData] = useState([]);

    useEffect(() => {
        // 监听updateData数据， chrome.runtime.sendMessage({type: "updateData", payload: data});
        // eslint-disable-next-line no-undef
        chrome.runtime.onMessage.addListener((request, sender, sendResponse) => {
            if (request.type === 'updateData') {
                const payload = request.payload;
                setData(payload);
            }
        });
        // const channel = new BroadcastChannel('dreamview-analysis');
        // channel.onmessage = (event) => {
        //     console.log(event)
        //     if (event.type === 'updateData') {
        //         const data = event?.payload;
        //         setData(data);
        //     }
        // };
    }, []);
    return (
        <div className='debug-app-container'>
            <ReactJson src={data} theme='monokai' collapsed={2} enableClipboard />
        </div>
    );
}

export default App;
