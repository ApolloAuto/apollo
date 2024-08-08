import React from 'react';
import ReactDOM from 'react-dom/client';
import './base.css';
import App from './component/App';

const root = ReactDOM.createRoot(document.getElementById('root'));

root.render(
    <React.StrictMode>
        <App />
    </React.StrictMode>,
);
