# `dreamview-analysis`

> Provide performance data collection and analysis capabilities for various front-end applications

## Usage

```
import { DreamviewAnalysis } from '@dreamview/dreamview-analysis';

DreamviewAnalysis.logData('componentName', { renderTime: 200 });
```

## Usage in React Projects

Import the `PerformanceMonitor` component and the `DreamviewAnalysis` service in your React application to display performance data:

```jsx
import React from 'react';
import ReactDOM from 'react-dom';
import { DreamviewAnalysis, PerformanceMonitor } from 'dreamview-analysis';

function App() {
    return (
        <div>
            <h1>Welcome to the App</h1>
            <PerformanceMonitor service={DreamviewAnalysis} />
        </div>
    );
}

ReactDOM.render(<App />, document.getElementById('root'));
