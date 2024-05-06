import React from 'react';
import { createRoot } from 'react-dom/client';
import { ThemeProvider } from '@dreamview/dreamview-core/src/store/ThemeProviderStore';

export interface ModalComponentProps {
    destroy: () => void;
}
export type WithModalComponentProps<T = unknown> = T & ModalComponentProps;

export default function showModal<T>(props: T, Component: (values: WithModalComponentProps<T>) => React.ReactElement) {
    const elem = document.createElement('div');
    const rootElem = document.querySelector('body') as Element;
    rootElem.append(elem);
    const root = createRoot(elem);
    const destroy = () => {
        root.unmount();
        if (elem?.parentNode) {
            elem.parentNode.removeChild(elem);
        }
    };

    const nextProps = {
        ...props,
        destroy,
    };

    root.render(
        <ThemeProvider>
            <Component {...nextProps} />
        </ThemeProvider>,
    );
}
