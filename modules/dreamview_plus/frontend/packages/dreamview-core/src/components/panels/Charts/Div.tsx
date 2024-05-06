import React from 'react';

export interface IfComponentsProps {
    rif?: boolean;
}
export interface ShowComponentsProps {
    rshow?: boolean;
}

export function ShowComponents<ComponentProps>(Component: React.FC<any>) {
    return function Comp(props: ComponentProps & ShowComponentsProps & React.Attributes) {
        const { rshow, ...rest } = props;
        return (
            <div style={{ display: rshow ? 'block' : 'none' }}>
                <Component {...rest} />
            </div>
        );
    };
}

export function IfComponents<ComponentProps>(Component: React.FC<any>) {
    return function Comp(props: ComponentProps & IfComponentsProps & React.Attributes) {
        const { rif, ...rest } = props;
        if (!rif) return null;
        return <Component {...rest} />;
    };
}

function DivComponent(props: React.Attributes) {
    return <div {...props} />;
}

const DivIfComponent = IfComponents<React.Attributes>(DivComponent);
interface Child {
    children?: any;
}
export function Div(props: IfComponentsProps & React.HTMLAttributes<HTMLDivElement> & Child) {
    if ('rif' in props) {
        return <DivIfComponent {...props} />;
    }
    return <DivComponent {...props} />;
}
