import React, { PropsWithChildren } from 'react';

interface ICombineContext {
    providers: JSX.Element[];
}

export default function CombineContext(props: PropsWithChildren<ICombineContext>) {
    const { providers, children } = props;
    const wrapped = providers.reduceRight(
        (wrappedChildren, provider) => React.cloneElement(provider, undefined, wrappedChildren),
        children,
    );
    // eslint-disable-next-line react/jsx-no-useless-fragment
    return <>{wrapped}</>;
}
