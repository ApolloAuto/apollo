import React, { useState, useMemo, useContext, createContext, ReactNode, useEffect } from 'react';

export type RouteType = {
    path: string;
    component: ReactNode;
    keepAlive?: boolean;
};

export type RouterContextType = {
    currentPath: string;
    navigate: (path: string) => void;
};

export const RouterContext = createContext<RouterContextType | undefined>(undefined);

type SimpleRouterProps = {
    initialPath?: string;
    children?: ReactNode;
};

export const SimpleRouter: React.FC<SimpleRouterProps> = function ({ initialPath = '/', children }) {
    const [currentPath, setCurrentPath] = useState(initialPath);

    const contextValue = useMemo(
        () => ({
            currentPath,
            navigate: setCurrentPath,
        }),
        [currentPath],
    );

    return <RouterContext.Provider value={contextValue}>{children}</RouterContext.Provider>;
};

type RouteProps = {
    path: string;
    children?: ReactNode;
};

export const Route: React.FC<RouteProps> = function ({ path, children }) {
    const routerContext = useContext(RouterContext);

    if (!routerContext) {
        throw new Error('Route must be used within SimpleRouter');
    }

    const isMatch = routerContext.currentPath === path;

    return isMatch ? <>{children}</> : null;
};

export const useNavigate = () => {
    const routerContext = useContext(RouterContext);
    if (!routerContext) {
        throw new Error('useNavigate must be used within SimpleRouter');
    }
    return routerContext.navigate;
};

type KeepAliveProps = {
    path: string;
    style?: React.CSSProperties;
    children?: ReactNode;
};

export const KeepAlive: React.FC<KeepAliveProps> = function ({ path, style, children }) {
    const routerContext = useContext(RouterContext);

    if (!routerContext) {
        throw new Error('KeepAlive must be used within SimpleRouter');
    }

    const isMatch = routerContext.currentPath === path;

    const [hasBeenMounted, setHasBeenMounted] = useState(false);

    useEffect(() => {
        if (isMatch) {
            setHasBeenMounted(true);
        }
    }, [isMatch]);

    return hasBeenMounted ? <div style={{ ...style, display: isMatch ? 'block' : 'none' }}>{children}</div> : null;
};
