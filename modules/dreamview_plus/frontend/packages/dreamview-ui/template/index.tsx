import React from 'react';
import PropTypes from 'prop-types';
import './index.less';

type Props = {
    primary: boolean;
    backgroundColor: string;
    size: string;
    label: string;
    onClick: () => any;
    children: React.ReactNode;
};

export function Component(props: Props) {
    const { children, backgroundColor } = props;
    return (
        <div className={['.storybook-component', `.storybook-component-${backgroundColor}`].join(' ')}>{children}</div>
    );
}

Component.propTypes = {
    /**
     * Is this the principal call to action on the page?
     */
    primary: PropTypes.bool,
    /**
     * What background color to use
     */
    backgroundColor: PropTypes.oneOf(['blue', 'red']),
    /**
     * How large should the button be?
     */
    size: PropTypes.oneOf(['small', 'medium', 'large']),
    /**
     * Button contents
     */
    label: PropTypes.string.isRequired,
    /**
     * Optional click handler
     */
    onClick: PropTypes.func,
};

Component.defaultProps = {
    backgroundColor: null,
    primary: true,
    size: 'medium',
    onClick: undefined,
};

Component.displayName = 'test';
