import React from 'react';
import { Form as InternalForm, FormProps } from 'antd';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function Form(
    props: FormProps & {
        children?: React.ReactNode;
    },
) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('form', customizePrefixCls);

    return (
        <InternalForm prefixCls={prefixCls} {...rest}>
            {children}
        </InternalForm>
    );
}

Form.propTypes = {};

Form.defaultProps = {};

Form.displayName = 'Form';

Form.useFormInstance = InternalForm.useFormInstance;

Form.Item = InternalForm.Item;
Form.List = InternalForm.List;

Form.useForm = () => InternalForm.useForm();
