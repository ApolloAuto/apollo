import React from 'react';
import { FormProps } from 'antd';
import { Form } from '.';
import { Input } from '../Input';
import { Switch } from '../Switch';

export function Template(
    props: FormProps & {
        children?: React.ReactNode;
    },
) {
    const [form] = Form.useForm();

    return (
        <div>
            <Form form={form} name='form'>
                <Form.Item
                    label='Name'
                    name='name'
                    rules={[
                        ({ getFieldValue }) => ({
                            validator(_, value) {
                                // 必填校验逻辑
                                if (!value) {
                                    return Promise.reject(new Error('Please enter'));
                                }
                                return Promise.resolve();
                            },
                        }),
                    ]}
                >
                    <div>
                        <Input placeholder='Please enter' />
                    </div>
                </Form.Item>
                <Form.Item label='switch' name='switch'>
                    <Switch />
                </Form.Item>
            </Form>
            ;
        </div>
    );
}
