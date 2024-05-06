import React from 'react';
import { CardProps } from 'antd';
import { Card } from '.';

export function Template(props: CardProps) {
    return (
        <Card
            hoverable
            style={{
                width: 260,
            }}
        >
            <p>Card content</p>
            <p>Card content</p>
            <p>Card content</p>
        </Card>
    );
}
