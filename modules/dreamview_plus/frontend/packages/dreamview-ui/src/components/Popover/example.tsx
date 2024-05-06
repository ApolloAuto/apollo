import React from 'react';
import { Popover } from '.';
import { Button } from '../Button';

const text = <span>Title</span>;

const content = (
    <div>
        <p>Content</p>
        <p>Content</p>
    </div>
);

const buttonWidth = 70;

// eslint-disable-next-line react/function-component-definition
export const Position: React.FC = () => (
    <div>
        <div style={{ marginLeft: buttonWidth, whiteSpace: 'nowrap' }}>
            <Popover placement='topLeft' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>TL</Button>
            </Popover>
            <Popover placement='top' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>Top</Button>
            </Popover>
            <Popover placement='topRight' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>TR</Button>
            </Popover>
        </div>
        <div style={{ width: buttonWidth, float: 'left' }}>
            <Popover placement='leftTop' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>LT</Button>
            </Popover>
            <Popover placement='left' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>Left</Button>
            </Popover>
            <Popover placement='leftBottom' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>LB</Button>
            </Popover>
        </div>
        <div style={{ width: buttonWidth, marginLeft: buttonWidth * 4 + 24 }}>
            <Popover placement='rightTop' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>RT</Button>
            </Popover>
            <Popover placement='right' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>Right</Button>
            </Popover>
            <Popover placement='rightBottom' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>RB</Button>
            </Popover>
        </div>
        <div style={{ marginLeft: buttonWidth, clear: 'both', whiteSpace: 'nowrap' }}>
            <Popover placement='bottomLeft' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>BL</Button>
            </Popover>
            <Popover placement='bottom' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>Bottom</Button>
            </Popover>
            <Popover placement='bottomRight' title={text} content={content} trigger='click'>
                <Button style={{ width: 80, marginLeft: '8px', marginBottom: '8px' }}>BR</Button>
            </Popover>
        </div>
    </div>
);
