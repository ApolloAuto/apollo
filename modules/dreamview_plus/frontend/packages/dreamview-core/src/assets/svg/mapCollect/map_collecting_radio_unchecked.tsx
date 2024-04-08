import React from 'react';

export default function IcRadioUnchecked() {
    return (
        <svg xmlns='http://www.w3.org/2000/svg' width='16px' height='16px' viewBox='0 0 16 16' version='1.1'>
            <title>单选/未选中</title>
            <defs>
                <rect id='path-1' x='0' y='0' width='16' height='16' />
            </defs>
            <g id='单选/未选中' stroke='none' strokeWidth='1' fill='none' fillRule='evenodd'>
                <g id='ic_check'>
                    <mask id='mask-2' fill='white'>
                        <use xlinkHref='#path-1' />
                    </mask>
                    <use id='矩形' fill='#D8D8D8' opacity='0' xlinkHref='#path-1' />
                    <g id='ic_-勾选' mask='url(#mask-2)' stroke='#A6B5CC'>
                        <rect id='矩形' x='0.5' y='0.5' width='15' height='15' rx='7.5' />
                    </g>
                </g>
            </g>
        </svg>
    );
}
