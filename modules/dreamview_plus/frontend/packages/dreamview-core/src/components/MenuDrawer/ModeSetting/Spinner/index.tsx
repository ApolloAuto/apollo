import React from 'react';
import './style.css';

export function Spinner() {
    return (
        <svg className='spinner' width='1em' height='1em' viewBox='0 0 66 66'>
            <circle fill='none' strokeWidth='6' strokeLinecap='round' stroke='#2D3140' cx='33' cy='33' r='30' />
            <circle className='path' fill='none' strokeWidth='6' strokeLinecap='round' cx='33' cy='33' r='30' />
        </svg>
    );
}
