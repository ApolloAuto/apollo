import { Meta, StoryObj } from '@storybook/react';
import { Component } from './index';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Component> = {
    title: 'Dreamview/Component',
    component: Component,
    tags: ['autodocs'],
    argTypes: {
        backgroundColor: { control: 'color' },
    },
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary: StoryObj<typeof Component> = {
    args: {
        primary: true,
        label: 'Primary',
        backgroundColor: 'blue',
    },
};

export const Secondary: StoryObj<typeof Component> = {
    args: {
        label: 'Secondary',
    },
};

export default meta;
