import { Meta, StoryObj } from '@storybook/react';
import { Button } from './index';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Button> = {
    title: 'Dreamview/Button',
    component: Button,
    tags: ['autodocs'],
    argTypes: {},
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary: StoryObj<typeof Button> = {
    args: {
        // type: 'primary',
        size: 'middle',
    },
};

export const Secondary: StoryObj<typeof Button> = {
    args: {
        type: 'default',
    },
};

export default meta;
