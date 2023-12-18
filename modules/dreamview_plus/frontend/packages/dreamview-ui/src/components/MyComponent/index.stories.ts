import { Meta, StoryObj } from '@storybook/react';
import { MyComponent } from './index';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof MyComponent> = {
    title: 'Dreamview/MyComponent',
    component: MyComponent,
    tags: ['autodocs'],
    argTypes: {
        backgroundColor: { control: 'color' },
    },
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary: StoryObj<typeof MyComponent> = {
    args: {
        primary: true,
        label: 'Primary',
        backgroundColor: 'blue',
    },
};

export const Secondary: StoryObj<typeof MyComponent> = {
    args: {
        label: 'Secondary',
        backgroundColor: 'red',
    },
};

export default meta;
