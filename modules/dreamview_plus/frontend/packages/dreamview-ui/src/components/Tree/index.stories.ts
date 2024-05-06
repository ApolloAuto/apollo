import { Meta, StoryObj } from '@storybook/react';
import { Tree } from './index';
import { Template } from './example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Tree> = {
    title: 'Dreamview/Tree',
    component: Tree,
    tags: ['autodocs'],
    argTypes: {
        // backgroundColor: { control: 'color' },
    },
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary = Template;

export default meta;
