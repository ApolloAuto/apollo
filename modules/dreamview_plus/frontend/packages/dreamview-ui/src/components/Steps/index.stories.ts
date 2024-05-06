import { Meta, StoryObj } from '@storybook/react';
import { Template } from './example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Template> = {
    title: 'Dreamview/Steps',
    component: Template,
    tags: ['autodocs'],
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary = Template.bind({});

export default meta;
