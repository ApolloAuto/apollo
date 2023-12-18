import { Meta, StoryObj } from '@storybook/react';
import { CheckboxGroup } from './index';
import { Template } from './example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof CheckboxGroup> = {
    title: 'Dreamview/CheckboxGroup',
    component: CheckboxGroup,
    tags: ['autodocs'],
    argTypes: {},
};

export const Primary = Template.bind({});

export default meta;
