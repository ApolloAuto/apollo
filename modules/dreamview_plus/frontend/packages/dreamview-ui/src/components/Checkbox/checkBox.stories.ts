import { Meta, StoryObj } from '@storybook/react';
import { Checkbox } from './checkBox';
import { Template } from './checkBox.example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof Checkbox> = {
    title: 'Dreamview/Checkbox',
    component: Checkbox,
    tags: ['autodocs'],
    argTypes: {},
};

export const CheckBox = Template.bind({});

export default meta;
