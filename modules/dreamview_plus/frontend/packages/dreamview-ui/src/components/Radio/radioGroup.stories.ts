import { Meta, StoryObj } from '@storybook/react';
import { RadioGroup } from './radioGroup';
import { RadioGroupTemplate } from './radioGroup.example';

// More on how to set up stories at: https://storybook.js.org/docs/react/writing-stories/introduction
const meta: Meta<typeof RadioGroup> = {
    title: 'Dreamview/RadioGroup',
    component: RadioGroup,
    tags: ['autodocs'],
    argTypes: {},
};

// More on writing stories with args: https://storybook.js.org/docs/react/writing-stories/args
export const Primary = RadioGroupTemplate.bind({});

export default meta;
