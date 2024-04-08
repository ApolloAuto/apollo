import { TagProps } from 'antd';
import { Tag } from '.';

export function Template(props: TagProps) {
    return (
        <div>
            <Tag color='#2db7f5'>#2db7f5</Tag>
            <Tag color='#87d068'>#87d068</Tag>
            <Tag color='#108ee9'>#108ee9</Tag>
        </div>
    );
}
