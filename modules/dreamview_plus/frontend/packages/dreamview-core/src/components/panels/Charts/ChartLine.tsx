import { makeStyles } from '@dreamview/dreamview-theme';
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import tinycolor from 'tinycolor2';
import { FormListFieldData } from 'antd';
import { Form, Input, IconPark, ColorPicker, Select, Popover } from '@dreamview/dreamview-ui';
import { SelectProps } from '@dreamview/dreamview-ui/src/components/Select';
import { useTranslation } from 'react-i18next';
import { ProtoLoader } from '@dreamview/dreamview-core/src/util/ProtoLoader';
import jsonDescribeToJson from '@dreamview/dreamview-core/src/util/jsonDescribeToJson';
import lodashGet from 'lodash/get';
import CustomPopover from '../../CustomPopover';
import { KEY, HIDDEL_OPTION, LINE_WIDTH, IChannelList, IChannelListItem, IChartListItem } from './const';

const useStyle = makeStyles((theme) => ({
    'chart-line-collapse': {
        position: 'relative',
        marginBottom: theme.tokens.margin.speace2,
        border: '0',
        '& .ant-form-item:last-of-type': {
            marginBottom: '0',
        },
        '& .dreamview-select': {
            width: '100% !important',
            height: '40px',
        },
        '& .ant-form-item-control': {
            width: '228px',
            flexGrow: 'unset',
        },
    },
    'chart-line-collapse-title': {
        display: 'flex',
        alignItems: 'center',
        border: theme.components.pncMonitor.chartLineBorder,
        borderTopLeftRadius: '6px !important',
        borderTopRightRadius: '6px !important',
        height: '40px',
        padding: `0 ${theme.tokens.padding.speace2}`,
        backgroundColor: theme.components.pncMonitor.chartBgColor,
        overflow: 'hidden',
        color: theme.tokens.colors.fontColor5,
        cursor: 'default',
    },
    'chart-line-collapse-content': {
        padding: theme.tokens.padding.speace2,
        backgroundColor: theme.components.pncMonitor.chartBgColor,
        border: theme.components.pncMonitor.chartLineBorder,
        borderTop: 'none',
        borderBottomLeftRadius: '8px',
        borderBottomRightRadius: '8px',
        '& .ant-form-item-control': {
            width: '228px',
        },
    },
    'chart-line-collapse-expand': {
        overflow: 'hidden',
        transition: theme.tokens.transitions.easeInOut('all'),
    },
    'arrow-down': {
        position: 'absolute',
        right: '16px',
        '& svg': {
            transition: theme.tokens.transitions.easeInOut('all'),
        },
    },

    'form-hidden-ic': {
        marginLeft: theme.tokens.margin.speace3,
        '& .ant-form-item-control-input': {
            minHeight: '0',
            height: 'auto',
        },
        '& .ant-form-item-control-input-content': {
            display: 'flex',
            alignItems: 'center',
        },
    },

    'line-color': {
        fontSize: '10px',
    },
    'line-title': {
        marginLeft: '6px',
    },
    'line-hidden-active': {
        display: 'block',
        // marginLeft: theme.tokens.margin.speace3,
        fontSize: theme.tokens.font.size.large,
        color: theme.tokens.colors.fontColor5,
    },
    'line-hidden-unactive': {
        display: 'block',
        // marginLeft: theme.tokens.margin.speace3,
        fontSize: theme.tokens.font.size.large,
        color: tinycolor(theme.tokens.colors.fontColor5).setAlpha(0.7).toRgbString(),
    },
    'line-delete': {
        marginLeft: theme.tokens.margin.speace2,
        fontSize: theme.tokens.font.size.large,
    },
}));
function IcArrowsDown(props: any) {
    const { isActive, className } = props;
    return <IconPark name='IcArrowsDown' className={className} rotate={isActive ? 180 : 0} />;
}

interface ChartLineProps {
    filed: FormListFieldData;
    add: () => void;
    remove: (name: number | number[]) => void;
    index: number;
    channelList: IChannelList;
    activeChartConfig: IChartListItem;
}

const lineWidthOption = [
    {
        label: 1,
        value: LINE_WIDTH.sm,
    },
    {
        label: 2,
        value: LINE_WIDTH.md,
    },
    {
        label: 3,
        value: LINE_WIDTH.lg,
    },
];

const useMyColorPickerStyle = makeStyles((theme) => ({
    'my-popover': {
        '& .dreamview-popover-inner': {
            background: 'rgba(255,77,88,0.25)',
        },
        '& .dreamview-popover-arrow::before': {
            background: theme.tokens.colors.transparent,
        },
        '& .dreamview-popover-arrow::after': {
            background: 'rgba(255,77,88,0.25)',
        },
        '& .dreamview-popover-inner-content': {
            color: '#FF4D58',
            ...theme.tokens.typography.content,
        },
        '& .anticon': {
            marginRight: '6px',
        },
    },
    'my-color-picker': {
        position: 'relative',
        height: '40px',
        background: theme.tokens.colors.background1,
        borderRadius: '6px',
        padding: `0 ${theme.tokens.padding.speace2}`,
        display: 'flex',
        alignItems: 'center',
        color: theme.tokens.colors.fontColor5,
        ...theme.tokens.typography.content,
        '& .ant-color-picker-trigger': {
            height: '16px',
            width: '16px',
            minWidth: '0',
            padding: 0,
            background: 'none',
            boxShadow: 'none',
            border: 'none',
            position: 'relative',
            zIndex: 2,
        },
        '& .ant-color-picker-color-block': {
            height: '16px',
            width: '16px',
        },
        '& input': {
            position: 'absolute',
            left: '0',
            right: '0',
            top: '0',
            bottom: '0',
            width: '100%',
            height: '100%',
            paddingLeft: '48px',
            transition: 'all 0.2s',
            border: theme.components.pncMonitor.chartEditingColorPickerBorder,
            borderRadius: '6px',
            background: theme.components.pncMonitor.pickerBgColor,
            outline: 'none',
            caretColor: theme.tokens.colors.brand3,
            color: theme.tokens.colors.fontColor5,
            ...theme.tokens.typography.content,
            '&:hover': {
                border: theme.components.pncMonitor.chartEditingColorPickerActiveBorder,
                boxShadow: theme.components.pncMonitor.chartEditingColorPickerBoxShadow,
            },
            '&:focus': {
                border: theme.components.pncMonitor.chartEditingColorPickerActiveBorder,
                boxShadow: theme.components.pncMonitor.chartEditingColorPickerBoxShadow,
            },
        },
    },
    'text-i': {
        marginTop: '2px',
    },
}));
interface MyColorPickerProps {
    value?: string;
    onChange?: (value: string) => void;
    // 为了不和form的用法冲突
    onValueChange?: (value: string) => void;
}

function MyColorPicker(props: MyColorPickerProps) {
    const { classes } = useMyColorPickerStyle();
    const { value, onChange: propOnChange, onValueChange } = props;
    const [isInputActive, setIsInputActive] = useState(false);
    const [tempValue, setTempValue] = useState(value);
    const [open, setOpen] = useState(false);
    const { t } = useTranslation('chartEditing');
    const realValue = isInputActive ? tempValue : value;

    const onChange = (val: string) => {
        if (propOnChange) {
            propOnChange(val);
        }
        if (onValueChange) {
            onValueChange(val);
        }
    };

    const onColorPickerChange = (_: any, hex: string) => {
        onChange(hex);
        setTempValue(hex);
    };

    const onInputChange = (e: any) => {
        let val = '';
        if (e?.target?.value) {
            val = e.target.value;
        }
        setTempValue(val);
    };
    const onFocus = () => {
        setIsInputActive(true);
    };

    const isColorValidate = () => tinycolor(tempValue).isValid();

    const timer = useRef<number>(0);
    const onBlur = () => {
        const isValidate = isColorValidate();
        if (isValidate) {
            onChange(tempValue);
        } else {
            setTempValue(value);
            setOpen(true);
            clearTimeout(timer.current);
            timer.current = window.setTimeout(() => {
                setOpen(false);
                clearTimeout(timer.current);
            }, 3000);
        }
        setIsInputActive(false);
    };

    const errorContext = (
        <span>
            <IconPark name='IcDownloadingCancel' />
            {t('invalidColor')}
        </span>
    );
    return (
        <Popover rootClassName={classes['my-popover']} open={open} content={errorContext}>
            <div className={classes['my-color-picker']}>
                <ColorPicker value={realValue} onChange={onColorPickerChange} />
                <span className={classes['text-i']}>&nbsp;#&nbsp;</span>
                <input
                    autoComplete='off'
                    onBlur={onBlur}
                    onFocus={onFocus}
                    value={(realValue || '').replace('#', '')}
                    onChange={onInputChange}
                />
            </div>
        </Popover>
    );
}

interface HiddenIcProps {
    value?: any;
    onChange?: (value: any) => void;
}

function HiddenIc(props: HiddenIcProps) {
    const { value, onChange } = props;
    const { classes } = useStyle();
    const { t } = useTranslation('chartEditing');

    const onHideClick = (e: any) => {
        if (e.stopPropagation) {
            e.stopPropagation();
        }
        if (onChange) {
            onChange(HIDDEL_OPTION.HIDE);
        }
    };

    const onShowClick = (e: any) => {
        if (e.stopPropagation) {
            e.stopPropagation();
        }
        if (onChange) {
            onChange(HIDDEL_OPTION.SHOW);
        }
    };

    const ic =
        value === HIDDEL_OPTION.SHOW ? (
            <IconPark name='IcVisual' onClick={onHideClick} className={classes['line-hidden-active']} />
        ) : (
            <IconPark name='IcNotVisual' onClick={onShowClick} className={classes['line-hidden-unactive']} />
        );
    const content = value === HIDDEL_OPTION.SHOW ? t('hideLine') : t('showLine');

    return (
        <CustomPopover trigger='hover' content={content}>
            <div>{ic}</div>
        </CustomPopover>
    );
}

interface FormHiddenIcProps {
    filed: FormListFieldData;
}

function FormHiddenIc(props: FormHiddenIcProps) {
    const { filed } = props;
    const { classes } = useStyle();

    return (
        <>
            <Form.Item className={classes['form-hidden-ic']} name={[filed.name, KEY.lineHidden]}>
                <HiddenIc />
            </Form.Item>
        </>
    );
}

type MySelectProp<T> = {
    onValueChange: (val: T) => void;
    onChange?: (name: string) => void;
} & SelectProps;

function MySelect<T>(props: MySelectProp<T>) {
    const { onValueChange, onChange, ...rest } = props;

    const onChangeHandler = (item: any, option: any) => {
        onValueChange(option);
        if (onChange) {
            onChange(item);
        }
    };

    return <Select onChange={onChangeHandler} {...rest} />;
}

export default function ChartLine(props: ChartLineProps) {
    const { index, filed, remove, channelList, activeChartConfig } = props;
    const { classes } = useStyle();
    const form = Form.useFormInstance();
    const { t } = useTranslation('chartEditing');
    const [activeChannel, setActiveChannel] = useState<IChannelListItem>();
    const [protoLoader] = useState(() => new ProtoLoader());
    const [openStatus, setOpenStatus] = useState(true);

    const onToogleOpenStatus = () => {
        setOpenStatus((prev) => !prev);
    };

    const [channelListChild, setChannelListChild] = useState([]);
    useEffect(() => {
        if (activeChannel) {
            protoLoader
                .getProtoDescriptor(activeChannel.dataName, activeChannel.channelName)
                .then((jsonDescribe) => {
                    if (jsonDescribe) {
                        const json = jsonDescribeToJson(jsonDescribe, activeChannel.msgType);
                        setChannelListChild(json.map((field) => ({ label: field, value: field })));
                    } else {
                        setChannelListChild([]);
                    }
                })
                .catch((err) => {
                    setChannelListChild([]);
                });
        }

        return () => {
            // donothing
        };
    }, [activeChannel]);

    // activeChartConfig变更时初始化activeChannel
    useEffect(() => {
        if (channelList.length && activeChartConfig?.uid) {
            const lineChannel = lodashGet(activeChartConfig.value, [KEY.lineList, filed.name, KEY.lineChannel]);
            const activeLineChannel = channelList.find((item) => item.value === lineChannel);
            setActiveChannel(activeLineChannel);
        }
    }, [channelList, activeChartConfig?.uid]);

    const [lineColor, setLineColor] = useState<string>(() =>
        form.getFieldValue([KEY.lineList, filed.name, KEY.lineColor]),
    );

    const handleColorPickerChange = useCallback((color: string) => {
        setLineColor(color);
    }, []);

    const onChannelChangeHandler = useCallback((value: IChannelListItem) => {
        setActiveChannel(value);
        form.setFieldValue([KEY.lineList, filed.name, KEY.lineDataName], value?.dataName);
        form.setFieldValue([KEY.lineList, filed.name, KEY.lineChannelX], undefined);
        form.setFieldValue([KEY.lineList, filed.name, KEY.lineChannelY], undefined);
    }, []);

    const onDeleteClick = (e: any) => {
        e.preventDefault();
        if (e.stopPropagation) {
            e.stopPropagation();
        }
        remove(filed.name);
    };

    const title = (
        <>
            <IconPark name='IcClassificationNotes' style={{ color: lineColor }} className={classes['line-color']} />
            <span className={classes['line-title']}>
                {t('line')}
                {index + 1}
            </span>
            <FormHiddenIc filed={filed} />
            <CustomPopover trigger='hover' content={t('deleteLine')}>
                <IconPark name='IcDelete' onClick={onDeleteClick} className={classes['line-delete']} />
            </CustomPopover>
            <IcArrowsDown className={classes['arrow-down']} isActive={openStatus} />
        </>
    );

    const children = (
        <>
            <Form.Item name={[filed.name, KEY.lineDataName]} hidden>
                <Input />
            </Form.Item>
            <Form.Item name={[filed.name, KEY.lineChannel]} label={t('labelYAxisLineChannel')}>
                <MySelect
                    onValueChange={onChannelChangeHandler}
                    popupMatchSelectWidth={false}
                    showSearch
                    options={channelList}
                    allowClear
                />
            </Form.Item>
            {!!channelListChild.length && (
                <>
                    <Form.Item name={[filed.name, KEY.lineChannelX]} label={t('labelYAxisLineChannelX')}>
                        <Select popupMatchSelectWidth={false} showSearch options={channelListChild} allowClear />
                    </Form.Item>
                    <Form.Item name={[filed.name, KEY.lineChannelY]} label={t('labelYAxisLineChannelY')}>
                        <Select popupMatchSelectWidth={false} showSearch options={channelListChild} allowClear />
                    </Form.Item>
                </>
            )}

            <Form.Item name={[filed.name, KEY.lineName]} label={t('labelYAxisLineName')}>
                <Input allowClear autoComplete='off' />
            </Form.Item>
            <Form.Item name={[filed.name, KEY.lineWidth]} label={t('labelYAxisLineWidth')}>
                <Select allowClear options={lineWidthOption} />
            </Form.Item>
            <Form.Item name={[filed.name, KEY.lineColor]} label={t('labelYAxisLineColor')}>
                <MyColorPicker onValueChange={handleColorPickerChange} />
            </Form.Item>
        </>
    );

    const height = channelListChild.length ? '353px' : '241px';

    return (
        <div className={classes['chart-line-collapse']}>
            <div onClick={onToogleOpenStatus} className={classes['chart-line-collapse-title']}>
                {title}
            </div>
            <div style={{ height: openStatus ? height : 0 }} className={classes['chart-line-collapse-expand']}>
                <div className={classes['chart-line-collapse-content']}>{children}</div>
            </div>
        </div>
    );
}
