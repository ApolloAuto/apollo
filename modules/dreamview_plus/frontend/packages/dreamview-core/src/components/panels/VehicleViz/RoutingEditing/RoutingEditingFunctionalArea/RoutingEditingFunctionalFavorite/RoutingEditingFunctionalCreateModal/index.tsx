import React, { useEffect, useState } from 'react';
import { Modal, Form, Input, Switch, InputNumber, message } from '@dreamview/dreamview-ui';
import EventEmitter from 'eventemitter3';
import { useTranslation } from 'react-i18next';
import { RoutingEditor } from '@dreamview/dreamview-carviz';
import useStyle from './useStyle';
import { WithModalComponentProps } from '../../../../../../../util/modal';
import CustomScroll from '../../../../../../CustomScroll';
import { RouteOrigin } from '../../../RouteManager/type';
import { DefaultPoint, DefaultRouting, RoutingType } from '../../../../../../../services/api/types';
import { MainApi } from '../../../../../../../services/api';
import { RouteManager } from '../../../RouteManager';
import { BusinessEventTypes } from '../../../../../../../store/EventHandlersStore/eventType';
import { IPanelContext } from '../../../../../base/store/PanelStore';

interface RoutingEditingFunctionalCreateModalProps {
    EE: EventEmitter;
    mainApi: MainApi | null;
    isMainConnected: boolean;
    routeManager: RouteManager;
    routingEditor: RoutingEditor;
    panelContext: IPanelContext;
    defaultRoutingList: DefaultRouting[];
    createCommonRoutingSuccessFunctional: () => void;
}

export default function RoutingEditingFunctionalCreateModal(
    props: WithModalComponentProps<RoutingEditingFunctionalCreateModalProps>,
) {
    const {
        EE,
        destroy,
        mainApi,
        panelContext,
        routeManager,
        routingEditor,
        isMainConnected,
        defaultRoutingList,
        createCommonRoutingSuccessFunctional,
    } = props;

    const [form] = Form.useForm();

    const { cx, classes } = useStyle();

    const { t } = useTranslation('routeEditing');

    const routeManagerMix = routeManager.currentRouteMix;

    const currentRouteLoopState = routeManagerMix.getCurrentRouteMix().currentRouteLoop.currentRouteLoopState;

    const currentRouteLoopTimes = routeManagerMix.getCurrentRouteMix().currentRouteLoop?.currentRouteLoopTimes;

    const [routeWayPoint, setRouteWayPoint] = useState([]);

    const [routeInitialPoint, setRouteInitialPoint] = useState({ x: 0, y: 0 } as DefaultPoint);

    const onOk = () => {
        form.validateFields().then(() => {
            if (isMainConnected && mainApi) {
                const saveDefaultRoutingForm = form.getFieldsValue();
                if (saveDefaultRoutingForm.loopRouting) {
                    const postCycleNumber = Number(saveDefaultRoutingForm.cycleNumber);
                    saveDefaultRoutingForm.cycleNumber = postCycleNumber > 10 ? 10 : postCycleNumber;
                }
                delete saveDefaultRoutingForm.loopRouting;
                mainApi
                    .saveDefaultRouting({
                        ...saveDefaultRoutingForm,
                        routingType: RoutingType.DEFAULT_ROUTING,
                        point: [routeInitialPoint, ...routeWayPoint],
                    })
                    .then(() => {
                        EE.emit(BusinessEventTypes.SimControlRoute, {
                            panelId: panelContext.panelId,
                            routeInfo: {
                                initialPoint: routeInitialPoint,
                                wayPoint: routeWayPoint,
                                cycleNumber: saveDefaultRoutingForm?.cycleNumber,
                            },
                        });
                        createCommonRoutingSuccessFunctional();
                        destroy();
                        message({ type: 'success', content: t('createCommonRouteSuccess') });
                    });
            }
        });
    };

    const onCancel = () => {
        destroy();
    };

    useEffect(() => {
        const initialPoint = routingEditor.initiationMarker.initiationMarkerPosition;
        const wayPoint = routingEditor.pathwayMarker.pathWatMarkerPosition;
        if (initialPoint) {
            setRouteInitialPoint(initialPoint);
        } else {
            mainApi.getStartPoint().then((res) => {
                setRouteInitialPoint({ x: res.x, y: res.y, heading: res?.heading });
            });
        }
        setRouteWayPoint(wayPoint);
    }, [isMainConnected]);

    return (
        <Modal
            title={t('routeCreateCommonBtn')}
            okText={t('create')}
            cancelText={t('cancel')}
            open
            onOk={onOk}
            onCancel={onCancel}
            rootClassName={classes['routing-modal']}
        >
            <Form
                form={form}
                clearOnDestroy
                name='form'
                // onValuesChange={onValuesChange}
                className={classes['create-modal-form']}
                initialValues={{ loopRouting: currentRouteLoopState, cycleNumber: currentRouteLoopTimes }}
            >
                <Form.Item
                    label={t('name')}
                    style={{ marginLeft: '74px' }}
                    name='name'
                    rules={[
                        ({ getFieldValue }) => ({
                            validator(_, value) {
                                // 必填校验逻辑
                                if (!value) {
                                    return Promise.reject(new Error(t('pleaseEnter')));
                                }
                                // 重名逻辑校验
                                if (value) {
                                    if (defaultRoutingList.find((item) => item.name === value)) {
                                        return Promise.reject(new Error(t('alreadyExists')));
                                    }
                                }
                                return Promise.resolve();
                            },
                        }),
                    ]}
                >
                    <Input placeholder='Please enter' style={{ width: '252px', height: '40px' }} />
                </Form.Item>
                <div className={classes['routing-form-initial']}>
                    <div className={classes['routing-form-colon']}>
                        {t('initialPoint')}
                        <span className={classes['routing-form-colon-distance']}>:</span>
                    </div>

                    <div className={classes['routing-form-initial-content']}>
                        <div>{`[${routeInitialPoint.x.toFixed(3)} ，${routeInitialPoint.y.toFixed(3)}]`}</div>
                        <div className={classes['routing-form-initial-content-heading']}>
                            {routeInitialPoint?.heading ? routeInitialPoint.heading.toFixed(3) : '-'}
                        </div>
                    </div>
                </div>
                <CustomScroll className={classes['routing-form-way']}>
                    <div className={classes['routing-form-way-con']}>
                        <div className={classes['routing-form-colon']}>
                            {t('wayPoint')}
                            <span className={classes['routing-form-colon-distance']}>:</span>
                        </div>
                        <div className={classes['routing-form-way-content']}>
                            {routeWayPoint?.map((item, index) => (
                                <div
                                    key={`${item.x}${item.y}${index + 1}`}
                                    className={classes['routing-form-way-item']}
                                >
                                    <div>{`[${item.x.toFixed(3)}，${item.y.toFixed(3)}]`}</div>
                                    <div className={classes['routing-form-way-item-heading']}>
                                        {item?.heading ? item.heading.toFixed(3) : '-'}
                                    </div>
                                </div>
                            ))}
                        </div>
                    </div>
                </CustomScroll>
                <Form.Item
                    label={t('loopRouting')}
                    style={{ marginLeft: '16px' }}
                    name='loopRouting'
                    valuePropName='checked'
                >
                    <Switch className={classes['routing-form-loop-disable']} />
                </Form.Item>
                <Form.Item shouldUpdate>
                    {({ getFieldValue }) => {
                        const isLoopRouting = getFieldValue('loopRouting');
                        return isLoopRouting ? (
                            <Form.Item
                                label={t('setLooptimes')}
                                style={{ marginLeft: '11px' }}
                                name='cycleNumber'
                                rules={[
                                    () => ({
                                        validator(_, value) {
                                            // 必填校验逻辑
                                            if (!value) {
                                                return Promise.reject(new Error('Please enter'));
                                            }
                                            if (Number(value) > 10) {
                                                return Promise.reject(new Error('Max loop times is 10'));
                                            }
                                            return Promise.resolve();
                                        },
                                    }),
                                ]}
                            >
                                <InputNumber type='number' max={10} precision={0} />
                            </Form.Item>
                        ) : null;
                    }}
                </Form.Item>
            </Form>
        </Modal>
    );
}
