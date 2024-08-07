import React, { useEffect, useMemo, useRef, useState, useLayoutEffect } from 'react';
import type { apollo } from '@dreamview/dreamview';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { isEmpty } from 'lodash';
import { useTranslation } from 'react-i18next';
import { useLocalStorage, KEY_MANAGER } from '@dreamview/dreamview-core/src/util/storageManager';
import useStyle from './useStyle';
import { usePanelContext } from '../base/store/PanelStore';
import { ObstacleTypeColorMap } from './utils';
import LayerMenu from './LayerMenu';
import { StreamDataNames } from '../../../services/api/types';
import Panel from '../base/Panel';
import CustomScroll from '../../CustomScroll';

type ICameraUpdate = apollo.dreamview.ICameraUpdate;

enum SubType {
    ST_UNKNOWN = 'ST_UNKNOWN',
    ST_UNKNOWN_MOVABLE = 'ST_UNKNOWN_MOVABLE',
    ST_UNKNOWN_UNMOVABLE = 'ST_UNKNOWN_UNMOVABLE',
    ST_CAR = 'ST_CAR',
    ST_VAN = 'ST_VAN',
    ST_TRUCK = 'ST_TRUCK',
    ST_BUS = 'ST_BUS',
    ST_CYCLIST = 'ST_CYCLIST',
    ST_MOTORCYCLIST = 'ST_MOTORCYCLIST',
    ST_TRICYCLIST = 'ST_TRICYCLIST',
    ST_PEDESTRIAN = 'ST_PEDESTRIAN',
    ST_TRAFFICCONE = 'ST_TRAFFICCONE',
}

function InternalCameraView() {
    const panelContext = usePanelContext();
    const { logger, panelId, onPanelResize, initSubscription } = panelContext;
    const localBoundingBoxManager = useLocalStorage(`${KEY_MANAGER.BBox}${panelId}`);

    const [showBoundingBox, setShowBoundingBox] = useState(() => localBoundingBoxManager.get());

    const [viewPortSize, setViewPortSize] = useState(null);
    const [data, setData] = useState<ICameraUpdate>();
    const canvasRef = useRef<HTMLCanvasElement>(null);

    useEffect(() => {
        initSubscription({
            [StreamDataNames.CAMERA]: {
                consumer: (camerData) => {
                    setData(camerData);
                },
            },
        });
    }, []);

    useLayoutEffect(() => {
        onPanelResize((width, height) => {
            setViewPortSize({ viewPortWidth: width, viewPortHeight: height });
        });
    }, []);

    useEffect(() => {
        if (isEmpty(data)) return;
        if (data?.image?.length > 0) {
            // 图片缩放比例
            const kImageScale = data.kImageScale;
            const imageData = new Uint8Array(data.image);
            const blob = new Blob([imageData], { type: 'image/jpeg' });
            // 使用 createImageBitmap 创建 ImageBitmap 对象

            createImageBitmap(blob)
                .then((imageBitmap) => {
                    // 获取 canvas 对象
                    const canvas = canvasRef.current;
                    if (!canvas) return;

                    const drawImageSize = {} as { drawImageWidth: number; drawImageHeight: number };

                    const flag =
                        imageBitmap.width / imageBitmap.height >
                        viewPortSize.viewPortWidth / viewPortSize.viewPortHeight;

                    const widthScale = viewPortSize.viewPortWidth / imageBitmap.width;
                    const heightScale = viewPortSize.viewPortHeight / imageBitmap.height;

                    const scale = flag ? widthScale : heightScale;

                    drawImageSize.drawImageWidth = imageBitmap.width * scale;
                    drawImageSize.drawImageHeight = imageBitmap.height * scale;

                    // 设置 canvas 尺寸
                    canvas.width = drawImageSize.drawImageWidth;
                    canvas.height = drawImageSize.drawImageHeight;

                    // 获取 2D 渲染上下文并绘制 ImageBitmap
                    const ctx = canvas.getContext('2d');
                    ctx.drawImage(imageBitmap, 0, 0, drawImageSize.drawImageWidth, drawImageSize.drawImageHeight);
                    // If there are obstacles
                    if (data.bbox2d && data.bbox2d.length > 0 && showBoundingBox) {
                        logger.debug(panelId, 'has obstacles');
                        data.bbox2d.forEach((bbox, index) => {
                            const obstaclesId = data.obstaclesId[index];
                            const obstaclesSubType = data.obstaclesSubType[index] as SubType;

                            ctx.strokeStyle = ObstacleTypeColorMap.get(obstaclesSubType) || 'red';

                            let { xmin, ymin, xmax, ymax } = bbox;
                            if (xmin === ymin && ymin === xmax && xmax === ymax) {
                                logger.debug(panelId, 'bbox is empty');
                                return;
                            }
                            [xmin, ymin, xmax, ymax] = [xmin, ymin, xmax, ymax].map(
                                (value) => value * (kImageScale ?? 1) * scale,
                            );
                            ctx.strokeRect(xmin, ymin, xmax - xmin, ymax - ymin);
                            ctx.fillStyle = ObstacleTypeColorMap.get(obstaclesSubType) || 'white';
                            ctx.font = '12px Arial';
                            ctx.fillText(`${obstaclesSubType.substring(3)}:${obstaclesId}`, xmin, ymin);
                        });
                    }
                })
                .catch((e) => {
                    // 错误处理
                    logger.error(e);
                });
        }
    }, [data, showBoundingBox, viewPortSize]);

    const { classes } = useStyle();
    return (
        <div className={classes['panel-camera-root']}>
            <div className={classes['camera-btn-container']}>
                <Popover
                    placement='leftTop'
                    content={<LayerMenu setShowBoundingBox={setShowBoundingBox} />}
                    trigger='click'
                >
                    <span className={classes['camera-btn-item']}>
                        <IconPark name='IcCoverageHover' />
                    </span>
                </Popover>
            </div>
            <CustomScroll className={classes['camera-canvas-container']}>
                <canvas ref={canvasRef} id={`camera-${panelId}`} className={classes['panel-camera-canvas']} />
            </CustomScroll>
        </div>
    );
}

function CameraView(props: any) {
    const Component = useMemo(
        () =>
            Panel({
                PanelComponent: InternalCameraView,
                panelId: props.panelId,
                subscribeInfo: [{ name: StreamDataNames.CAMERA, needChannel: true }],
            }),
        [],
    );
    return <Component {...props} />;
}

InternalCameraView.displayName = 'InternalCameraView';

export default React.memo(CameraView);
