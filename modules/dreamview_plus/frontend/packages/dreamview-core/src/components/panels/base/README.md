
# Panel

## 摘要
Panel组件是Dreamview中所有面板的容器，为各个面板提供了数据订阅（有Channel订阅与无Channel订阅），添加订阅Channel，更新订阅Channel，取消订阅，面板全屏，快捷键注册，面板大小监听以及其他相关信息。

## 数据订阅
面板数据订阅包含面板初始化订阅，添加或更新Channel订阅以及取消订阅。

### 面板初始化订阅
面板初始化订阅相关代码如下：
```TypeScript
useEffect(() => {
    if (!isMainConnected || metadata.length <= 0) return noop;
    if (subscribeInfo) {
        const len = subscribeInfo?.length;
        // const obs: CountedSubject<unknown>[] = [];
        for (let i = 0; i < len; i += 1) {
            const curSubcibe = subscribeInfo[i];
            const name = curSubcibe?.name;
            const curItem = metadata.find((item) => item.dataName === name);
            if (!curItem) {
                // eslint-disable-next-line no-continue
                continue;
            }

            const connectedSubj: CountedSubject<unknown> = handleSubscribe(curSubcibe);
            // 2.若包含channel => subcribeToDataWithChannel => obs.push
            if (connectedSubj) {
                connectedSubjectsRef.current.push({
                    name,
                    connectedSubject: connectedSubj,
                });
            }
        }
        if (!hasSubscribed) {
            setHasSubscribed(true);
        }

        // eslint-disable-next-line no-restricted-syntax
        for (const subject of connectedSubjectsRef.current) {
            const subscription = subject.connectedSubject.subscribe((val) => {
                checkDataEmpty(val);

                if (initSubscriptionMapRef.current && !isEmpty(initSubscriptionMapRef.current)) {
                    const consumer = initSubscriptionMapRef.current[subject.name].consumer ?? defaultConsumer;
                    consumer(val);
                }
            });

            subscriptionsRef.current.push({
                name: subject.name,
                subscription,
            });
        }
    }

    return () => {
        connectedSubjectsRef.current = [];
        subscriptionsRef.current.forEach((subscription) => {
            subscription.subscription.unsubscribe();
        });
    };
}, [metadata, isMainConnected]);
```

该useEffect作用是处理面板初始化传入的订阅信息。主要过程如下：
1. 遍历订阅信息subscribeInfo，过滤掉metadata中不存在相关name的订阅信息
2. 调用handleSubscribe来处理数据订阅，handleSubscribe可以处理不同的订阅类型：无Channel订阅，有明确Channel名称的订阅，Channel为default的订阅
3. 将handleSubscribe处理得到的CountedSubject存入connectedSubjectsRef
4. 遍历connectedSubjectsRef，为每个CountedSubject声明面板提供的对应的订阅函数，完成初始化数据订阅

### 添加订阅Channel & 更新订阅Channel

添加Channel和更新Channel的主要逻辑都在addChannel函数中：
```TypeScript
const addChannel = useCallback(
    (newChannelInfo: SubscribeInfo) => {
        if (isMainConnected) {
            const curSubjects = connectedSubjectsRef.current;
            const targetSubjectIndex = curSubjects.findIndex(
                (curSubject) => curSubject.name === newChannelInfo.name,
            );
            subscriptionsRef.current.forEach((subscription) => {
                subscription.subscription.unsubscribe();
            });

            if (targetSubjectIndex === -1) {
                // 不存在 => 新增订阅
                const newConnectedSubj: CountedSubject<unknown> = handleSubscribe(newChannelInfo);
                if (newConnectedSubj) {
                    const subscription = newConnectedSubj.subscribe((val) => {
                        checkDataEmpty(val);

                        if (initSubscriptionMapRef.current && !isEmpty(initSubscriptionMapRef.current)) {
                            const consumer =
                                initSubscriptionMapRef.current[newChannelInfo.name]?.consumer ??
                                defaultConsumer;
                            consumer(val);
                        }
                    });

                    curSubjects.push({
                        name: newChannelInfo.name,
                        connectedSubject: newConnectedSubj,
                    });

                    subscriptionsRef.current.push({
                        name: newChannelInfo.name,
                        subscription,
                    });
                }
            } else {
                // 存在 => 重新订阅
                const subscriptionIndex = subscriptionsRef.current.findIndex(
                    (subscription) => subscription.name === newChannelInfo.name,
                );
                const curSubscription = subscriptionsRef.current[subscriptionIndex];

                curSubscription.subscription.unsubscribe();

                const newConnectedSubj: CountedSubject<unknown> = handleSubscribe(newChannelInfo);
                if (newConnectedSubj) {
                    const newSubscription = newConnectedSubj.subscribe((val) => {
                        checkDataEmpty(val);

                        if (initSubscriptionMapRef.current && !isEmpty(initSubscriptionMapRef.current)) {
                            const consumer =
                                initSubscriptionMapRef.current[newChannelInfo.name]?.consumer ??
                                defaultConsumer;
                            consumer(val);
                        }
                    });

                    // 2.2替换掉connectedSubjectsRef.current数组中对应的subject
                    curSubjects[targetSubjectIndex] = {
                        name: newChannelInfo.name,
                        connectedSubject: newConnectedSubj,
                    };

                    subscriptionsRef.current[subscriptionIndex] = {
                        name: newChannelInfo.name,
                        subscription: newSubscription,
                    };
                }
            }
        }
    },
    [isMainConnected],
);
```

addChannel可以处理添加和更新Channel信息，主要流程如下：
1. 遍历connectedSubjectsRef查找当前新Channel对应name是否已经订阅
2. 若未订则调用handleSubscribe来新增订阅，若已订阅则重新订阅

addChannel在Panel中有两处使用：
```TypeScript
useNotifyInitialChanel(panelId, addChannel);
useUpdateChannel(panelId, addChannel);
```
useNotifyInitialChanel是用于ChannelSlector（Channel下拉框选择器）初始选择默认Channel
useUpdateChannel是用于更新Channel

### 取消订阅
取消订阅的逻辑在closeSubcription中：

```TypeScript
const closeSubcription = useCallback(
    (name: string) => {
        const targetSubjectIndex = connectedSubjectsRef.current.findIndex(
            (curSubject) => curSubject.name === name,
        );

        if (targetSubjectIndex !== -1) {
            const subscriptionIndex = subscriptionsRef.current.findIndex(
                (subscription) => subscription.name === name,
            );
            const curSubscription = subscriptionsRef.current[subscriptionIndex];

            curSubscription.subscription.unsubscribe();

            subscriptionsRef.current = subscriptionsRef.current.filter(
                (subscriptionInfo) => subscriptionInfo.name !== name,
            );
            connectedSubjectsRef.current = connectedSubjectsRef.current.filter(
                (connectedSubjectInfo) => connectedSubjectInfo.name !== name,
            );
        }
    },
    [isMainConnected],
);
```
取消订阅逻辑如下：
1. 在connectedSubjectsRef查找要取消订阅的name，若存在则进行取消操作
2. 找到对应subscription，执行curSubscription.subscription.unsubscribe();
3. 更新subscriptionsRef（保存已订阅的subscription[]）与connectedSubjectsRef（保存已订阅的Subject[]）

## 面板大小监听

面板全屏功能是基于react-resize-detector完成的，面板开发者可使用onPanelResize来监听面板的宽高：
```TypeScript
const { ref: panelRootRef } = useResizeDetector({
    onResize: (width: number, height: number) => {
        panelWidthRef.current = width;
        if (resizeCallBackRef.current) {
            resizeCallBackRef.current(width, height);
        }
        resizeCallBackArrRef.current.forEach((cb) => {
            try {
                cb(width, height);
            } catch (err) {
                console.log(err);
            }
        });
    },
});

const onPanelResize = useCallback((onResize: OnResizeCallback) => {
    resizeCallBackRef.current = onResize;
    resizeCallBackArrRef.current.push(onResize);
    if (panelWidthRef.current !== -1) {
        onResize(panelWidthRef.current);
    }
}, []);
```
其中panelRootRef为PanelRoot组件的ref

## 面板全屏

面板全屏逻辑的代码位于modules/dreamview_plus/frontend/packages/dreamview-core/src/components/panels/base/RenderTile/index.tsx
进入全屏的相关代码如下：

```TypeScript
const enterFullScreen = useCallback(async () => {
    if (inFullScreen) return;

    const fullScreenHookConfig = hooksManager.getHook(panelId);
    let hookBeforeResBoolean = false;

    if (fullScreenHookConfig?.beforeEnterFullScreen) {
        hookBeforeResBoolean = await hooksManager.handleFullScreenBeforeHook(
            fullScreenHookConfig.beforeEnterFullScreen,
        );
    }

    const isExecutable = !fullScreenHookConfig?.beforeEnterFullScreen || hookBeforeResBoolean;

    if (isExecutable) {
        _setFullScreen(true);

        if (fullScreenHookConfig?.afterEnterFullScreen) {
            fullScreenHookConfig?.afterEnterFullScreen();
        }
    }
}, [inFullScreen, panelId]);
```

进入和退出全屏主要通过调用_setFullScreen更新全屏状态。
此外，还支持进入和退出全屏的钩子函数的配置。主要通过hooksManager来管理不同面板的进入和退出全屏钩子函数。
在Panel中通过使用registerFullScreenHooks来注册相关钩子函数：

```TypeScript
const registerFullScreenHooks = useCallback((hookConfig: FullScreenHookConfig) => {
    hooksManager.addHook(panelId, hookConfig);
}, []);
```

通过<PanelTileProvider />传入enterFullScreen与exitFullScreen：

```TypeScript
<PanelTileProvider
    path={path}
    enterFullScreen={enterFullScreen}
    exitFullScreen={exitFullScreen}
    fullScreenFnObj={fullScreenFnRef.current}
>
    ...
</PanelTileProvider>
```

在Panel中通过usePanelTileContext获取：
```TypeScript
  const { enterFullScreen, exitFullScreen, fullScreenFnObj } = usePanelTileContext();
```

## 快捷键注册

面板的快捷键注册相关接口主要有setKeyDownHandlers与removeKeyDownHandlers。
setKeyDownHandlers的作用是保存当前Panel的快捷键状态与store中的状态：

```TypeScript
const setKeyDownHandlers = useCallback((handlers: KeyHandlers[]) => {
    _setKeyDownHandlers((prevHandlers) => [...prevHandlers, ...handlers]);
    // save

    // eslint-disable-next-line no-restricted-syntax
    for (const handler of handlers) {
        if (handler?.isGloable) {
            panelInfoDispatch(addGloableKeyHandler([handler]));
        } else {
            panelInfoDispatch(addKeyHandler({ panelId, keyHandlers: [handler] }));
        }
    }
}, []);
```

快捷键注册实现逻辑主要在<KeyListener />组件中，快捷键注册主要依赖于rxjs的fromEvent实现：
```TypeScript
useEffect(() => {
    if (keyDownHandlers) {
        const len = keyDownHandlers.length;
        for (let i = 0; i < len; i += 1) {
            const keyDownEvent = fromEvent(document, 'keydown');
            const setKeyDown = getMultiPressedKey(keyDownEvent);
            const keyDownHandler = keyDownHandlers[i];
            setKeyDown(
                (event) => {
                    if (activeRef.current) {
                        keyDownHandler?.handler(event as KeyboardEvent);
                    }
                },
                keyDownHandler?.keys,
                keyDownHandler?.functionalKey,
            );
        }
    }

    return () => {
        // 想要清空keyDownEventRef.current注册的所有回调
        for (const sp of subscriptionsRef.current) {
            sp.unsubscribe();
        }

        subscriptionsRef.current = [];
    };
}, [keyDownHandlers]);
```
上述代码主要逻辑如下：
1. 通过fromEvent拿到keydown事件
2. 通过getMultiPressedKey获得一个可以配置快捷键事件回调的setKeyDown
3. 使用setKeyDown配置相关快捷键与对应回调函数（当前面板选中即activeRef.current时回调逻辑才会执行）
4. getMultiPressedKey的作用类似工厂函数，用于生产各事件的快捷键配置函数

## 面板选中

面板选中/激活依赖 <PanelRoot id={panelId} style={style} onClick={onPanleClick} ref={panelRootRef} /> 的onClick事件，处理逻辑如下：

```TypeScript
const onPanleClick: React.MouseEventHandler<HTMLDivElement> = useCallback(() => {
    if (!isSelected) {
        panelInfoDispatch(addSelectedPanelId(panelId));
    }
}, [isSelected, panelInfoDispatch]);
```

更新面板激活态通过更新PanelInfoStore中的selectedPanelId实现。
UI逻辑位置：modules/dreamview_plus/frontend/packages/dreamview-core/src/components/panels/base/RenderTile/index.tsx

```TypeScript
const isSelected = useMemo(() => panelInfoState?.selectedPanelIds?.has(panelId) ?? false, [panelInfoState]);

...

const finalCls = `${cx(classes['panel-item-container'], {
    [classes.dragging]: dragging,
})} ${classNames('panel-container', {
    'panel-selected': isSelected,
})}`;
```

## Channel选择器

Channel选择器为Panel提供了切换Channel的功能。
主要逻辑位于：modules/dreamview_plus/frontend/packages/dreamview-core/src/components/panels/base/ChannelSelect/demo.tsx

```TypeScript
function DemoChannelSelect(props: RenderToolbarProps) {
    
    ...

    const channels = useMemo(() => {
        if (!curMeta) {
            return [];
        }

        return curMeta.channels.map((channel) => ({
            label: channel.channelName,
            value: channel.channelName,
        }));
    }, [curMeta]);
    const notifyInitialChannel = useRegisterNotifyInitialChanel(panelId);

    const onChange = (value, option) => {
        setCurVal(value);
        updateChannel({
            name: props.name,
            channel: value,
            needChannel: true,
        });
        localStorage.setItem(`${panelId}-selected-cn`, value);
    };

    useEffect(() => {
        if (channels.length > 0) {
            const cacheChannel = localStorage.getItem(`${panelId}-selected-cn`);
            let cn = channels[0]?.value;
            for (const channel of channels) {
                if (cacheChannel === channel.value) {
                    cn = cacheChannel;
                    break;
                }
            }
            setCurVal(cn);
            notifyInitialChannel({
                name: curMeta.dataName,
                channel: cn,
                needChannel: true,
            });
        } else {
            setCurVal(undefined);
        }
    }, [channels]);

    return <ChannelSelect value={curVal} options={channels} onChange={onChange} />;
}
```

主要逻辑为每次切换时缓存，并调用updateChannel发布更新Channel的事件，通知Pannel更新Channel。
当首次加载时，优先取缓存Channel其次取channels数组中第一个作为默认值。
