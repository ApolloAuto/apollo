# Store组件文档

## 简介

Store组件是我们DreamviewPlus客户端的核心部分，负责管理和维护应用的状态。它采用了现代的前端架构理念，结合响应式编程和不可变状态管理，为开发者提供了一个高效、可靠的状态管理解决方案。

## 组件结构

- **Reducer.ts** - 定义了处理状态变更的Reducer函数和相关的Action接口。
- **Store.ts** - 实现了Store，管理应用的状态，并提供更新状态的接口。
- **Provider.ts** - 提供了一个React组件，用于将Store注入到React应用中。
- **Persistor.ts** - 负责状态的持久化，保证应用状态在会话间的持续性。
- **Middleware.ts** - 实现了中间件，用于在状态更新过程中执行额外的逻辑，如异步操作处理。
- **Factory.tsx** - 提供了用于创建和配置Store实例的工厂方法，特别适用于React环境。

## 功能描述

### Store.ts

- 使用rxjs的BehaviorSubject实现了响应式状态管理。
- 提供了状态订阅的能力，允许组件在状态变更时自动更新。
- 集成了Persistor和Middleware，增强了状态管理的功能。

### Reducer.ts

- 定义了`Action`和`PayloadAction`接口，为状态更新提供了标准化的操作指令。
- 每个Reducer函数根据接收到的action类型和payload来决定如何更新状态。

### Provider.ts

- 使用React的Context API将Store注入React组件。
- 使得React组件能够通过Context访问Store中的状态和dispatch方法。

### Persistor.ts

- 实现了状态的持久化逻辑。
- 通过LocalStorage等机制保证状态在浏览器会话之间的一致性。

### Middleware.ts

- 提供了一种机制，在状态更新的过程中插入自定义逻辑。
- 常用于处理如API请求等异步操作。

### Factory.tsx

- 提供了一个高度可配置的Store实例创建方式。
- 使得在React环境中集成Store更加灵活和方便。

## 使用方法

### Store的创建和配置

在`Factory.tsx`中，我们提供了创建Store实例的方法。开发者可以根据自己的需求配置Reducer、Middleware和Persistor。以下是创建Store实例的一个示例：

```javascript
import { createStoreProvider } from './Factory.tsx';
const { StoreProvider, useStore } = createStoreProvider<IInitState, TYPES.CombineAction>({
    initialState: initState,
    reducer,
});

const AppWithStore = () => {
    return (
        <StoreProvider>
            <ChildComponent />
        </StoreProvider>
    );
};
```

## 使用案例

### 状态更新

假设我们有一个用于用户信息同步的action。首先，在`action.ts`中定义处理这个action：

```typescript
import {PayloadAction} from '../base/Reducer';
import {INIT_USER_INFO} from './actionTypes';

export type IInitState = {
    userInfo: {
        avatar_url: string;
        displayname: string;
        id: string;
    };
    isLogin: boolean;
};

type InitUserInfoAction = PayloadAction<typeof INIT_USER_INFO, IInitState>;

export const initUserInfo = (payload: IInitState): InitUserInfoAction => ({
    type: INIT_USER_INFO,
    payload,
});

export type CombineAction = InitUserInfoAction;
```

通过工厂函数生成`StoreProvider`和`useStore`：

```typescript
import { Factory } from '../base';
import { IInitState, CombineAction } from './actions';
import { initState, reducer } from './reducer';

export const { StoreProvider: UserInfoStoreProvider, useStore: useUserInfoStore } = Factory.createStoreProvider<
    IInitState,
    CombineAction
>({
    initialState: initState,
    reducer,
});
```

然然后在组件中使用`useStore`获取状态和dispatch方法更新状态：

```typescript
 const [store, dispatch] = useUserInfoStore();

dispatch({
        userInfo: {
            avatar_url: undefined,
            displayname: undefined,
            id: undefined,
        },
        isLogin: true,
    },
);
```

### 异步操作

目前已经预置异步Middleware，我们可以定义异步action，比如请求接口后同步数据：

```typescript
export const changeMode = (
    mainApi: MainApi,
    payload: TYPES.ChangeModePayload,
    callback?: (mode: CURRENT_MODE) => void,
): AsyncAction<TYPES.IInitState, ChangeModeAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeMode', { state, payload });
        await mainApi.changeSetupMode(payload);
        if (callback) {
            callback(payload);
        }
    };
};
```

### 自定义中间件

框架支持可插拔的中间件定义，`[Middleware.ts](Middleware.ts)`已经预置了4类中间件，分别是：
- 异步action处理中间件
- 日志中间件
- 崩溃处理中间件
- 同步调试中间件

如何定义一个中间件呢？我们以异步action处理中间件为例，首先我们需要定义一个中间件函数：

```typescript
export function asyncActionMiddleware<S, A>(
    store: Store<S, A>,
    next: (action: A) => void,
    action: AsyncAction<S, A> | A,
): void {
    if (typeof action === 'function') {
        (action as AsyncAction<S, A>)(store.dispatch, store.getState());
    } else {
        next(action);
    }
}
```

然后再初始化Store的工厂函数[Factory.tsx](Factory.tsx)中，将中间件函数传入：
```typescript
  store.applyMiddleware(...(props.middleware || middleware), ...middlewareExtra);
```

### 状态持久化
为了方便快速恢复状态，我们提供了状态持久化的功能。在[Factory.tsx](Factory.tsx)中，我们可以通过传入`persistor`来配置持久化的方式：

```typescript
 store.persist(iPersistor);
```

持久化方法

在[Persistor.ts](Persistor.ts)中，我们提供了持久化方法创建的工厂函数，开发者可以根据自己的需求配置持久化的方式：

```typescript
 createLocalStoragePersistor('dv-panel-inner-stoer');
```



## 结论

本文档提供了Store组件的详细说明，包括其功能描述、代码结构、使用方法和具体案例。我们希望这份文档能够帮助开源开发者更好地理解和使用这个组件，为Dreamview Plus应用的开发和维护做出贡献。欢迎任何形式的反馈和建议。
