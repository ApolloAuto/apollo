## @dreamview/dreamview-theme
##### 1、dreamview-theme是什么
dreamview-theme是dreamview主题样式变量的全集，赋予每一个变量一种语义，在开发过程中避免直接使用css样式，以达到统一管理样式的目的
##### 2、Usage
```
import {Provider as ThemeProvider, makeStyles} from '@dreamview/dreamview-theme';

const useStyle = makeStyles((theme) => {
    root: {
        backgroundColor: theme.tokens.backgroundColor.main,
        color: theme.tokens.font.color.main,
    },
});

function ChildComponents() {
    const {classes} = useStyle();
    
    return (<div className={classes.root}>root</div>)
}

function App(){
    return (
        <ThemeProvider>
            <ChildComponents />
        </ThemeProvider>    
    )
}
```
##### 3、自定义主题
通过替换tokens的变量的值达到替换主题的效果
```
import {Provider as ThemeProvider, makeStyles} from '@dreamview/dreamview-theme';

const useStyle = makeStyles((theme) => {
    root: {
        backgroundColor: theme.tokens.backgroundColor.main,
        color: theme.tokens.font.color.main,
    },
})
function ChildComponents() {
    const {classes} = useStyle();

    return (<div className={classes.root}>root</div>)
}

function App(){
    const customTheme = {
        tokens: {
            backgroundColor: {
                main: 'blue'            
            }        
        }    
    }
    return (
        <ThemeProvider config={customTheme}>
            <ChildComponents />
        </ThemeProvider>    
    )
}
```
##### 4、使用components，dremview-colors中提供了一部分定义好的样式的合集，包含了组件样式和部分功能样式，正在完善中，我们称为components
```
import {Provider as ThemeProvider, makeStyles} from '@dreamview/dreamview-theme';

// 这里以button组件样式为例
const useStyle = makeStyles((theme) => {
    root: theme.components.button.primary,
})

function ChildComponents() {
    const {classes} = useStyle();
    
    return (<button className={classes.root}>root</button>)
}

function App(){
    return (
        <ThemeProvider>
            <ChildComponents />
        </ThemeProvider>    
    )
}
```
##### 5、开发
```
├── README.md
├── createStyles.tsx // 创建provider和context
├── main.ts // 入口文件
├── scripts
│   └── generateType.js // 生成类型定义的脚本
├── tokens // 变量合集
│   ├── baseToken // 基础变量
│   │   ├── allColors.js // 主题色的全集
│   │   ├── colors.ts // 把每个主题色衍生出10个与之颜色相近的颜色
│   │   ├── index.ts // 基础变量
│   │   └── type.ts // 
│   └── components // components
│       ├── button.ts // button组件的样式
│       └── index.ts
```
- 增加主题色
	1. 在allColors.js中增加主题色
	2. 运行命令生成类型文件yarn lerna run generate:types --scope=@dreamview/dreamview-theme
- 修改/增加/删除基础变量
	1. 修改tokens/baseToke/index.ts文件即可
- 增加一组样式的合集
	1. 在tokens/components下创建文件夹编写自己的样式合集
	2. 在tokens/components/index.ts中导出你创建的文件

