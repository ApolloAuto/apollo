/* eslint-disable */
/**
* @function :   插件的调用。
* @how2use  :  使用过程，可以复制过去，仅需修改调用按钮id即可；如需透传数据信息，请联系百度反馈接口人。
*/


/**
 * @function :第一次加载插件，之后调用插件
 */
function init_feedback() {
    if (bds && bds.qa && bds.qa.ShortCut && bds.qa.ShortCut.initRightBar) {
        // 初始化反馈插件的样式参数

        const fb_options = {
            needImage: true, // 是否需要用户展示图片(截图，或者上传)
            upload_file: true, // 是否选择上传图片， true 表示上传图片， false表示截图
            appid: 240308, // 产品线的id
            productLine: 90965, //
            wenjuanTitle: '', // 问卷调查标题
            wenjuanURL: '', // 问卷调查链接
            issuePlaceholder: '请首先阅读 常见问题： 【**提问前必读**】', // 问题描述提示语
            contactPlaceholder: '请输入邮箱地址，便于及时处理您的问题', // 联系方式提示语
            showPosition: 'center', // 显示位置，中心：center，右下： bottom_right
            contactWayType: 'email', // 联系方式类型 contact_way，email，qq，tel
            needContactWay: true, // 是否显示联系方式
            needHotQuestion: true, // 是否显示常见问题
            needQuestionnaire: false, // 是否显示问卷调查
            needFeedbackType: true, // 是否显示反馈类型
            needProductType: false, // 是否显示问题类型
            needEvaluate: true, // 是否启用用户评价
            typeArray: [{"id":"65469","name":"其他问题"}], // 下拉列表项
            titleBgColor: '#FFFFFF', // 标题栏背景色
            buttonColor: '#3582FA', // 按钮的颜色
            mainFontColor: '#151515', // 文字的主体颜色
            secondaryFontColor: '#999999', // 文字的辅助颜色(用于input textarea输入框的提示文本)
            titleColor: '#333333', // 标题栏颜色
            authType: 'ufo', // 环境
        };

        bds.qa.ShortCut.initRightBar(fb_options);
        // 产品线需要透传的参数，注意：json的标题必须是规范json写法，双引号，最后无逗号
        const pro_data = {
            // "extend_feedback_channel": 0  // 反馈来源
        };
        bds.qa.ShortCut._getProData(pro_data);
    }
}

/**
 * @function :校验js加载完成
 * @returns :{boolean}
 */
export default function popupFeedbackForm() {
    if (window.bds && window.bds.qa && window.bds.qa.ShortCut) {
        init_feedback();
    } else {
        loadPlugin(init_feedback);
    }
    return false;
}

function loadPlugin(callback: any) {
    loadScript('https://ufosdk.baidu.com/Public/feedback/js/dist/feedback_plugin_2.0.js',
        callback,
        {
            charset: 'utf-8',
            id: 'feedback_script',
        }
    );
}

function loadScript(url: any, callback: any, opt: any) {
    const script = document.createElement('script') as any;
    var opt = opt || {};
    script.type = 'text/javascript';
    if (opt.charset) {
        script.charset = opt.charset;
    }
    if (opt.id) {
        script.id = opt.id;
    }

    if (script.readyState) {
        script.onreadystatechange = function () {
            if (script.readyState == 'loaded' || script.readyState == 'complete') {
                script.onreadystatechange = null;
                callback();
            }
        };
    } else {
        script.onload = function () {
            callback();
        };
    }
    script.src = url;
    document.body.appendChild(script);
}

loadPlugin(() => console.log('loaded baidu feedback plugin'));
