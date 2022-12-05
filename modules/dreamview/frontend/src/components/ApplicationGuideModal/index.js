import classnames from 'classnames';
import React from 'react';

import './style.scss';

class ApplicationGuideModal extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      visible: false,
      classes: '',
    };
    this.enterAnimate = this.enterAnimate.bind(this);
    this.leaveAnimate = this.leaveAnimate.bind(this);
  }

  componentDidMount() {
    // Exclude the guide in the cloud experiment
    if (window.location.href.indexOf('practical') === -1) {
      this.enterAnimate();
    }
  }

  // 进入动画
  enterAnimate() {
    const enterClasses = 'modal-enter';
    const enterActiveClasses = 'modal-enter-active';
    const enterEndActiveClasses = 'modal-enter-end';
    const enterTimeout = 0;
    const enterActiveTimeout = 200;
    const enterEndTimeout = 100;
    this.setState({ visible: true, classes: enterClasses });
    const enterActiveTimer = setTimeout((_) => {
      this.setState({ classes: enterActiveClasses });
      clearTimeout(enterActiveTimer);
    }, enterTimeout);
    const enterEndTimer = setTimeout((_) => {
      this.setState({ classes: enterEndActiveClasses });
      clearTimeout(enterEndTimer);
    }, enterTimeout + enterActiveTimeout);

    const initTimer = setTimeout((_) => {
      this.setState({ classes: '' });
      clearTimeout(initTimer);
    }, enterTimeout + enterActiveTimeout + enterEndTimeout);
  }

  // 离开动画
  leaveAnimate() {
    const leaveClasses = 'modal-leave';
    const leaveActiveClasses = 'modal-leave-active';
    const leaveEndActiveClasses = 'modal-leave-end';
    const leaveTimeout = 0;
    const leaveActiveTimeout = 100;
    const leaveEndTimeout = 200;

    this.setState({ classes: leaveClasses });
    const leaveActiveTimer = setTimeout(() => {
      this.setState({ classes: leaveActiveClasses });
      clearTimeout(leaveActiveTimer);
    }, leaveTimeout);
    const leaveEndTimer = setTimeout(() => {
      this.setState({ classes: leaveEndActiveClasses });
      clearTimeout(leaveEndTimer);
    }, leaveTimeout + leaveActiveTimeout);

    const initTimer = setTimeout(() => {
      this.setState({ visible: false, classes: '' });
      clearTimeout(initTimer);
    }, leaveTimeout + leaveActiveTimeout + leaveEndTimeout);
  }

  render() {
    const { visible, classes } = this.state;

    if (!visible) {
      return null;
    }
    return (
      <div className="application-guide-modal">
        <div className="application-guide-modal-mask" />
        <div
          className={classnames(['application-guide-modal-content', classes])}
        >
          <div className="application-guide-modal-content-header-title">
            自动驾驶系统使用须知
          </div>
          <div className="application-guide-modal-content-body">
            <p>
              本使用须知旨在帮助车辆操控者理解自动驾驶系统可以安全操控和运行的使用范围，通过仔细阅读本须知，车辆操控者可以清晰地了解如下使用条件和注意事项，以便更安全的使用自动驾驶车辆。
            </p>
            {/* <ul>
              <li>自动驾驶模式下安全运行需满足的环境条件</li>
              <li>自动驾驶模式下安全运行需满足的车辆状态</li>
              <li>车辆安全操作注意事项</li>
            </ul> */}
            <p className="font-bold font-EB">
              {/* eslint-disable-next-line max-len */}
              警告：D-KIT套件属于测试、实验性质的产品，主要用于教学，并非商业化运营产品（如robotaxi）。切勿依靠D-KIT套件的自动驾驶系统来保障人身安全。车辆使用者及安全员有责任时刻保持警觉，安全使用，并掌控车辆。
            </p>
            {/* <h6>一、自动驾驶模式下安全运行需满足的环境条件</h6>
            <p>
              D-KIT开发套件不能在完全脱离人员监管的情况下使用，请在满足如下外部环境条件时启动自动驾驶模式，需要您严格遵守如下条件使用自动驾驶车辆，否则可能造成人员伤害或财务损失。
            </p> */}
            <h6>一、车辆状态检查</h6>
            <ul>
              <li>
                {/* eslint-disable-next-line max-len */}
                使用车辆前，请检查车胎是否损坏、充气压力（正常胎压2.5-2.6kpa）是否合适以及胎纹内是否嵌入异物，请不要在胎压过低甚至轮胎漏气的情况下启动车辆进行任何作业，以免产生不必要的危险。
              </li>
              <li>使用车辆前，检查车辆底部是否有泄露液体或易燃物。</li>
              <li>车辆上电后，请检查上电开关接通时各项指示灯的工作状况。</li>
              <li>
                车辆上电后，请确认动力电池电量（ SOC 值），若电池电量低于 20%
                ，建议充满电后在使用车辆，否则可能产生未知风险。
              </li>
              <li>
                遥控车辆及自动驾驶行驶前需确认控制遥控器及紧急遥控器可正常工作。
              </li>
              <li>请勿牵引其它车辆。</li>
            </ul>
            <h6>二、人员配备要求</h6>
            <p>
              D-KIT开发套件不能在完全脱离人员监管的情况下使用，在演示车辆自动驾驶能力时，确保
              <span className="font-bold">至少 2 人</span>配合操作：
            </p>
            <ul>
              <li>1 人操作自动驾驶软件</li>
              <li>1 人作为安全员操作遥控器，做好随时接管或紧急制动准备</li>
            </ul>
            <h6>三、车辆使用环境要求</h6>
            <p>
              请在满足如下外部环境条件时启动自动驾驶模式，需要您严格遵守如下条件使用自动驾驶车辆，否则可能造成较为严重的人员伤害或财物损失。
            </p>
            <table>
              <thead>
                <tr>
                  <th>环境条件</th>
                  <th>使用要求</th>
                </tr>
              </thead>
              <tbody>
                {/* <tr>
                  <td className='center'>
                    人员配备
                  </td>
                  <td className='pl-40'>
                    需配备安全员，做好随时接管准备
                  </td>
                </tr> */}
                <tr>
                  <td className="center">运行区域要求</td>
                  <td>
                    <ul>
                      <li>
                        确保在封闭区域运行此产品，请勿在公共道路、机动车道或高速公路行驶
                      </li>
                      <li className="font-bold">
                        确保可以对封闭区域内的其它交通参与者的数量及行为进行管制
                      </li>
                    </ul>
                  </td>
                </tr>
                <tr>
                  <td className="center">道路要求</td>
                  <td>
                    <ul>
                      <li>
                        确保在水平（坡度小于1%）、干燥、具有良好附着力的混凝土或沥青路面上行驶
                      </li>
                      <li>
                        确保在宽度不低于3.2m，最大曲率半径不低于1/0.172
                        m的道路上行驶
                      </li>
                    </ul>
                  </td>
                </tr>
                <tr>
                  <td className="center">交通参与者</td>
                  <td>
                    <ul>
                      <li>
                        请勿在道路上出现动物、垃圾、建筑设备等情况下使用车辆
                      </li>
                      <li className="font-bold">
                        确保周围的行人、机动车、非机动车的数量和行为可控，避免出现突然横穿马路，快速奔跑，试探自动驾驶车辆等危险行为
                      </li>
                    </ul>
                  </td>
                </tr>
                <tr>
                  <td className="center">交通元素</td>
                  <td className="font-bold">
                    D-KIT套件自动驾驶系统不支持识别任何交通元素（不支持红绿灯、人行道、停止线等）
                  </td>
                </tr>
                <tr>
                  <td className="center">特殊障碍物</td>
                  <td>
                    <p className="font-bold">
                      由于车辆感知系统存在盲区，使用区域请勿出现如下障碍物：
                    </p>
                    <ul>
                      <li>无低矮障碍物（在车辆前方1m内的30cm以下障碍物）</li>
                      <li>无悬空障碍物（悬空细长型或镂空形的障碍物）</li>
                      <li>无悬空离地2m以下障碍物</li>
                    </ul>
                  </td>
                </tr>
                <tr>
                  <td className="center">温度</td>
                  <td className="font-bold">
                    确保D-KIT套件在-10℃～50℃ 的温度范围内使用
                  </td>
                </tr>
                <tr>
                  <td className="center">天气</td>
                  <td>
                    <p>确保在满足如下天气条件下使用车辆：</p>
                    <ul>
                      <li>无降水（不能有雨、雨夹雪、雪等）</li>
                      <li>风速＜7.9m/s</li>
                      <li>能见度＞500m</li>
                    </ul>
                  </td>
                </tr>
                <tr>
                  <td className="center">光照</td>
                  <td>
                    建议在白天（光照度≥2000 lux）操作车辆，避免在傍晚、夜晚行车
                  </td>
                </tr>
                <tr>
                  <td className="center">驾驶操作限制</td>
                  <td>自动驾驶模式下，车辆出厂默认最大车速为1 km/h</td>
                </tr>
                <tr>
                  <td className="center">通信信号</td>
                  <td>
                    <ul>
                      <li>开机启动区域内卫星定位信号良好；</li>
                      <li>
                        4G/5G通信信号稳定良好（网络延时低于150ms，上传文件速度&gt;1MB/s，不能频繁断网）
                      </li>
                    </ul>
                  </td>
                </tr>
              </tbody>
            </table>
            {/* <h6>二、自动驾驶模式下安全运行需满足的车辆状态</h6> */}
            {/* <p> */}
            {/* eslint-disable-next-line max-len */}
            {/* 自动驾驶系统开启后，系统会实时监控车辆自身状态，以识别车辆状态是否能够支撑自动驾驶系统正常运行的因素；当出现如下表所列事件时，车辆会退出自动驾驶模式、或是不能进入自动驾驶模式直至触发事件解除。
            </p> */}
            {/* <table>
              <thead>
                <tr>
                  <th>触发事件类型</th>
                  <th>事件名称</th>
                  <th>说明</th>
                  <th>备注</th>
                </tr>
              </thead>
              <tbody>
                <tr>
                  <td rowSpan={3}>
                    主动触发
                  </td>
                  <td>遥控器接管开关置于遥控驾驶状态</td>
                  <td>车辆退出自动驾驶模式，进入遥控模式</td>
                  <td>接管后车辆操作权归还遥控器接管人员</td>
                </tr>
                <tr>
                  <td>车身急停开关按下</td>
                  <td>车辆退出自动驾驶模式，进入紧急停车模式，刹车制动</td>
                  <td>遥控不可接管车辆，操作车辆无响应,需通过车身急停开关解除急停模式</td>
                </tr>
                <tr>
                  <td>遥控器急停开关按下</td>
                  <td>车辆退出自动驾驶模式，进入紧急停车模式，刹车制动</td>
                  <td>通过遥控器可解除急停模式</td>
                </tr>
                <tr>
                  <td rowSpan={4}>
                    主动触发
                  </td>
                  <td>车辆碰撞致前/后防撞条触发</td>
                  <td>车辆退出自动驾驶模式，急刹停车</td>
                  <td>解除碰撞后，遥控可接管车辆</td>
                </tr>
                <tr>
                  <td>车辆转向系统故障</td>
                  <td>车辆退出自动驾驶模式</td>
                  <td>通过执行cyber_monitor，可查看chasiss_details中的车辆故障信息</td>
                </tr>
                <tr>
                  <td>车辆驱动系统故障</td>
                  <td>车辆退出自动驾驶模式</td>
                  <td>通过执行cyber_monitor，可查看chasiss_details中的车辆故障信息</td>
                </tr>
                <tr>
                  <td>车辆制动系统故障</td>
                  <td>车辆退出自动驾驶模式</td>
                  <td>通过执行cyber_monitor，可查看chasiss_details中的车辆故障信息</td>
                </tr>
              </tbody>
            </table>
            <p>注：”主动触发类型“是指安全员主动接管或者拍下急停开关等事件类型；”被动触发类型“是指车辆因系统故障或防撞条收到挤压等事件类型</p> */}
            <h6>四、车辆安全操作要求</h6>
            <ul>
              <li>
                在使用D-KIT套件前请详细阅读《车辆使用说明》或参加相关培训，严禁将车辆交予未经操作培训的人员独自操作使用。
              </li>
              <li>
                车辆不能在完全脱离人员监管的情况下使用，禁止在超出本产品使用范围的环境条件下使用本产品。
              </li>
              <li>
                安全起见，遥控驾驶时建议车速在10km/h以下，并时刻注意危险情况，随时做好紧急制动准备，正常测试时控制遥控器和紧急遥控器需有专人负责。
              </li>
              <li>
                车辆正常运行的过程中禁止任何人员进入车辆行驶前方15米内区域。
                <ul>
                  <li>
                    人工驾驶模式下操作者请时刻保持车辆处于视线范围以内，
                    同时密切关注车辆运行环境变化，及时提醒过往行人注意避让车辆或者停车让行。
                  </li>
                  <li>
                    车辆处于自动驾驶模式时请务必提前确认运行路线畅通，及时排除有可能导致安全事故以及导致车辆异常的因素。
                  </li>
                </ul>
              </li>
              <li>
                如果车辆出现任何异常问题，如动力系统、制动系统、转向系统等出现故障，必须立即停止使用车辆，严禁车辆带故障运行，以保证人员和财产不至于受到威胁。
              </li>
              {/* <li>
                安全起见，遥控驾驶时建议车速在10km/h以下，并时刻注意危险情况，随时做好紧急制动准备，正常测试时控制遥控器和紧急遥控器需有专人负责。
              </li>

              <li>
                如果车辆出现任何异常问题，如制动失效、转向失效等故障必须立即停止车辆使用，严禁车辆带故障运行，以保证人员和财产不至于受到威胁。
              </li>
              <li>
                每月至少2次检查轮胎气压，如有必要，应按时充气，这将显著延长轮胎使用寿命，并让车辆保持安全，车辆正常胎压2.5-2.6kpa。
              </li>
              <li>请勿在坡度大于1%的区域使用和停放车辆</li>
              <li>请勿牵引其它车辆。</li> */}
            </ul>
          </div>
          <div
            className="application-guide-modal-btn"
            onClick={() => this.leaveAnimate()}
          >
            我知道了
          </div>
        </div>
      </div>
    );
  }
}

export default ApplicationGuideModal;
