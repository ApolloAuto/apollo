import { observable, action, computed } from 'mobx';

export default class StudioConnector {

  // 本地场景集数据
  @observable scenarioSet = [];

  // 当前选中场景集id
  @observable currentScenarioSetId = 'none';

  // 当前选中场景集Id
  @observable currentScenarioId = 'none';

  // 云端场景集数据
  @observable remoteScenarioSetList = [];

  // certificate_status = "expired" | "notFound" | "normal"
  //                              过期 | 未找到 | 正常
  @observable certificateStatus = 'normal';

  @observable remoteScenarioSelectConditionValue = 'All';

  /**
   * 同步证书状态
   * @param status
   * 'OK'
   * | 'CLIENT_CRT_NOT_FOUND' | 'CLIENT_KEY_NOT_FOUND' | 'AUTH_ID_NOT_FOUND'
   * | 'CLIENT_CRT_EXPIRED' | 'SERVER_CRT_EXPIRED'
   */
  @action checkCertificate(status) {
    switch (status) {
      case 'OK':
        this.certificateStatus = 'normal';
        break;
      case 'CLIENT_CRT_NOT_FOUND':
      case 'CLIENT_KEY_NOT_FOUND':
      case 'AUTH_ID_NOT_FOUND':
        this.certificateStatus = 'notFound';
        break;
      case 'CLIENT_CRT_EXPIRED':
      case 'SERVER_CRT_EXPIRED':
        this.certificateStatus = 'expired';
        break;
      default:
        this.certificateStatus = 'noFound';
    }
  }

  /**
   * 更新云端场景集数据
   * @param scenarioSetObjects {
   * {[scenarioId: string]: {name: string, status: string}}
   * | {error_msg: string}
   * }
   */
  @action updateRemoteScenarioSetList(scenarioSetObjects) {
    if (!scenarioSetObjects.error_msg) {
      this.remoteScenarioSetList = Object.keys(scenarioSetObjects)
        .map((scenarioSetId) => {
          const scenarioSet = scenarioSetObjects[scenarioSetId];
          return {
            scenarioSetId, name: scenarioSet.name, status: scenarioSet.status,
          };
        });
    }
  }

  /**
   * 更新本地场景集状态
   * @param scenarioInfo {{
   *  currentScenarioId: string,
   *  currentScenarioSetId: string,
   *  scenarioSet: {
   *    [scenarioId: string]: {
   *  scenarioSetName: string,
   *  scenarios: {
   *    mapName: string,
   *    scenarioId: string,
   *    scenarioName: string,
   *  }[]
   *  }}
   * }}
   */
  @action updateLocalScenarioInfo(scenarioInfo) {
    this.currentScenarioSetId = scenarioInfo.currentScenarioSetId;
    this.currentScenarioId = scenarioInfo.currentScenarioId;
    this.scenarioSet = Object.keys(scenarioInfo.scenarioSet).map((scenarioSetId) => {
      const scenarioSet = scenarioInfo.scenarioSet[scenarioSetId];
      return {
        scenarioSetId, name: scenarioSet.scenarioSetName,
        scenarios: scenarioSet.scenarios.map((scenario) => {
          return {
            mapName: scenario.mapName,
            scenarioId: scenario.scenarioId,
            scenarioName: scenario.scenarioName,
          };
        }),
      };
    });
  }

  @action updatingTheScenarioSetById(scenarioSetId) {
    this.remoteScenarioSetList = this.remoteScenarioSetList.map((item) => {
      if (item.scenarioSetId === scenarioSetId) {
        item.status = 'updating';
      }
      return item;
    });
  }

  /**
   *
   * @param value "All" | "notDownloaded" | "toBeUpdated" | "updating" | "downloaded" | "fail"
   */
  @action updateRemoteScenarioSelectConditionValue(value) {
    this.remoteScenarioSelectConditionValue = value;
  }

  @computed get remoteScenarioSetListFiltered() {
    const { remoteScenarioSelectConditionValue } = this;
    if (remoteScenarioSelectConditionValue === 'All') {
      return this.remoteScenarioSetList;
    }
    return this.remoteScenarioSetList.filter((item) => {
      return item.status.indexOf(remoteScenarioSelectConditionValue) > -1;
    });
  }

  /**
   * 更新远端场景集状态
   * @param scenarioSetId string
   * @param status "notDownloaded" | "toBeUpdated" | "updating" | "downloaded" | "fail"
   * @param errorMsg string
   */
  @action updateRemoteScenarioSetStatus(scenarioSetId, status, errorMsg) {
    if (scenarioSetId) {
      this.remoteScenarioSetList = this.remoteScenarioSetList.map((item) => {
        if (item.scenarioSetId === scenarioSetId) {
          item.status = status;
          item.errorMsg = errorMsg;
        }
        return item;
      });
    }
  }
}
