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

  // 类型筛选
  @observable typeConditionValue = 'All';

  // 状态筛选
  @observable statusConditionValue = 'All';

  // 云端动力学模型数据
  @observable remoteDynamicModelList = [];

  // 云端播包数据
  @observable remoteRecordList = [];

  // 车辆更新状态 0:未更新 1:更新中 2:更新成功 3:更新失败
  @observable vehicleUpdateStatus = 0;

  // 车辆信息列表
  // @observable vehicleInfoList = [
  //   {
  //     vehicle_id: '12',
  //     vin: 'CH0000030',
  //     vtype: '1',
  //   },
  //   {
  //     vehicle_id: '13',
  //     vin: 'CH0040000',
  //     vtype: '2',
  //   },
  //   {
  //     vehicle_id: '142',
  //     vin: 'CH0000003',
  //     vtype: '3',
  //   },
  //   {
  //     vehicle_id: '15',
  //     vin: 'CH0000020',
  //     vtype: '4',
  //   },
  //   {
  //     vehicle_id: '16',
  //     vin: 'CH0003000',
  //     vtype: '5',
  //   },
  //   {
  //     vehicle_id: '17',
  //     vin: 'CH0000011',
  //     vtype: '6',
  //   },
  //   {
  //     vehicle_id: '17',
  //     vin: 'CH0000011',
  //     vtype: '7',
  //   },
  // ];
  @observable vehicleInfoList = [];

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
            id: scenarioSetId, name: scenarioSet.name, status: scenarioSet.status, type: 1,
          };
        });
    }
  }

  /**
   * 更新云端动力学模型数据
   * @param dynamicModelObjects {
   *  {[name:string]: {name: string, status: string}}
   * | {error_msg: string}
   * }
   */
  @action updateRemoteDynamicsModelList(dynamicModelObjects) {
    if (!dynamicModelObjects.error_msg) {
      this.remoteDynamicModelList = Object.keys(dynamicModelObjects)
        .map((name) => {
          const dynamicModel = dynamicModelObjects[name];
          return {
            id: name, name: dynamicModel.name, status: dynamicModel.status, type: 2,
          };
        });
    }
  }

  /**
   * 更新云端数据包数据
   * @param recordObjects {
   *   data_list: {name: string, status: string}[],
   * | {error_msg: string}
   * }
   */
  @action updateRemoteRecordsList(recordObjects) {
    if (!recordObjects.error_msg) {
      this.remoteRecordList = Object.keys(recordObjects)
        .map((name) => {
          const record = recordObjects[name];
          return {
            id: name, name: record.name, status: record.status, type: 3,
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
      if (item.id === scenarioSetId) {
        item.status = 'updating';
      }
      return item;
    });
  }

  /**
   *
   * @param value "All" | "notDownloaded" | "toBeUpdated" | "updating" | "downloaded" | "fail"
   */
  @action updateStatusConditionValue(value) {
    this.statusConditionValue = value;
  }

  /**
   *
   * @param value 1|2|3 // 场景集|动力学模型|数据包
   */
  @action updateTypeConditionValue(value) {
    this.typeConditionValue = value;
  }

  // 云端场景集筛选数据
  @computed get remoteScenarioSetListFiltered() {
    const { typeConditionValue, statusConditionValue } = this;
    // 仅类型状态为1或者全部时候返回云端场景集数据
    if (typeConditionValue === 'All' || typeConditionValue === '1') {
      if (statusConditionValue === 'All') {
        return this.remoteScenarioSetList;
      }
      return this.remoteScenarioSetList.filter((item) => {
        return item.status.indexOf(statusConditionValue) > -1;
      });
    }
  }

  // 云端动力学模型筛选数据
  @computed get remoteDynamicModelListFiltered() {
    const { typeConditionValue, statusConditionValue } = this;
    // 仅类型状态为1或者全部时候返回云端场景集数据
    if (typeConditionValue === 'All' || typeConditionValue === '2') {
      if (statusConditionValue === 'All') {
        return this.remoteDynamicModelList;
      }
      return this.remoteDynamicModelList.filter((item) => {
        return item.status.indexOf(statusConditionValue) > -1;
      });
    }
  }

  // 云端数据包筛选数据
  @computed get remoteRecordListFiltered() {
    const { typeConditionValue, statusConditionValue } = this;
    // 仅类型状态为1或者全部时候返回云端场景集数据
    if (typeConditionValue === 'All' || typeConditionValue === '3') {
      if (statusConditionValue === 'All') {
        return this.remoteRecordList;
      }
      return this.remoteRecordList.filter((item) => {
        return item.status.indexOf(statusConditionValue) > -1;
      });
    }
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
        if (item.id === scenarioSetId) {
          item.status = status;
          item.errorMsg = errorMsg;
        }
        return item;
      });
    }
  }

  /**
   * 更新远端动力学模型状态
   * @param name string
   * @param status "notDownloaded" | "toBeUpdated" | "updating" | "downloaded" | "fail"
   * @param errorMsg string
   */
  @action updateRemoteDynamicsModelStatus(name, status, errorMsg) {
    if (name) {
      this.remoteDynamicModelList = this.remoteDynamicModelList.map((item) => {
        if (item.id === name) {
          item.status = status;
          item.errorMsg = errorMsg;
        }
        return item;
      });
    }
  }

  /**
   * 更新远端数据包状态
   * @param name string
   * @param status "notDownloaded" | "toBeUpdated" | "updating" | "downloaded" | "fail"
   * @param errorMsg string
   */
  @action updateRemoteRecordStatus(name, status, errorMsg) {
    if (name) {
      this.remoteRecordList = this.remoteRecordList.map((item) => {
        if (item.id === name) {
          item.status = status;
          item.errorMsg = errorMsg;
        }
        return item;
      });
    }
  }

  /**
   * 更新车辆信息
   * @param vehicleInfoList {
   * [vehicle_id: string]: {vehicle_id: string, vin: string, vtype: string}
   * }
   * @param vehicleUpdateStatus 0 | 1 | 2| 3 // 0: 未更新 1: 更新中 2: 更新成功 3: 更新失败
   */
  @action updateVehicleInfo(vehicleInfoListObj, vehicleUpdateStatus) {
    // 更新成功同步车辆信息
    if (vehicleUpdateStatus === 2) {
      const vehicleInfoList = Object.values(vehicleInfoListObj);
      this.vehicleInfoList = vehicleInfoList;
    }
    this.vehicleUpdateStatus = vehicleUpdateStatus;
  }
}
