#ifndef NAVI_AUTO_LOCATION_DEF_H
#define NAVI_AUTO_LOCATION_DEF_H


#include "navi_car_adapter_def.h"

NAMESPACE_BAIDU
NAMESPACE_BAIDU_ADAPTER

USING_NAMESPACE_ADAPTER

#define BNALocation_LOG

#define BNA_LOCATION_INFO_MILE_TO_KILOMETER  1.8
#define BNA_LOCATION_DISTRICT_MAX_LENGTH        32
#define BNA_ANIMATION_NODE_MAX  20

//location info
typedef struct _BNALocationServiceInfo
{
    char      date_time[20];       //time string
    char      status;                    //accept status: A:good; V:valid
    double latitude;
    double longitude;
    char      north_south;//南北极
    char      east_west;//东西经
    double  speed;//速度
    double  high;//高度
    int          satellite_num;//卫星个数
    double  course;//方向角度
}BNALocationServiceInfo;

//point
typedef struct _BNALocationServicePoint
{
    double latitude;
    double longitude;
}BNALocationServicePoint;

//district info
typedef struct _BNALocationDistritInfo
{
    int  dist_type;                                                                                          //district type
    int  dist_id;                                                                                               //districtID
    int  province_id;
    int  city_id;
    char us_name[BNA_LOCATION_DISTRICT_MAX_LENGTH];
    BNALocationServiceInfo center_point;
    unsigned int  child_cnt;
}BNALocationDistritInfo;


//direction
typedef enum _BNALocationDirectionInfo
{
    BNA_LOCATION_DIRECTION_INFO_INVALID           = 0,
    BNA_LOCATION_DIRECTION_INFO_EAST                 = 1,
    BNA_LOCATION_DIRECTION_INFO_WEST                = 2,
    BNA_LOCATION_DIRECTION_INFO_SOUTH             = 3,
    BNA_LOCATION_DIRECTION_INFO_NORTH             = 4,
    BNA_LOCATION_DIRECTION_INFO_EAST_NORTH   = 5,
    BNA_LOCATION_DIRECTION_INFO_EAST_SOUTH   = 6,
    BNA_LOCATION_DIRECTION_INFO_WEST_NORTH  = 7,
    BNA_LOCATION_DIRECTION_INFO_WEST_SOUTh   = 8,
}BNALocationDirectionInfo;




////////////////////////////////////////////////////////MapMatch////////////////////////////////////////////////////////
/** 时间 */
typedef struct _BNALocationTime
{
    unsigned short 	usYear;
    unsigned char 		ucMon;
    unsigned char 		ucDay;
    unsigned char 		ucHour;
    unsigned char 		ucMin;
    unsigned char 		ucSecond;
    unsigned char		ucReserved;
    unsigned int		unMillisecond;
}BNALocationTime;

//position
typedef struct _BNALocationPosPoint
{
    double x;
    double y;
}BNALocationPosPoint;

/** GPS推算类型 */
typedef enum _BNAGPSDRType
{
    BNA_GPS_DR_TYPE_INVALID ,	/**< 无效值 */
    BNA_GPS_DR_TYPE_INITPOS,	/**< 初始位置 */
    BNA_GPS_DR_TYPE_NODR,		/**< 未经过推算 */
    BNA_GPS_DR_TYPE_INS ,		/**< 惯导推算 */
    BNA_GPS_DR_TYPE_AVE_SPEED	/**< 平均速度推算 */
}BNAGPSDRType;


/** GPS漂移类型 */
typedef enum _BNAGPSDriftType
{
    BNA_GPS_DRIFTTYPE_UNKNOWN ,		/**< 未知 */
    BNA_GPS_DRIFTTYPE_SINGLESIDE ,	/**< 单侧遮挡 */
    BNA_GPS_DRIFTTYPE_TOPSIDE		/**< 头顶遮挡 */
}BNAGPSDriftType;

/*
 * 定位模式
 * note : 注意与JNIGuidance.java中常量的对应
 */
typedef enum _BNALocateMode
{
    BNA_LOCATE_MODE_INVALID ,		/** 无效值 */
    BNA_LOCATE_MODE_GPS ,			/** 真实GPS位置 */
    BNA_LOCATE_MODE_ROUTEDEMOGPS ,	/** 从路线模拟位置 */
    BNA_LOCATE_MODE_NEMADEMOGPS ,	/** 从GPS NEMA数据文件模拟位置 */
    BNA_LOCATE_MODE_MANUALDEMOGPS,  /** 通过键盘操作手动模拟GPS位置 */
    BNA_LOCATE_MODE_NMEADEMOGPS = 100   /** 从GPS NMEA数据文件模拟位置 */
}BNALocateMode;

/** 动画节点类型 */
typedef enum _BNAAnmationNodeType
{
    BNA_ANMATIONNODE_FAST,			//快速行驶
    BNA_ANMATIONNODE_LOW,			//低速行驶
    BNA_ANMATIONNODE_CROSS_ENTER,	//快要进入机动点，减速，放大，2d切换
    BNA_ANMATIONNODE_CROSS_OUT,	//走出机动点一段距离，开始加速，缩小，3d切换
}BNAAnmationNodeType;


/** 定位统计项--偏航类型*/
typedef enum _BNAStatisticYawType
{
    BNA_STATISTIC_INVALIDYAW = 0x00000000,   /** 无效*/
    BNA_STATISTIC_FASTYAW = 0x00000001,    /** 快速偏航*/
    BNA_STATISTIC_FORCEYAW = 0x00000002,   /** 强制偏航*/
    BNA_STATISTIC_NORMALYAW = 0x00000003   /** 正常偏航*/
}BNAStatisticYawType;



/**定位统计项--偏航点位置类型*/
typedef enum _BNAStatisticYawLocType
{
    BNA_STATISTIC_INVALIDPOS = 0x00000000,   /** 无效*/
    BNA_STATISTIC_YAWSTARTPOS = 0x00000001,  /** 起点偏航*/
    BNA_STATISTIC_YAWMIDPOS = 0x00000002,    /** 导航过程中偏航*/
    BNA_STATISTIC_YAWENDPOS = 0x00000003     /** 终点偏航*/
}BNAStatisticYawLocType;


/** 导航模式 */
typedef enum _BNANaviMode
{
    BNA_NAVI_MODE_INVALID = 0,     /**< 无效 */
    BNA_NAVI_MODE_NORMAL,          /**< 正常导航模式 */
    BNA_NAVI_MODE_SLIGHT,          /**< 轻导航模式 */
}BNANaviMode;

/** 轻导航自动刷新路线类型 */
typedef enum _BNALightNaviAutoRefreshType
{
    BNA_LIGHTNAVI_AUTOREFRESH_TYPE_INVALID = 0,  /**< 无效值 */
    BNA_LIGHTNAVI_AUTOREFRESH_TYPE_SINGLEROUTE,  /**< 单一路线刷新 */
    BNA_LIGHTNAVI_AUTOREFRESH_TYPE_PASSDECPOINT, /**< 过分歧点自动刷新 */
}BNALightNaviAutoRefreshType;


/** 匹配状态 */
typedef enum _BNAMMState
{
    BNA_MAP_MATCH_INVALID ,				/**<  无效 */
    BNA_MAP_MATCH_YAW ,					/**<  偏航 */
    BNA_MAP_MATCH_FORCEYAW,				/**<  强制偏航*/
    BNA_MAP_MATCH_ONROUTE,				/**<  匹配在路线上 */
    BNA_MAP_MATCH_TRYAW                 /**<  限行路段触发偏航*/
}BNAMMState;

/** 到达状态 */
typedef enum _BNAMMArriveState
{
    BNA_MM_ARRIVE_STATE_NO ,		/**<  无效 */
    BNA_MM_ARRIVE_STATE_1,			/**<  到达1判定成功 */
    BNA_MM_ARRIVE_STATE_2			/**<  达到2判定成功 */
}BNAMMArriveState;


/**route Shape ID*/
typedef struct _BNARouteShapID
{
    int 	nLegIdx;	/**< leg Index*/
    int 	nStepIdx;	/**< Step Index */
    int	    nLinkIdx;	/**< Link Index */
    int 	nShapeIdx;	/**< ShapePoint Index */
    bool	bIsLast;	/**< 是否是Link上最后一个ShapeNode */
}BNARouteShapID;

/**route Link ID*/
typedef struct _BNARouteLinkID
{
    int 	nLegIdx;	/**< leg Index*/
    int 	nStepIdx;	/**< Step Index */
    int	nLinkIdx;	/**< Link Index */
    bool	bIsLast;	/**< 是否是最后一个Link */
}BNARouteLinkID;


/** GPS状态，输出(经过惯导推算) */
typedef struct _BNAGPSStatus
{
    bool 					bFixed;			/**< 是否定位(包括实际定位和推算定位) */
    bool 					bDrift;			/**< 是否漂移 */
    bool 					bLose;			/**< 是否丢失 */
    BNAGPSDriftType	enDirftType;	/**< 漂移类型 */
}BNAGPSStatus;

/** GPS星历 */
typedef struct _BNAGPSStar
{
    BNALocationTime	 stTime;			/**<  卫星时间 */
    int		nStarCnt;		/**<  卫星数 */
    struct Star
    {
        int nStarNo;			/**<  卫星编号[01-32] */
        int nUpAngle;			/**<  仰角[0-90]度 */
        int nAngle;			/**<  方位角[0-359]度 */
        int nSN;				/**<  信噪比[0-99]dB */
    }StarInfo[12];
}BNAGPSStar;

/** GPS星历 */
typedef struct _BNAGPSStarInfo
{
    int		nStarVisibleCnt;	/**<  可见卫星数 */
    int		nStarUsedCnt;		/**<  使用卫星数 */
    struct Star
    {
        int	nStarId;			/**<  卫星编号[01-32] */
        float  fUpAngle;			/**<  仰角[0-90]度 */
        float  fAngle;				/**<  方位角[0-359]度 */
        float  fSNR;				/**<  信噪比[0-99]dB */
        bool	bIsUsed;			/**<  是否在定位中使用 */
        bool	bIsHaveAlmanac;		/**<  是否有历书*/
        bool	bIsHaveEphemeris;	/**<  是否有星历*/
    }StarInfo[60];
}BNAGPSStarInfo;

/** 动画节点定义 */
typedef struct _BNAAnimationNode
{
    float						fSpeed;			//速度，单位：m/s
    float						fRoate;			//旋转角度,0-360
    BNAAnmationNodeType       eAnimationType;
    BNALocationPosPoint					stPos;			//点坐标，单位：默卡托
    float						fTime;			//此段动画执行时间
    double					fDist;			//此段动画总长度
}BNAAnimationNode;

typedef struct _BNAAnmationNodes
{
    BNAAnimationNode			nodes[BNA_ANIMATION_NODE_MAX];	/**< 动画节点，最多ANIMATION_NODE_MAX个点*/
    int						nCount;						/**< 动画节点个数*/
    double						dLastCrossDist;				/**< 上一个机动点积距*/

    int 	nLegIdx;										/**< 最后一个点leg Index*/
    int 	nStepIdx;										/**< 最后一个点Step Index */
    int	nLinkIdx;										/**< 最后一个点 Link Index */
    int 	nShapeIdx;										/**< 最后一个点ShapePoint Index */
    double dShapeOffset;									/**< 与最后一个点偏移量*/
    BNALocationPosPoint lastPoint;                                     /**< 上次最后一个点的经纬度坐标*/
}BNAAnmationNodes;

/** GPS位置 */
typedef struct _BNAGPSPos
{
    BNALocationTime			stTime;						/**< 系统时间 */
    unsigned int				unTickCount;				/**< TickCount */
    BNALocationPosPoint			stPos;						/**< 位置 */
    float				fSpeed;						/**< 速度，单位米/秒 */
    float				fAngle;						/**< 方位角，单位度，正北方向为起点顺时针 */
    float				fPrecision;					/**< 精确度 */
    float				fVPrecision;				/**< 垂直精确度 */
    BNAGPSDRType	enDRType;					/**< 推算类型 */
    double 			dShapeStartOffsetFromLink;	/**< Shape Start离多线段起点偏移，对于推算有效 */
    double 			dOffsetFromShapeLineStart;	/**< 离形状Link起点偏移，对于推算有效 */
    bool				bIsWifiLoc;					/**< 是否为wifi定位点*/
    float				fAltitude;					/**< 海拔（高度）*/
}BNAGPSPos;

////////////////////////////////GPS Result////////////////////////////////
typedef struct _BNAGPSResult
{
    int						nGPSChangeKind;	/**< GPS改变类型 */
    BNALocateMode			enLocateMode;	/**< 定位模式 */
    BNAGPSStatus				stConnectStatus;/**< GPS连接状态 */
    BNAGPSStar				stStar;			/**< GPS星历 */
    BNAGPSStarInfo			stStarInfo;		/**< GPS星历,由客户传递 */
    BNAGPSPos				stPos;			/**< GPS位置 */
    double						dGPSConfidence; /**< GPS 可信度 0.01 - 1.0*/
    bool						   bIsTunnelFlag; /**< 当前GPS匹配link向前一段范围是否为隧道*/
    BNAAnmationNodes			anmationNodes;	/**< 动画节点*/
    bool						bIsRealLoc;      /**< 是否真实定位点(包括GPS、Wifi)*/
    bool						bIsCalcFailed;  /**< 未定位成功且推算失败*/
    unsigned int            unSequence; /** 序列号,映射MapMatching和DR输出 */
}BNAGPSResult;

////////////////////////////////Sensor Angle////////////////////////////////
typedef struct _BNASensorAngle
{
    double		fTrueHeading;     /**< Y轴和正北方向的夹角(该值不准确时，上层会直接使用与地磁北极的夹角)，单位度 */
    double		fHeadingAccuracy; /**< Y轴和正北夹角准确度，单位度*/
}BNASensorAngle;


/** 匹配位置 */
typedef struct _BNAMatchPos
{
    unsigned int			unOriGPSTickCount;	/**< TickCount */
    BNALocationPosPoint		stOriGPSPos;		/**< 原始GPS位置 */
    float			fOriSpeed;			/**< 原始GPS速度，单位米/秒 */
    float			fOriAngle;			/**< 原始GPS方位角，单位度，正北方向为起点顺时针 */
    float          fPrecision;         /**< 原始GPS水平精度 */
    float          fVPrecision;        /**< 原始GPS垂直精度*/
    BNAGPSDRType	enOriDRType;    /**< 原始GPS推算类型 */
    BNALocationPosPoint 		stDestGPSPos;		/**< 匹配后GPS位置 */
    float			fSpeed;				/**< 匹配后速度，单位米/秒 */
    float			fAngle;				/**< 匹配后方位角，单位度，正北方向为起点顺时针 */
    BNARouteShapID stRouteShapeID; 	/**< 匹配路线上的索引 */
    unsigned int 			unTotalShapeIdx;	/**< 总的Shape索引 */
    // need triple length
    char			usMatchLinkName[NE_ROADNAME_MAX_LENGTH*3];
                                        /**< 匹配道路名字 */
    unsigned int			unAddDist;			/**< 匹配位置积算距离，单位米 */
    double			dLinkLength;		/**< 匹配Link长度，单位米 */
    double			dShapeStartOffsetFromLink;
                                        /**< 当前匹配ShapeLink起点在Link上的偏移距离，单位米 */
    double			dOffsetFromShapeStart;
                                        /**< 在当前Shape上的偏移距离，单位米 */
    float			fPrjDist;			/**< 投影距离，单位米 */
    float			fDiffAngle;			/**< 差分角，单位度 */
    float			fLineDiffAngle;		/**< 连线差分角，单位度 */
    double			dWeightValue;		/**< 匹配权值 */
    double         dWeightOffset;      /**< 匹配权值偏移 */
    bool           bIsBestRoadMatch;   /**< 是否路网最佳匹配 */

    bool			bIsOnNavRoute;		/**< 当前匹配点是否在导航路径上*/
    bool			bExistNearstCross;	/**< 当前匹配点周边一定距离内是否存在路口*/
    bool			bNearstCrossForward;/**< 当前匹配点最近路口是否在自车前方*/
    double 		dMinDistToCross;	/**< 当前匹配点距离最近路口的距离，单位米，bExistNearstCross为TRUE时有效*/
    double			dDistToAdjacentCross;
                                        /**< 当前匹配点距离另一个路口的距离 单位米*/
    BNARouteLinkID	stNearstCrossLinkID;/**< 当前匹配点距离最近路口的LinkID*/
    int			nLinkLevel;			/**< 当前匹配点所在道路的等级*/
    unsigned int   		unLinkGpsBias;		/**< 当前匹配点所在道路的GPS偏差率*/
    BNALocationPosPoint        stVechicleFreePos;  /**< 车标自由位置*/
    float          fVechicleFreeAngle; /**< 车标自由角度*/
    bool			bIsTimeRegularFlag;      /**< 当前匹配点是否能够触发限行路段*/
    unsigned int			unStartTimeRegularHour;  /**< 限行交规开始时间（时）*/
    unsigned int			unStartTimeRegularMin;   /**< 限行交规开始时间（分）*/
    float			fMapAngle;				 /**< 用于图像展示的角度*/
    unsigned int            unLinkProperty;        /**< link道路属性 */
}BNAMatchPos;

////////////////////////////////Match Result////////////////////////////////
typedef struct _BNAMapMatchResult
{
    BNAStatisticYawType enYawType;		/**<  偏航类型*/
    BNAStatisticYawLocType  enYawLocType;    /**<  偏航点位置类型*/
    BNAMMState			enMatchState;       /**<  匹配状态 */
    BNAMMArriveState	enArriveState;      /**<  到达状态 */
    BNALocationTime				stGPSTime;			/**<  GPS时间 */
    unsigned int					unGPSTickCount;		/**<  TickCount */
    BNAMatchPos				stMatchPos;         /**<  匹配位置 */
    float				    fAltitude;			/**< 海拔（高度）*/
    unsigned int					unYawTickCount;     /**<  偏航时间 */
    bool					bIsWait;            /**<  是否观望 */
    bool                   bIsVehicleFree;     /**<  是否车标自由状态 */
    bool                   bIsFreeWait;        /**<  是否车标自由等待 */
    bool                   bCruiseYaw;         /**<  巡航模式的伪偏航 */
    bool                   bCruiseBufferData;  /**<  巡航模式新缓存数据 */
    bool					bIsMainSlaveSwitch; /**< 是否主辅路口 */
    bool					bIsInstantSwitch;   /**< 是否可以进行主辅路、高架路切换*/
    bool					bIsModifyPos;		/**< 是否纠偏后点*/
    int					nPreCrossDist;		/**< 距离前个路口距离*/
    int					nNextCrossDist;	    /**<距离下个路口距离*/
    bool				    bIsNoLink;			/**<路网匹配判断周围是否没有路网*/
    bool					bIsYawChange;		/**< 判断是否主动偏航*/
    bool					bIsStartPosYaw;		/**< 是否起点偏航*/
    bool					bIsShowChangeRoadUI;/**< 是否显示在线主辅路切换框*/
    bool					bIsHideChangeRoadUI;/**< 是否消隐在线主辅路切换框*/
    unsigned int					unMatchRouteNum;	/**< [轻导航]当前匹配的路线编号，可能取值0,1,2*/
    unsigned char 					chHideRouteNum;		/**< [轻导航]需要消隐的路线编号，按位表示相应路线（最低为表示0号路，以此类推），
                                                     需要消隐则置相应位为1*/
    bool 					bIsWifiLoc;			/**< 是否wifi定位点匹配结果*/
    BNANaviMode       enNaviMode;			/**< 当前导航模式*/
    int					nYawJudgeTime;		/**< 偏航或切换路线的响应时间*/

    float					fPreseureValue;			/**< 气压传感器值 */
    bool					bIsViaductYaw;			/**< 是否为高架区域偏航 */
    bool					bIsNeedViaducJudge;		/**< 是否需要触发高架定位判断 */
    int					nViaductYawPosition;	/**< 高架区域偏航车真实位置，无效/普通道路/高架/匝道 */
    bool					bIsNeedPressureValue;	/**< 是否需要利用气压计进行判断 */
    bool					bIsNeedRefreshRoute;	/**< 轻导航相关，是否需要刷新路线 */
    int                    nToChangeRouteIdx;      /**< 导航Instant待切换路线编号*/
    BNALightNaviAutoRefreshType          enAutoRefreshType;  /**< 轻导航相关，自动刷新路线类型 */
    unsigned int            unSequence;            /** location序列号 */
}BNAMapMatchResult;

typedef enum _BNALocationSimulationType
{
    BNA_LOCATION_SIMULATION_TYPE_NAVITRACK = 1,
    BNA_LOCATION_SIMULATION_TYPE_NMEA,
    BNA_LOCATION_SIMULATION_TYPE_CCOS,
    BNA_LOCATION_SIMULATION_TYPE_DESAYSV,
    BNA_LOCATION_SIMULATION_TYPE_ROUTE,
    BNA_LOCATION_SIMULATION_TYPE_ANDROID,
    BNA_LOCATION_SIMULATION_TYPE_DEFAULT
}BNALocationSimulationType;


NAMESPACE_END
NAMESPACE_END

#endif // NAVI_AUTO_LOCATION_DEF_H
