export type useStatisticsType =
    | boolean
    | {
          useMax?: boolean;
          useMin?: boolean;
          useSum?: boolean;
          useCount?: boolean;
          useAverage?: boolean;
      };
export interface MetricOptions {
    // 是否使用统计功能
    useStatistics?: useStatisticsType;
    // 转换函数
    transformFunc?: (metric: any) => any;
}
