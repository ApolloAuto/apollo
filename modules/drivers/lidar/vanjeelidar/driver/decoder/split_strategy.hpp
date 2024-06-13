
#pragma once
#include <cmath>
#include <stdint.h>
namespace vanjee
{
namespace lidar
{
        /// @brief 机械式雷达的分帧模式接口
        class SplitStrategy
        {
        public:
            /// @brief 使用者遍历Packet中的Block，以Block的水平角为参数，调用SplitStrategy::newBlock()
            virtual bool newBlock(int64_t value, int8_t mirror_id = -1) = 0;
            virtual ~SplitStrategy() = default;
        };

        /// @brief 按Block角度分帧
        class SplitStrategyByAngle : public SplitStrategy
        {
        private:
            const int8_t split_mirror_id_; 
            int64_t prev_angle_;        

        public:
            SplitStrategyByAngle(int8_t split_mirror_id)
                : split_mirror_id_(split_mirror_id), prev_angle_(0)
            {
            }
            virtual ~SplitStrategyByAngle() = default;
            /// @brief 当前一个Block的角度`prev_angle_`在`split_angle_`之前，
            virtual bool newBlock(int64_t angle, int8_t mirror_id = -1)
            {
                if((mirror_id == -1 || mirror_id == split_mirror_id_) && angle <= prev_angle_)
                {
                    prev_angle_ = angle;
                    return true;
                }
                else
                {
                    prev_angle_ = angle;
                    return false;
                }
            }
        };
        
        /// @brief 按Block号分帧
        class SplitStrategyByBlock : public SplitStrategy
        {
        private:
            const int8_t split_mirror_id_; 
            int64_t prev_block_;        

        public:
            SplitStrategyByBlock(int8_t split_mirror_id)
                : split_mirror_id_(split_mirror_id), prev_block_(0)
            {
            }
            virtual ~SplitStrategyByBlock() = default;
            /// @brief 当前一个Block的角度`prev_angle_`在`split_angle_`之前，
            virtual bool newBlock(int64_t block, int8_t mirror_id = -1)
            {
                if((mirror_id == -1 || mirror_id == split_mirror_id_) && block <= prev_block_)
                {
                    prev_block_ = block;
                    return true;
                }
                else
                {
                    prev_block_ = block;
                    return false;
                }
            }
        };

} 

} 
