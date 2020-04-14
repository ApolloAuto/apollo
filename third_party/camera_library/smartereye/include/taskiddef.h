#ifndef TASKIDDEF_H
#define TASKIDDEF_H

struct TaskId
{
    enum Enumeration {
        NoTask          = 0,
        ObstacleTask    = 1 << 0,
        LaneTask        = 1 << 1,
        DisplayTask     = 1 << 2,
        HeightTask      = 1 << 3,
        AllTask         = ObstacleTask | LaneTask | DisplayTask | HeightTask
    };
};

#endif // TASKIDDEF_H


