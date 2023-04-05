
#include "local_path_planner.h"


void DWA::calc_dynamic_window()
{
        // 車両モデルによるWindow
    double Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    double Vd[] = {roomba_.velocity - max_accel_*dt_,roomba_.velocity + max_accel_*dt_,roomba_.yawrate  - max_dyawrate_*dt_,roomba_.yawrate  + max_dyawrate_*dt_};

    dw_.min_vel     = std::max(Vs[0], Vd[0]);
    dw_.max_vel     = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}
