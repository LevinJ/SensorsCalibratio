#ifndef YAW_CALIB_YAW_CALIB_H_
#define YAW_CALIB_YAW_CALIB_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include <Eigen/Geometry>

#include "util.h"

using namespace SPLINTER;

class YawCalib {
    public:
        YawCalib(const std::string output_dir);
        bool LoadData(const std::string file_path, int start_frame, int end_frame);
        bool Calibrate();
        bool GetYawSegs(const SPLINTER::DataTable &sample_x, const SPLINTER::DataTable &sample_y, std::vector<SPLINTER::DataTable> &samples_yaw);
        bool CalibrateSingle(const SPLINTER::DataTable &sample_yaw, double &estimate_yaw);
        double GetFinalYaw();

    private:
        int bspine_degree_ = 4;
        int time_gap_ = 10;
        int pose_num_;
        double final_yaw_;
        std::vector<double> pose_x_;
        std::vector<double> pose_y_;
        std::vector<double> pose_yaw_;
        std::vector<double> ve_;
        std::vector<double> vn_;

        // visualization
        std::string output_dir_;
        bool save_trajectory_xy = true;
};

#endif // YAW_CALIB_YAW_CALIB_H_