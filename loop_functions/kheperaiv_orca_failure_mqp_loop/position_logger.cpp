//
// Created by Yaşar İdikut on 4/17/24.
//

#include "position_logger.h"


position_logger::position_logger(std::string output_folder):
        position_logging_output_folder(std::move(output_folder)) {

    std::cout << "Running pos logger" << std::endl;

    // If the folder doesn't exist, create it
    struct stat info;
    if (!(info.st_mode & S_IFDIR))
        std::filesystem::create_directory(position_logging_output_folder);

    position_logging_output_file = (std::filesystem::path(position_logging_output_folder) / "position_logs.txt").string();
    std::ostringstream osss;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    osss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");
    position_logging_output_file_w_stamp = (std::filesystem::path(position_logging_output_folder) / (osss.str()+"_position_logs.txt")).string();

    std::cout << "position_logging_output_file: " << position_logging_output_file << std::endl;
    std::cout << "position_logging_output_file_w_stamp: " << position_logging_output_file_w_stamp << std::endl;

    std::ofstream oss1;
    oss1.open(position_logging_output_file, std::ios::out);
    std::ofstream oss2;
    oss2.open(position_logging_output_file_w_stamp, std::ios::out);
    ticks = 0;
}


void position_logger::write_to_logs(std::vector<CKheperaIVEntity*> cKheperaIVs) {
    std::string line = "";
    for (int ki = 0; ki < cKheperaIVs.size(); ++ki) {
        auto cKheperaIV = cKheperaIVs[ki];
        auto &cController = dynamic_cast<CKheperaIVORCAFailureMQP &>(cKheperaIV->GetControllableEntity().GetController());

        CVector2 cPos;
        cPos.Set(cKheperaIV->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 cKheperaIV->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        line = line + std::to_string(cPos.GetX()) + "," + std::to_string(cPos.GetY()) + "," + std::to_string(ticks) + ";";
    }
    std::ofstream oss1;
    oss1.open(position_logging_output_file, std::ios::app);
    oss1 << line << std::endl;
    std::ofstream oss2;
    oss2.open(position_logging_output_file_w_stamp, std::ios::app);
    oss2 << line << std::endl;
    ticks += 1;
}


