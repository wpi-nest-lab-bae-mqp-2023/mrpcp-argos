//
// Created by Yaşar İdikut on 4/17/24.
//

#ifndef MRPCP_ARGOS_POSITION_LOGGER_H
#define MRPCP_ARGOS_POSITION_LOGGER_H
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <filesystem>
#include <iostream>
#include <utility>
#include <fstream>
#include <sstream>
#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include "controllers/kheperaiv_orca_failure_mqp/kheperaiv_orca_failure_mqp.h"


class position_logger {
public:
    position_logger(std::string position_logging_output_folder);
    virtual ~position_logger() {}

    void write_to_logs(std::vector<CKheperaIVEntity*> cKheperaIVs);

    std::string position_logging_output_folder;
    std::string position_logging_output_file;
    std::string position_logging_output_file_w_stamp;

    unsigned int ticks;
};


#endif //MRPCP_ARGOS_POSITION_LOGGER_H
