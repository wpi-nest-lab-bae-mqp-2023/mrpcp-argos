#ifndef MRPCP_ARGOS_ORCA_FAILURE_MQP_HTTP_CLIENT_H
#define MRPCP_ARGOS_ORCA_FAILURE_MQP_HTTP_CLIENT_H

#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace mqp_packets {
    struct res {
        std::string job_id;
        std::map<std::string, std::string> params;

        std::vector<std::vector<int>> robot_node_path;
        std::vector<std::vector<double>> robot_world_path;

        std::string status;
    };
        NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(res, job_id, params, robot_node_path, robot_world_path, status);
}

class mqp_http_client {
public:
    static bool solve(std::vector<std::vector<std::vector<std::vector<double>>>> *path_arr,
                      std::string host,
                      unsigned int k,
                      unsigned int n_a,
                      double fcr,
                      unsigned int rp,
                      double ssd,
                      std::string mode);
    static void printPaths(std::vector<std::vector<std::vector<std::vector<double>>>> path_arr);
    static void printPath(std::vector<std::vector<std::vector<double>>> path_arr);

private:
    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);
    static bool make_http_req(nlohmann::json *data, const std::string& url);

};


#endif //MRPCP_ARGOS_MQP_HTTP_CLIENT_H
