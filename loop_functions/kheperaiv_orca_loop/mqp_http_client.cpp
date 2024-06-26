#include "mqp_http_client.h"

#include <iostream>
#include <unistd.h>               // for linux

#include <argos3/core/utility/logging/argos_log.h>


using json = nlohmann::json;


size_t mqp_http_client::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

bool mqp_http_client::make_http_req(nlohmann::json *data, const std::string& req_url) {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if (!curl) {
        return false;
    }
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "");
    curl_easy_setopt(curl, CURLOPT_URL, req_url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

//    std::cout << readBuffer << std::endl;
    *data = json::parse(readBuffer);

    return true;
}


bool mqp_http_client::solve(std::vector<std::vector<std::vector<std::vector<float>>>> *path_arr,
                            std::string host,
                            int k,
                            float n_a,
                            float fcr,
                            float fr,
                            float ssd,
                            std::string mode,
                            int rp) {


    std::string req_url = host+"/solve?k="+std::to_string(k)+"&n_a=" + std::to_string(int(n_a)) + "&fcr=" + std::to_string(fcr) +  "&ssd=" + std::to_string(int(ssd)) + "&mode=" + mode + "&rp=" + std::to_string(int(rp));

    json data;
    mqp_packets::res mqp_res;
    while (true) {
        make_http_req(&data, req_url);
        if (data["status"] == "completed") { break; }
        usleep(1000000);
    }

    *path_arr = data["robot_world_path"].get<std::vector<std::vector<std::vector<std::vector<float>>>>>();

    return true;
}

bool mqp_http_client::recalculate(std::vector<std::vector<std::vector<std::vector<float>>>> *path_arr,
                            std::string host,
                            int curr_robots_pos,
                            int curr_fuel_levels,
                            int failed_robot_id) {

    std::cout << "Calling the recalculate endpoint...\n" << std::endl;


    /*
    std::string req_url = fmt::format("{}/recalculate?curr_robots_pos={}&curr_fuel_levels={}&failed_robot_id={}",
                                      host,
                                      curr_robots_pos,
                                      curr_fuel_levels,
                                      failed_robot_id
                                    );
    */


    json data;
    mqp_packets::res mqp_res;
    while (true) {
        make_http_req(&data, req_url);
        if (data["status"] == "completed") { break; }
        usleep(1000000);
    }

    *path_arr = data["robot_world_path"].get<std::vector<std::vector<std::vector<std::vector<float>>>>>();

//    std::cout << "Ran the initial solve endpoint!\n" << std::endl;
    return true;
}


void mqp_http_client::printPaths(std::vector<std::vector<std::vector<std::vector<float>>>> path_arr) {
    for (int ki = 0; ki < path_arr.size(); ++ki) {
        std::cout << "Robot #" << ki << std::endl;
        mqp_http_client::printPath(path_arr[ki]);
    }
}


void mqp_http_client::printPath(std::vector<std::vector<std::vector<float>>> path_arr) {
    for (int subtouri = 0; subtouri < path_arr.size(); ++subtouri) {
        std::cout << "\tSubtour #" << subtouri << std::endl;
        for (int pointi = 0; pointi < path_arr[subtouri].size(); ++pointi) {
            std::cout << "\t\tPoint: [" << path_arr[subtouri][pointi][0] << ", " << path_arr[subtouri][pointi][1] << "]" << std::endl;
        }
    }
}
