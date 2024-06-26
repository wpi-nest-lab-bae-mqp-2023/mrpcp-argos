#include "mqp_http_client.h"

#include <iostream>
#include <unistd.h>               // for linux

using json = nlohmann::json;

std::string replace_all(std::string str, const std::string& from, const std::string& to) {
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
    }
    return str;
}


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

    *data = json::parse(readBuffer);

    return true;
}


bool mqp_http_client::solve(std::vector<std::vector<std::vector<std::vector<double>>>> *path_arr,
                            std::string host,
                            unsigned int k,
                            unsigned int n_a,
                            double fcr,
                            unsigned int rp,
                            double ssd,
                            std::string mode) {

    std::cout << "Calling the initial solve endpoint...\n" << std::endl;
    std::string req_url = host+"/solve?k="+std::to_string(k)+"&n_a=" + std::to_string(n_a) + "&fcr=" + std::to_string(fcr) +  "&ssd=" + std::to_string(ssd) + "&mode=" + mode + "&rp=" + std::to_string(rp);

    json data;
    mqp_packets::res mqp_res;
    while (true) {
        make_http_req(&data, req_url);
        if (data["status"] == "completed") { break; }
        usleep(1000000);
    }

    *path_arr = data["robot_world_path"].get<std::vector<std::vector<std::vector<std::vector<double>>>>>();

//    std::cout << "Ran the initial solve endpoint!\n" << std::endl;
    return true;
}

bool mqp_http_client::recalculate(std::vector<std::vector<std::vector<std::vector<double>>>> *path_arr,
                                  std::string host,
                                  unsigned int k,
                                  unsigned int n_a,
                                  double fcr,
                                  unsigned int rp,
                                  double ssd,
                                  std::string mode,
                                  std::string curr_fuel_levels,
                                  std::string curr_robots_pos
                          ) {


    std::cout << "Attempting to recalculate...\n" << std::endl;

    std::string req_url = host+"/recalculate?job_id="+std::to_string(k)+"_" + std::to_string(n_a)  +  "_" + std::to_string(ssd) + "_" + std::to_string(fcr) + "_" + std::to_string(rp) + "_m&curr_robots_pos=" + curr_robots_pos + "&curr_fuel_levels=" + curr_fuel_levels;
    req_url = replace_all(req_url, " ", "");
    req_url = replace_all(req_url, ",", "%2C");

    json data;
    mqp_packets::res mqp_res;

    while (true) {
        make_http_req(&data, req_url);
        if (data["status"] == "completed") { break; }
        usleep(1000000);
    }

    *path_arr = data["robot_world_path"].get<std::vector<std::vector<std::vector<std::vector<double>>>>>();

//    std::cout << "Ran the initial solve endpoint!\n" << std::endl;
    return true;
}


void mqp_http_client::printPaths(std::vector<std::vector<std::vector<std::vector<double>>>> path_arr) {
    for (int ki = 0; ki < path_arr.size(); ++ki) {
        std::cout << "Robot #" << ki << std::endl;
        mqp_http_client::printPath(path_arr[ki]);
    }
}


void mqp_http_client::printPath(std::vector<std::vector<std::vector<double>>> path_arr) {
    for (int subtouri = 0; subtouri < path_arr.size(); ++subtouri) {
        std::cout << "\tSubtour #" << subtouri << std::endl;
        for (int pointi = 0; pointi < path_arr[subtouri].size(); ++pointi) {
            std::cout << "\t\tPoint: [" << path_arr[subtouri][pointi][0] << ", " << path_arr[subtouri][pointi][1] << "]" << std::endl;
        }
    }
}
