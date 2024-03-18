#include "mqp_http_client.h"

#include <iostream>
#include<unistd.h>               // for linux
#include <fmt/core.h>


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

    std::cout << readBuffer << std::endl;
    *data = json::parse(readBuffer);

    return true;
}


bool mqp_http_client::solve(std::vector<std::vector<std::vector<float>>> *path_arr, std::string host) {


    std::cout << "Calling the initial solve endpoint...\n" << std::endl;
    std::string req_url = fmt::format("{}/solve?n_a=5&k=5&q_k=0.65&rp=2&l=1.5&mode=h1&d=3", host);

    json data;
    mqp_packets::res mqp_res;
    while (true) {
        make_http_req(&data, req_url);
        if (data["status"] == "completed") { break; }
        usleep(1000000);
    }

    std::cout << data["robot_world_path"].dump()<< std::endl;

    *path_arr = data["robot_world_path"][0].get<std::vector<std::vector<std::vector<float>>>>();

    std::cout << "Ran the initial solve endpoint!\n" << std::endl;
    return true;
}