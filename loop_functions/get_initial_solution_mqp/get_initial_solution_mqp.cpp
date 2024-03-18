#include "get_initial_solution_mqp.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <sstream>
#include <list>
#include <curl/curl.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <sstream>
#include <list>
#include <fstream>
#include <unistd.h>

#include "loop_functions/get_initial_solution_mqp/mqp_http_client.h"



using json = nlohmann::json;


size_t GetInitialSolutionMQP::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
 {
     ((std::string*)userp)->append((char*)contents, size * nmemb);
     return size * nmemb;
 }



/****************************************/
/****************************************/

static const Real        FB_RADIUS        = 0.085036758f;
static const Real        FB_AREA          = ARGOS_PI * Square(0.085036758f);
static const std::string FB_CONTROLLER    = "ffc";
static const UInt32      MAX_PLACE_TRIALS = 20;
static const UInt32      MAX_ROBOT_TRIALS = 20;

/****************************************/
/****************************************/

GetInitialSolutionMQP::GetInitialSolutionMQP() {
}

/****************************************/
/****************************************/

GetInitialSolutionMQP::~GetInitialSolutionMQP() {
}

/****************************************/
/****************************************/

void GetInitialSolutionMQP::Init(TConfigurationNode& t_tree) {
    std::cout << "Setting up in get_initial_solution_mqp.cpp\n" << std::endl;

    mqp_http_client::solve(&path_arr, "http://127.0.0.1:5000");

//    try {
//         CURL *curl;
//         CURLcode res;
//         std::string readBuffer;
//
//         curl = curl_easy_init();
//         if(curl) {
//             curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "");
//             curl_easy_setopt(curl, CURLOPT_URL, "http://127.0.0.1:5000/solve?n_a=3&k=5&q_k=0.65&rp=1&l=1&mode=m&d=3");
//             curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
//             curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
//             res = curl_easy_perform(curl);
//             curl_easy_cleanup(curl);
//
//             std::cout << readBuffer << std::endl;
//             // EXPECTED OUTPUT
//             // {
//             //  "userId": 1,
//             //  "id": 1,
//             //  "title": "delectus aut autem",
//             //  "completed": false
//             //}
//
//             json data = json::parse(readBuffer);
////             data["title"] = "Something else...";
////
//             std::cout << data["robot_world_path"].dump() << std::endl;
//             // EXPECTED OUTPUT
//             // {
//             //  "userId": 1,
//             //  "id": 1,
//             //  "title": "Something else...",
//             //  "completed": false
//             //}
//
//
//         }
//
//   }
//   catch(CARGoSException& ex) {
//      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
//   }

    std::cout << "Ran init in get_initial_solution_mqp.cpp\n" << std::endl;
//    usleep(100000000);

}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(GetInitialSolutionMQP, "get_initial_solution_mqp");
