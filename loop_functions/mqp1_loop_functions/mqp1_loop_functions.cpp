#include "mqp1_loop_functions.h"

#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <sstream>
#include <list>
#include <curl/curl.h>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;


size_t MQP1LoopFunctions::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
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

MQP1LoopFunctions::MQP1LoopFunctions() {
}

/****************************************/
/****************************************/

MQP1LoopFunctions::~MQP1LoopFunctions() {
}

/****************************************/
/****************************************/

void MQP1LoopFunctions::Init(TConfigurationNode& t_tree) {
    std::cout << "Setting up in mqp1_loop_functions.cpp\n" << std::endl;

    try {
         CURL *curl;
         CURLcode res;
         std::string readBuffer;

         curl = curl_easy_init();
         if(curl) {
             curl_easy_setopt(curl, CURLOPT_URL, "https://jsonplaceholder.typicode.com/todos/1");
             curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
             curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
             res = curl_easy_perform(curl);
             curl_easy_cleanup(curl);

             std::cout << readBuffer << std::endl;
             // EXPECTED OUTPUT
             // {
             //  "userId": 1,
             //  "id": 1,
             //  "title": "delectus aut autem",
             //  "completed": false
             //}

             json data = json::parse(readBuffer);
             data["title"] = "Something else...";

             std::cout << data.dump(4) << std::endl;
             // EXPECTED OUTPUT
             // {
             //  "userId": 1,
             //  "id": 1,
             //  "title": "Something else...",
             //  "completed": false
             //}


         }

   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the loop functions", ex);
   }

    std::cout << "Ran init in mqp1_loop_functions.cpp\n" << std::endl;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(MQP1LoopFunctions, "mqp1_loop_functions");
