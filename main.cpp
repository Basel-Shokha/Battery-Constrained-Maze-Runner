#include <iostream>
#include "External Libraries/httplib.h"
#include "MazeSolver.h"

using nlohmann::json;


// Add this global variable near the top of main.cpp
std::string latest_maze_json = "{}";

// Inside main(), right next to your existing svr.Post("/solve", ...) endpoint:
svr.Options("/send_maze", [](const httplib::Request&, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin",  "*");
    res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type");
    res.status = 204;
});

svr.Post("/send_maze", [](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    latest_maze_json = req.body; // Store the raw layout data
    std::cout << "[SERVER] New maze configuration cached from Chrome.\n";
    res.set_content("{\"status\":\"stored\"}", "application/json");
});

svr.Get("/get_maze", [](const httplib::Request&, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_content(latest_maze_json, "application/json");
});


int main() {
    httplib::Server svr;

    // CORS preflight — browser sends this before every POST
    svr.Options("/solve", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin",  "*");
        res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        res.status = 204;
    });

    svr.Post("/solve", [](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        try {
            json request = json::parse(req.body);
            MazeSolver solver(request);
            json response;
            solver.dumpSolution(response);
            res.set_content(response.dump(4), "application/json");
        } catch (const std::exception& e) {
            std::cout << "ERROR: " << e.what() << std::endl;

            json err;
            err["feasible"] = false;
            err["message"]  = std::string("Server error: ") + e.what();
            res.status = 500;
            res.set_content(err.dump(), "application/json");
        }
    });

    std::cout << "Maze solver listening at http://localhost:8080\n";

    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        std::ifstream f("index.html");
        std::string html((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
        res.set_content(html, "text/html");
    });

    svr.listen("0.0.0.0", 8080);
}

