#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <set>
#include "External Libraries/httplib.h"
#include "MazeSolver.h"

using nlohmann::json;

// Global memory caches to sync data over the air
std::string rawM5InstructionString = "START:1,0\n";
std::string latestMazeJson = "{}";
json globalTelemetryCache = {{"stepIdx",0},{"yaw",0.0},{"left",0},{"front",0},{"right",0},{"state","IDLE"}};
bool newMissionAvailable = false;

std::string getHeadingLabel(int dir) {
    if (dir == 0) return "NORTH";
    if (dir == 1) return "EAST";
    if (dir == 2) return "SOUTH";
    if (dir == 3) return "WEST";
    return "UNKNOWN";
}

// ── NEW: REAL-TIME WALL RADAR SCANNER ─────────────────────────────────
// Counts how many open grids are ahead in a given direction before hitting a wall or board boundary
int countOpenGrids(int startR, int startC, int direction, int maxRows, int maxCols, const json& wallsJson) {
    int openGrids = 0;
    int currR = startR;
    int currC = startC;

    // Helper to check if a wall exists between two adjacent cells
    auto hasWall = [&](int r1, int c1, int r2, int c2) {
        for (const auto& w : wallsJson) {
            if (!w.contains("between") || w["between"].size() < 2) continue;
            int cell1R = w["between"][0][0].get<int>();
            int cell1C = w["between"][0][1].get<int>();
            int cell2R = w["between"][1][0].get<int>();
            int cell2C = w["between"][1][1].get<int>();

            if (((cell1R == r1 && cell1C == c1) && (cell2R == r2 && cell2C == c2)) ||
                ((cell1R == r2 && cell1C == c2) && (cell2R == r1 && cell2C == c1))) {
                return true;
            }
        }
        return false;
    };

    while (true) {
        int nextR = currR;
        int nextC = currC;

        if      (direction == 0) nextR--; // NORTH
        else if (direction == 1) nextC++; // EAST
        else if (direction == 2) nextR++; // SOUTH
        else if (direction == 3) nextC--; // WEST

        // Check absolute grid boundaries
        if (nextR < 0 || nextR >= maxRows || nextC < 0 || nextC >= maxCols) {
            break;
        }

        // Check if a wall blocks the movement into the next cell
        if (hasWall(currR, currC, nextR, nextC)) {
            break;
        }

        openGrids++;
        currR = nextR;
        currC = nextC;
    }

    return openGrids;
}

int main() {
    httplib::Server svr;

    auto add_cors_headers = [](httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin",  "*");
        res.set_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
    };

    svr.Options("/send_maze", [&](const httplib::Request&, httplib::Response& res) { add_cors_headers(res); res.status = 204; });
    svr.Options("/get_telemetry", [&](const httplib::Request&, httplib::Response& res) { add_cors_headers(res); res.status = 204; });
    svr.Options("/solve", [&](const httplib::Request&, httplib::Response& res) { add_cors_headers(res); res.status = 204; });

    // ── ROUTE A: CHROME WEB PAGE PACKS & UPLOADS MAZE BLUEPRINT ──────
    svr.Post("/send_maze", [&](const httplib::Request& req, httplib::Response& res) {
        add_cors_headers(res);
        try {
            json request = json::parse(req.body);
            std::cout << "\n======================================================\n";
            std::cout << "[WIFI SERVER] New maze layout received from Chrome!\n";

            std::string spawnDirStr = request["robot_direction"].get<std::string>();
            int initialSpawnDirectionInt = 1; // Default EAST
            if      (spawnDirStr == "north") initialSpawnDirectionInt = 0;
            else if (spawnDirStr == "south") initialSpawnDirectionInt = 2;
            else if (spawnDirStr == "west")  initialSpawnDirectionInt = 3;

            MazeSolver solver(request);
            json solution;
            solver.dumpSolution(solution);

            if (solution["feasible"].get<bool>() == false) {
                std::cout << "[STATUS] Solver declared layout INFEASIBLE!\n";
                res.set_content("{\"status\":\"stored\",\"feasible\":false}", "application/json");
                return;
            }

            json path = solution["path"];
            int totalNodes = path.size();
            int rows = request["config"]["rows"].get<int>();
            int cols = request["config"]["columns"].get<int>();
            json wallsJson = request["walls"];

            std::stringstream textStream;
            int totalCompressedSteps = 0;
            std::stringstream stepBuffer;

            int i = 0;
            while (i < totalNodes - 1) {
                int currR = path[i]["cell"][0].get<int>();
                int currC = path[i]["cell"][1].get<int>();
                int nextR = path[i+1]["cell"][0].get<int>();
                int nextC = path[i+1]["cell"][1].get<int>();

                int activeHeading = 1;
                if      (nextR < currR) activeHeading = 0;
                else if (nextR > currR) activeHeading = 2;
                else if (nextC > currC) activeHeading = 1;
                else if (nextC < currC) activeHeading = 3;

                int gridsCount = 0;
                int j = i;
                int actionCode = 0; // Default: normal drive step

                while (j < totalNodes - 1) {
                    int rA = path[j]["cell"][0].get<int>();
                    int cA = path[j]["cell"][1].get<int>();
                    int rB = path[j+1]["cell"][0].get<int>();
                    int cB = path[j+1]["cell"][1].get<int>();

                    int checkHeading = 1;
                    if      (rB < rA) checkHeading = 0;
                    else if (rB > rA) checkHeading = 2;
                    else if (cB > cA) checkHeading = 1;
                    else if (cB < cA) checkHeading = 3;

                    if (checkHeading == activeHeading) {
                        gridsCount++;
                        j++;
                        // If the cell we just arrived at is tagged as a charging station,
                        // force a stop right here instead of flowing through it!
                        if (path[j]["type"].get<std::string>() == "charge") {
                            actionCode = 1; // Mark this step as a charging action step
                            break;
                        }
                    }
                    else {
                        break;
                    }
                }

                int targetIntersectionR = path[j]["cell"][0].get<int>();
                int targetIntersectionC = path[j]["cell"][1].get<int>();

                // FIXED: Use the wall radar function to calculate real physical space parameters
                int totalOpenGridsAhead = countOpenGrids(currR, currC, activeHeading, rows, cols, wallsJson);
                int stopLimitRemainingGrids = countOpenGrids(targetIntersectionR, targetIntersectionC, activeHeading, rows, cols, wallsJson);

                stepBuffer << "STEP:" << totalCompressedSteps << "," << activeHeading << ","
                           << gridsCount << "," << totalOpenGridsAhead << "," << stopLimitRemainingGrids << "," << actionCode << "\n";

                std::cout << " GENERATED STEP [" << totalCompressedSteps << "] --> " << getHeadingLabel(activeHeading)
                          << " | Run Grids: " << gridsCount << " | Stop Expected Grids Ahead: " << stopLimitRemainingGrids << "\n";
                totalCompressedSteps++;
                i = j;
            }

            textStream << "START:" << initialSpawnDirectionInt << "," << totalCompressedSteps << "\n";
            textStream << stepBuffer.str();
            rawM5InstructionString = textStream.str();
            latestMazeJson = request.dump();

            std::cout << "[WIFI SERVER] Path unrolled. Mission cached inside memory router.\n";
            std::cout << "------------------------------------------------------\n";
            std::cout << ">>> RAW MISSION PAYLOAD CACHED FOR M5 WI-FI PULL <<<\n";
            std::cout << rawM5InstructionString;
            std::cout << "------------------------------------------------------\n";
            std::cout << "======================================================\n\n";
            newMissionAvailable = true;
            res.set_content("{\"status\":\"stored\",\"feasible\":true}", "application/json");
        }
        catch (const std::exception& e) {
            res.status = 400;
            res.set_content("{\"status\":\"error\"}", "application/json");
        }
    });

    // ── ROUTE B: M5 STICK REQUESTS INSTRUCTIONS OVER WI-FI ──────────
    svr.Get("/get_instructions", [&](const httplib::Request&, httplib::Response& res) {
        res.set_content(rawM5InstructionString, "text/plain");
        if (newMissionAvailable) {
            std::cout << "[WIFI LINK] M5 Stick successfully pulled a fresh mission array over Wi-Fi!\n";
            newMissionAvailable = false;
        }
    });

    // ── ROUTE C: M5 STICK GETS MAZE JSON ────────────────────────────
    svr.Get("/get_maze", [&](const httplib::Request&, httplib::Response& res) {
        res.set_content(latestMazeJson, "application/json");
    });

    // ── ROUTE D: M5 STICK PUSHES LIVE CLOSED-LOOP TELEMETRY ──────────
    svr.Post("/update_telemetry", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            json data = json::parse(req.body);
            globalTelemetryCache["stepIdx"] = data["stepIdx"];
            globalTelemetryCache["yaw"]     = data["yaw"];
            globalTelemetryCache["left"]    = data["left"];
            globalTelemetryCache["front"]   = data["front"];
            globalTelemetryCache["right"]   = data["right"];
            globalTelemetryCache["state"]   = data["state"];

            std::cout << "[TELEMETRY] State: " << data["state"].get<std::string>()
                      << " | Yaw: " << data["yaw"].get<float>() << " deg\n";

            res.status = 200;
            res.set_content("{\"status\":\"ok\"}", "application/json");
        } catch(...) {
            res.status = 400;
            res.set_content("{\"status\":\"error\"}", "application/json");
        }
    });

    // ── ROUTE E: CHROME BROWSER POLLS TELEMETRY FOR SHADOW DRAWING ───
    svr.Get("/get_telemetry", [&](const httplib::Request&, httplib::Response& res) {
        add_cors_headers(res);
        res.set_content(globalTelemetryCache.dump(), "application/json");
    });

    // ── ROUTE F: PATH SOLVER ──────────────────────────────────────────
    svr.Post("/solve", [&](const httplib::Request& req, httplib::Response& res) {
        add_cors_headers(res);
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

    // ── ROUTE G: M5 TO PC MESSAGE ROUTE ───────────────────────────────
    svr.Post("/m5_to_pc", [&](const httplib::Request& req, httplib::Response& res) {
        std::cout << "[M5 → PC] " << req.body << "\n";
        res.set_content("{\"status\":\"received\"}", "application/json");
    });

    // ── ROUTE H: PC TO M5 MESSAGE ROUTE ───────────────────────────────
    svr.Get("/pc_to_m5", [&](const httplib::Request&, httplib::Response& res) {
        res.set_content("M5 ACKNOWLEDGED", "text/plain");
    });

    // ── ROUTE I: ROOT / SERVE INDEX.HTML ──────────────────────────────
    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        std::ifstream f("index.html");
        std::string html((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
        res.set_content(html, "text/html");
    });

    std::cout << "Wireless local server listening at http://0.0.0.0:8085\n";

    if (!svr.listen("0.0.0.0", 8085)) {
        std::cerr << "\n[CRITICAL ERROR] Server failed to bind to port 8085!\n";
        return 1;
    }

    return 0;
}

