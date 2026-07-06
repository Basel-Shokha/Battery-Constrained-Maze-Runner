// ============================================================
//  main.cpp — High-Performance C++ Server Routing Backbone
// ============================================================
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <set>
#include "External Libraries/httplib.h"
#include "MazeSolver.h"

using nlohmann::json;

std::string rawM5InstructionString = "START:1,0\n";
std::string latestMazeJson = "{}";
json globalTelemetryCache = {{"stepIdx",0},{"yaw",0.0},{"left",0},{"front",0},{"right",0},{"state","IDLE"}};
bool newMissionAvailable = false;
bool calibrationRequested = false;

void printM5Packet(const std::string& packet) {
    std::cout << "\n========== PACKET SENT TO M5 ==========\n"
              << packet;

    // Keep the closing separator on its own line.
    if (packet.empty() || packet.back() != '\n') {
        std::cout << '\n';
    }

    std::cout << "=======================================\n" << std::flush;
}



std::string getHeadingLabel(int dir) {
    if (dir == 0) return "NORTH";
    if (dir == 1) return "EAST";
    if (dir == 2) return "SOUTH";
    if (dir == 3) return "WEST";
    return "UNKNOWN";
}

int countOpenGrids(int startR, int startC, int direction, int maxRows, int maxCols, const json& wallsJson) {
    int openGrids = 0;
    int currR = startR; int currC = startC;

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
        int nextR = currR; int nextC = currC;
        if      (direction == 0) nextR--;
        else if (direction == 1) nextC++;
        else if (direction == 2) nextR++;
        else if (direction == 3) nextC--;

        if (nextR < 0 || nextR >= maxRows || nextC < 0 || nextC >= maxCols) break;
        if (hasWall(currR, currC, nextR, nextC)) break;

        openGrids++;
        currR = nextR; currC = nextC;
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

    svr.Options("/send_maze",     [&](const httplib::Request&, httplib::Response& res) { add_cors_headers(res); res.status = 204; });
    svr.Options("/start_journey", [&](const httplib::Request&, httplib::Response& res) { add_cors_headers(res); res.status = 204; });
    svr.Options("/get_telemetry", [&](const httplib::Request&, httplib::Response& res) { add_cors_headers(res); res.status = 204; });
    svr.Options("/solve",         [&](const httplib::Request&, httplib::Response& res) { add_cors_headers(res); res.status = 204; });

    // ── ROUTE A: RECEIVE & BUILD STRUCT SOLUTION ────────────────────
    svr.Post("/send_maze", [&](const httplib::Request& req, httplib::Response& res) {
        add_cors_headers(res);
        try {
            json request = json::parse(req.body);
            std::string spawnDirStr = request["robot_direction"].get<std::string>();
            int initialSpawnDirectionInt = 1;
            if      (spawnDirStr == "north") initialSpawnDirectionInt = 0;
            else if (spawnDirStr == "south") initialSpawnDirectionInt = 2;
            else if (spawnDirStr == "west")  initialSpawnDirectionInt = 3;

            MazeSolver solver(request); json solution; solver.dumpSolution(solution);

            if (solution["feasible"].get<bool>() == false) {
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

                int gridsCount = 0; int j = i; int actionCode = 0;

                while (j < totalNodes - 1) {
                    int rA = path[j]["cell"][0].get<int>(); int cA = path[j]["cell"][1].get<int>();
                    int rB = path[j+1]["cell"][0].get<int>(); int cB = path[j+1]["cell"][1].get<int>();

                    int checkHeading = 1;
                    if      (rB < rA) checkHeading = 0;
                    else if (rB > rA) checkHeading = 2;
                    else if (cB > cA) checkHeading = 1;
                    else if (cB < cA) checkHeading = 3;

                    if (checkHeading == activeHeading) {
                        gridsCount++; j++;
                        if (path[j]["type"].get<std::string>() == "charge") {
                            actionCode = 1; break;
                        }
                    } else break;
                }

                int targetIntersectionR = path[j]["cell"][0].get<int>();
                int targetIntersectionC = path[j]["cell"][1].get<int>();

                int totalOpenGridsAhead = countOpenGrids(currR, currC, activeHeading, rows, cols, wallsJson);
                int stopLimitRemainingGrids = countOpenGrids(targetIntersectionR, targetIntersectionC, activeHeading, rows, cols, wallsJson);

                stepBuffer << "STEP:" << totalCompressedSteps << "," << activeHeading << ","
                           << gridsCount << "," << totalOpenGridsAhead << "," << stopLimitRemainingGrids << "," << actionCode << "\n";
                totalCompressedSteps++;
                i = j;
            }

            textStream << "START:" << initialSpawnDirectionInt << "," << totalCompressedSteps << "\n";
            textStream << stepBuffer.str();
            
            // Stash structured configuration profile in memory pool (Await trigger execution button)
            rawM5InstructionString = textStream.str();
            latestMazeJson = request.dump();
            newMissionAvailable = false;

            res.set_content("{\"status\":\"stored\",\"feasible\":true}", "application/json");
        } catch (...) {
            res.status = 400; res.set_content("{\"status\":\"error\"}", "application/json");
        }
    });

    
    
    
    // ── WEB UI REQUESTS A GYRO CALIBRATION ──────────────────────────
    svr.Post("/calibrate", [&](const httplib::Request&, httplib::Response& res) {
        add_cors_headers(res);
        calibrationRequested = true;
        std::cout << "[SERVER] Calibration requested by web UI.\n";
        res.set_content("{\"status\":\"calibration_queued\"}", "application/json");
    });

    // ── M5 POLLS THIS; FLAG SELF-CLEARS SO IT ONLY FIRES ONCE ───────
    svr.Get("/get_calibrate", [&](const httplib::Request&, httplib::Response& res) {
        add_cors_headers(res);
        if (calibrationRequested) {
            calibrationRequested = false;
            res.set_content("{\"calibrate\":true}", "application/json");
        } else {
            res.set_content("{\"calibrate\":false}", "application/json");
        }
    });

    
    
    // ── NEW: ROUTE B: INJECT START COMMAND FROM WEB DISPATCHER ────────
    svr.Post("/start_journey", [&](const httplib::Request& req, httplib::Response& res) {
        add_cors_headers(res);
        // Append execution command onto raw text segment
        rawM5InstructionString += "CMD:START_JOURNEY\n";
        newMissionAvailable = true;
        std::cout << "[SERVER HANDSHAKE] Start command logged. Sending execution cue to M5!\n";
        res.set_content("{\"status\":\"started\"}", "application/json");
    });

    svr.Get("/get_instructions", [&](const httplib::Request&, httplib::Response& res) {
        res.set_content(rawM5InstructionString, "text/plain");
        if (newMissionAvailable) {
            printM5Packet(rawM5InstructionString);
            std::cout << "[WIFI LINK] M5 Stick fetched solution array. Running now.\n";
            newMissionAvailable = false;
        }
    });

    svr.Get("/get_maze", [&](const httplib::Request&, httplib::Response& res) {
        res.set_content(latestMazeJson, "application/json");
    });

    svr.Post("/update_telemetry", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            json data = json::parse(req.body);
            globalTelemetryCache["stepIdx"] = data["stepIdx"];
            globalTelemetryCache["yaw"]     = data["yaw"];
            globalTelemetryCache["left"]    = data["left"];
            globalTelemetryCache["front"]   = data["front"];
            globalTelemetryCache["right"]   = data["right"];
            globalTelemetryCache["state"]   = data["state"];
            res.status = 200; res.set_content("{\"status\":\"ok\"}", "application/json");
        } catch(...) {
            res.status = 400; res.set_content("{\"status\":\"error\"}", "application/json");
        }
    });

    svr.Get("/get_telemetry", [&](const httplib::Request&, httplib::Response& res) {
        add_cors_headers(res); res.set_content(globalTelemetryCache.dump(), "application/json");
    });

    svr.Post("/solve", [&](const httplib::Request& req, httplib::Response& res) {
        add_cors_headers(res);
        try {
            json request = json::parse(req.body);
            MazeSolver solver(request); json response; solver.dumpSolution(response);
            res.set_content(response.dump(4), "application/json");
        } catch (const std::exception& e) {
            json err; err["feasible"] = false; err["message"] = e.what();
            res.status = 500; res.set_content(err.dump(), "application/json");
        }
    });

    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        std::ifstream f("index.html");
        std::string html((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        res.set_content(html, "text/html");
    });

    std::cout << "Wireless local server listening at http://0.0.0.0:8085\n";
    svr.listen("0.0.0.0", 8085);
    return 0;
}
