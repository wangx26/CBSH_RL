#include <iostream>
#include <fstream>
#include <random>
#include <algorithm>

#include "mapf_map.h"
#include "log.h"

namespace mapf {

    void Map::SetOffset() {
        move_offset_.resize(valid_move_::MOVE_COUNT);
        move_offset_[valid_move_::WAIT] = 0;
        move_offset_[valid_move_::NORTH] = -width_;
        move_offset_[valid_move_::EAST] = 1;
        move_offset_[valid_move_::SOUTH] = width_;
        move_offset_[valid_move_::WEST] = -1;
    }

    void Map::LoadPictureMap(const std::string file_path) {
        //
    }

    void Map::LoadFileMap(const std::string file_path) {
        std::string line;
        std::fstream f(file_path.c_str(), std::ios_base::in);
        if(f.is_open()){
            std::getline(f, line);
            width_ = std::atoi(line.c_str());
            std::getline(f, line);
            height_ = std::atoi(line.c_str());
            for(int i = 0; i < height_; ++i) {
                std::getline(f, line);
                for(int j = 0; j < width_; ++j){
                    if(line[j] == '1' || line[j] == '@' || line[j] == 'T'){
                        map_.push_back(1);
                    }
                    else{
                        map_.push_back(0);
                    }
                }
            }
        }
        else {
            std::cout << "Open map fail :" << file_path.c_str() << std::endl;
            return;
        }
        f.close();
    }

    void Map::LoadAgentFile(const std::string file_path, std::vector<int> &starts, 
        std::vector<int> &goals, int &agent_num) {
        std::ifstream fp(file_path.c_str());
        if(fp.is_open()) {
            std::string line;
            std::getline(fp, line);
            agent_num = atoi(line.c_str());
            while(std::getline(fp, line)) {
                std::string num;
                std::istringstream readstr(line);
                std::getline(readstr, num, ',');
                int sx = atoi(num.c_str());
                std::getline(readstr, num, ',');
                int sy = atoi(num.c_str());
                std::getline(readstr, num, ',');
                int gx = atoi(num.c_str());
                std::getline(readstr, num, ',');
                int gy = atoi(num.c_str());
                starts.push_back(ToLoc(sy, sx));    // CBSH2????????????x,y??????
                goals.push_back(ToLoc(gy, gx));
            }
        }
        else{
            LOG_ERROR_STREAM("Fail to open agent file");
        }
        fp.close();
    }

    bool Map::IsBlocked(int x, int y) const {
        int loc = ToLoc(x, y);
        if(loc < 0 || loc >= map_.size()){
            return false;
        }
        else{
            return map_[loc] == 1;
        }
    }

    bool Map::IsBlocked(int loc) const {
        if(loc < 0 || loc >= map_.size()){
            return false;
        }
        else{
            return map_[loc] == 1;
        }
    }

    int Map::ToLoc(int x, int y) const {
        return width_ * y + x;
    }

    std::pair<int, int> Map::ToXY(int loc) const {
        return std::make_pair(loc % width_, loc / width_);
    }

    int Map::YXToLoc(int y, int x) const {
        return width_ * x + y;
    }

    std::pair<int, int> Map::ToYX(int loc) const {
        return std::make_pair(loc / width_, loc % width_);
    }

    std::vector<int> Map::RandStart(int agent_num, int rand_seed) {
        std::vector<int> all_loc;
        for(int i = 0; i < width_ * height_; ++i){
            if(map_[i] == 0){
                all_loc.push_back(i);
            }
        }
        //std::random_device rd;
        //std::mt19937 g(rd());
        std::srand(rand_seed);
        std::random_shuffle(all_loc.begin(), all_loc.end());
        std::vector<int> result(all_loc.begin(), all_loc.begin() + agent_num);
        return result;
    }

    std::vector<int> Map::RandGoal(int agent_num, int rand_seed) {
        std::vector<int> all_loc;
        for(int i = 0; i < width_ * height_; ++i){
            if(map_[i] == 1){
                all_loc.push_back(i);
            }
        }
        //std::random_device rd;
        //std::mt19937 g(rd());
        std::srand(rand_seed);
        std::random_shuffle(all_loc.begin(), all_loc.end());
        std::vector<int> result(all_loc.begin(), all_loc.begin() + agent_num);
        return result;
    }

    int Map::GetWidth() const {
        return width_;
    }

    int Map::GetHeight() const {
        return height_;
    }

    bool Map::ValidMove(int curr_loc, int next_loc) const {
        if(next_loc < 0 || next_loc >= width_ * height_) {
            return false;
        }
        std::pair<int, int> curr = ToXY(curr_loc);
        std::pair<int, int> next = ToXY(next_loc);
        return abs(next.first - curr.first) + abs(next.second - curr.second) < 2;
    }

    std::vector<int> Map::GetMoveOffset() const {
        return move_offset_;
    }

    int Map::GetMapSize() const {
        return width_ * height_;
    }

}// mapf