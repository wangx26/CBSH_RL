#ifndef MAPFMAP_H
#define MAPFMAP_H


#include <string>
#include <vector>
#include <memory>

namespace mapf {
    class Map
    {
    private:
        std::string id_;
        int width_;
        int height_;
        std::vector<int> map_;
        std::vector<int> move_offset_;
    public:
        typedef std::shared_ptr<Map> Ptr;
        typedef std::shared_ptr<const Map> ConstPtr;

        enum valid_move_ { NORTH, EAST, SOUTH, WEST, WAIT, MOVE_COUNT};

        Map()=default;
        ~Map() = default;

        void LoadPictureMap(const std::string file_path);

        void LoadFileMap(const std::string file_path);

        void LoadAgentFile(const std::string file_path, std::vector<int> &starts,
        std::vector<int> &goals, int &agent_num);

        bool IsBlocked(int x, int y) const ;

        bool IsBlocked(int loc) const ;

        int ToLoc(int x, int y) const ;

        std::pair<int, int> ToXY(int loc) const ;

        int YXToLoc(int y, int x) const ;

        std::pair<int, int> ToYX(int loc) const ;

        std::vector<int> RandStart(int agent_num, int rand_seed);

        std::vector<int> RandGoal(int agent_num, int rand_seed);

        int GetWidth() const;
        int GetHeight() const;

        bool ValidMove(int curr_loc, int next_loc) const;

        std::vector<int> GetMoveOffset() const;

        int GetMapSize() const;

        void SetOffset();

        std::vector<int> GetMap() const;

        // TODO:
        void RandomMapTrain();
        void RandomMapTest();
        void LoadBenchmarkMap(bool train);
    };
}

#endif //MAPFMAP_H