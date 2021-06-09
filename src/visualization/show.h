#ifndef SHOW_H
#define SHOW_H

#include <opencv2/opencv.hpp>

#include "mapf_map/mapf_map.h"
#include "agent/agent.h"

namespace mapf {
    class Show
    {
    private:
        const Map::Ptr &map_;
        std::string image_path_;
        std::string video_path_;
        int scalar_;
        int max_len_;
    public:
        Show(const Map::Ptr &map, std::string image_path, std::string video_path);
        ~Show()=default;

        void GenerateImage(const std::vector<Agent::Ptr> &paths);
        void GenerateVideo() const;
    };    
}
#endif //SHOW_H