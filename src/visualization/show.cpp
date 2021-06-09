#include "show.h"

namespace mapf {
    Show::Show(const Map::Ptr &map, std::string image_path, std::string video_path)
        : map_(map), image_path_(image_path), video_path_(video_path),
          scalar_(5), max_len_(0)
    {
        //
    }

    void Show::GenerateImage(const std::vector<Agent::Ptr> &paths) {
        // 地图可能发生变化，画每个时间点的图都需要重新扫描地图
        max_len_ = 0;
        for(const auto p: paths) {
            if(p->GetPath().size() > max_len_) {
                max_len_ = p->GetPath().size();
            }
        }
        for(int t = 0; t < max_len_; ++t) {
            std::vector<int> image_map; // 0空白，1货架，2车辆，3抵达目标车辆，4带货架车辆
            image_map.resize(map_->GetMapSize(), 0);
            // 扫描地图信息
            for(int i = 0; i < map_->GetMapSize(); ++i) {
                if(map_->IsBlocked(i)) {
                    image_map[i] = 1;
                }
            }
            // 扫描路径车辆信息
            for(const auto p: paths) {
                if(t < p->GetPath().size()) {
                    int loc = p->GetPath()[t];
                    image_map[loc] = 2;
                }
                else {
                    int loc = p->GetPath().back();
                    image_map[loc] = 3;
                }
            }
            // 画图
            cv::Mat img(map_->GetHeight() * scalar_, map_->GetWidth() * scalar_, CV_8UC3, cv::Scalar(255,255,255));
            for(int loc = 0; loc < image_map.size(); ++loc) {
                std::pair<int, int> cor = map_->ToXY(loc);
                cv::Point p1(cor.first * scalar_, cor.second * scalar_);
                cv::Point p2((cor.first + 1) * scalar_, (cor.second + 1) * scalar_);
                if(image_map[loc] == 1) {
                    cv::rectangle(img, p1, p2, cv::Scalar(232,197,159), CV_FILLED);
                    cv::rectangle(img, p1, p2, cv::Scalar(0,0,0), 1);
                }
                else if(image_map[loc] == 2) {
                    cv::rectangle(img, p1, p2, cv::Scalar(49,208,49), CV_FILLED);
                }
                else if(image_map[loc] == 3) {
                    cv::rectangle(img, p1, p2, cv::Scalar(0,255,255), CV_FILLED);
                }
                else if(image_map[loc] == 4) {
                    //cv::rectangle(img, p1, p2, cv::Scalar(49,208,49));
                }
            }
            std::string image_name = image_path_ + "/path_image" + std::to_string(t) + ".jpg";
            cv::imwrite(image_name, img);
        }
    }

    void Show::GenerateVideo() const {
        std::string image_name = image_path_ + "/path_image" + std::to_string(0) + ".jpg";
        cv::Mat src = cv::imread(image_name, 1);
        int isColor = 1;//彩色
        int fps = 3;//视频的帧率
        int frameWidth = src.cols;
        int frameHeight = src.rows;

        //输出视频保存位置，编码格式，帧率，尺寸，是否彩色
        cv::VideoWriter writer(video_path_ + "/mafp_path100_free.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 
        fps, cv::Size(frameWidth, frameHeight), isColor);

        for (int i = 0; i < max_len_; i++)
        {
            std::string image_name = image_path_ + "/path_image" + std::to_string(i) + ".jpg";
            src = cv::imread(image_name, 1);
            writer.write(src);//输出视频
        }
    }
}