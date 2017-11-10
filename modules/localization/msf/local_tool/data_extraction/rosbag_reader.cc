#include "modules/localization/msf/local_tool/data_extraction/rosbag_reader.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace apollo {
namespace localization {
namespace msf {
RosbagReader::RosbagReader() {

}

RosbagReader::~RosbagReader() {

}

void RosbagReader::Subscribe(const std::string& topic, 
                BaseExporter::OnRosmsgCallback call_back, 
                BaseExporter::Ptr exporter) {
    _call_back_map[topic] = std::make_pair(exporter, call_back);
    _topics.push_back(topic);
}
                
void RosbagReader::Read(const std::string &file_name) {
    rosbag::Bag bag;
    bag.open(file_name, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(_topics));
 
    foreach(rosbag::MessageInstance const m, view)
    {
        const std::string tp = m.getTopic();
        std::cout << "Read topic: " << tp << std::endl;

        std::unordered_map<std::string, 
            std::pair<BaseExporter::Ptr, 
            BaseExporter::OnRosmsgCallback>>::iterator it 
                            = _call_back_map.find(tp);
        if (it != _call_back_map.end())
        {
            BaseExporter &exporter = *(it->second.first);
            BaseExporter::OnRosmsgCallback call_back = it->second.second;
            
            (exporter.*call_back)(m);
        }
    }

    bag.close();
}

}
}
} 
