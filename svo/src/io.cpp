#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

// std
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <sstream>
#include <limits>

#include <boost/multiprecision/cpp_dec_float.hpp>
#include <boost/multiprecision/cpp_bin_float.hpp>
#include <boost/multiprecision/float128.hpp>


// svo
#include <svo/io.h>
#include <svo/global.h>
#include <svo/map.h>
#include <svo/common/frame.h>
#include <svo/common/point.h>

namespace svo {
namespace io {

std::shared_ptr<svo::Transformation> poseFromTumByNsec(const std::string &tumFilePath, const uint64_t nsecRefStamp, bool allowTimediff)
{
    std::shared_ptr<svo::Transformation> refPose = nullptr;

    std::ifstream data(tumFilePath);
    std::string line;
    
    // iterate over all csv rows
    while (std::getline(data, line))
    {

        std::stringstream lineStream(line);
        std::string cell;
        int i = 0;
        double x, y, z;
        double qx, qy, qz, qw;
        uint64_t timestamp = 0;

        bool skipRow = true;

        // iterate over current row content
        while (std::getline(lineStream, cell, ' '))
        {
            if (i == 0)
            {
              // parse scientific e9 notation into double 
              boost::multiprecision::cpp_dec_float_50 stampBoost;  // high precision
              
              // VLOG(40) << cell;
              std::istringstream iss(cell);  // parse string represening the timesamp in seconds e+09
              iss >> stampBoost;  // forward string to boost double
              std::setprecision(21);
              // VLOG(40) << std::setprecision(21) <<"TUM Trajectory: Parsed boost timestamp: " << stampBoost;

              stampBoost *= 1e9;  // convert secs to nsecs
              
              // convert into nanosecond integer timestamp
              timestamp = static_cast<uint64_t>(stampBoost);
              // VLOG(40) << "TUM Trajectory: Parsed nsec timestamp: " << timestamp;

              // time diff threshold
              uint64_t thresh = 32; //  msec time  // TODO: make parametrizable
              thresh *= 1e6;  // extend msec to nsec

              if (timestamp == nsecRefStamp)
              {
                skipRow = false;
                VLOG(40) << "TUM trajectory file parsing: matching timestamp found at: " << timestamp ;
              }
              // during init we require the ref pose for triangulation so we lax the timestamp matching
              else if (allowTimediff)
              {
                uint64_t timediff = std::max(timestamp, nsecRefStamp) - std::min(timestamp, nsecRefStamp);
                if (timediff <= thresh )
                {
                  skipRow = false;
                  VLOG(40) << "TUM trajectory file parsing: roughly matching timestamp found with difference of: " << timediff;
                }
              }
              
            }

            // parse pose
            if (i == 1)
            {
                x = (std::stod(cell));
            }
            if (i == 2)
            {
                y = (std::stod(cell));
            }
            if (i == 3)
            {
                z = (std::stod(cell));
            }
            if (i == 4)
            {
                qx = std::stod(cell);
            }
            if (i == 5)
            {
                qy = std::stod(cell);
            }
            if (i == 6)
            {
                qz = std::stod(cell);
            }
            if (i == 7)
            {
                qw = std::stod(cell);
            }

            i = i + 1;
        
        }

        if (skipRow) {continue;}  // skip to new row if timestamp dont match
        
        svo::Quaternion quat_world_frame(qw, qx, qy, qz);  // rotates cam into world which is the rotation of camera with respect to the world
        svo::Position pos_world_frame(x, y, z);  // position of cam with respect to world
        svo::Transformation pose_w_f(pos_world_frame, quat_world_frame);

        refPose = std::make_shared<svo::Transformation>(pose_w_f);
        
        data.close();

        return refPose;
    }
    SVO_WARN_STREAM("No suitable pose found in tum file to reference timestamp!");
    data.close();

    return refPose;
}

bool saveMap(
    const MapPtr& map,
    const std::string& save_dir,
    const std::string& map_name)
{
  std::string filename = save_dir+"/"+map_name;
  std::cout << "save map to file " << filename << std::endl;
  YAML::Emitter out;
  out << YAML::BeginMap
      << YAML::Key << "frames" << YAML::Value
      << YAML::BeginSeq;

  // safe frames
  for(const auto& keyval : map->keyframes_)
  {
    const FramePtr& frame = keyval.second;
    const Vector3d& t = frame->T_world_cam().getPosition();
    const Eigen::Quaterniond& q = frame->T_world_cam().getRotation().toImplementation();
    out << YAML::BeginMap
        << YAML::Key << "frame_id" << YAML::Value << frame->id()
        << YAML::Key << "cam_name" << YAML::Value << frame->cam()->getLabel()
        << YAML::Key << "timestamp" << YAML::Value << frame->getTimestampNSec()
        << YAML::Key << "T_world_cam" << YAML::Value << YAML::Flow
        << YAML::BeginSeq
          << t.x() << t.y() << t.z()
          << q.x() << q.y() << q.z() << q.w()
        << YAML::EndSeq
        //<< YAML::Comment("tx ty tz qx qy qz qw")
        << YAML::EndMap;
  }
  out << YAML::EndSeq
      << YAML::Key << "features" << YAML::Value
      << YAML::BeginSeq;

  // safe features
  CHECK(false) << "fix implementation.";
  std::unordered_set<PointPtr> points;
  /*
  for(const auto& keyval : map->keyframes_)
  {
    for(const FeaturePtr& ftr : keyval.second->fts_)
    {
      if(ftr->point->type() == Point::TYPE_CORNER_SEED
         || ftr->point->type() == Point::TYPE_EDGELET_SEED )
      {
        continue;
      }

      out << YAML::BeginMap
          << YAML::Key << "frame_id" << YAML::Value << keyval.second->id()
          << YAML::Key << "point_id" << YAML::Value << ftr->point->id()
          << YAML::Key << "px" << YAML::Value << YAML::Flow
          << YAML::BeginSeq
            << ftr->px[0]
            << ftr->px[1]
          << YAML::EndSeq
          << YAML::Key << "level" << YAML::Value << ftr->level
          << YAML::EndMap;
      points.insert(ftr->point);
    }
  }
  */

  out << YAML::EndSeq
      << YAML::Key << "points" << YAML::Value
      << YAML::BeginSeq;

  // safe points
  for(const PointPtr point : points)
  {
    out << YAML::BeginMap
        << YAML::Key << "point_id" << YAML::Value << point->id()
        << YAML::Key << "pos" << YAML::Value << YAML::Flow
        << YAML::BeginSeq
          << point->pos_[0]
          << point->pos_[1]
          << point->pos_[2]
        << YAML::EndSeq
        << YAML::EndMap;
  }

  out << YAML::EndSeq;

  // write to file
  std::ofstream fout(filename.c_str());
  if(!fout.is_open())
    return false;
  fout << out.c_str();
  fout.close();
  return true;
}

bool loadMap(
    const std::string& load_dir,
    MapPtr& map)
{
  // TODO: assume camera has name cam0 and is in cam_calib.yaml file
  SVO_ERROR_STREAM("FIX LOAD CAMERA IMPLEMENTATION");
  CameraPtr cam;
  /*
  CameraPtr cam = vk::cameras::factory::loadFromYAML(
        load_dir+"/cam_calib.yaml", "cam0");
  */
  if(cam == nullptr)
    return false;
  cam->printParameters(std::cout, "Loaded camera:");

  map = std::make_shared<Map>();

  const std::string map_filename(load_dir+"/map.yaml");
  std::cout << "loading map from file: " << map_filename << std::endl;
  YAML::Node data = YAML::LoadFile(map_filename);

  // load frames
  for(YAML::const_iterator it=data["frames"].begin(); it!=data["frames"].end(); ++it)
  {
    YAML::Node frame = *it;
    Vector3d t_wc(frame["T_world_cam"][0].as<double>(),
                  frame["T_world_cam"][1].as<double>(),
                  frame["T_world_cam"][2].as<double>());
    Eigen::Quaterniond q_wc(frame["T_world_cam"][6].as<double>(),
                     frame["T_world_cam"][3].as<double>(),
                     frame["T_world_cam"][4].as<double>(),
                     frame["T_world_cam"][5].as<double>());

    FramePtr f = std::make_shared<Frame>(
          frame["frame_id"].as<int>(),
          frame["timestamp"].as<double>(),
          cam,
          Transformation(q_wc, t_wc));
    map->addKeyframe(f, false);
  }
  std::cout << "loaded " << map->size() << " frames." << std::endl;

  // load points
  std::unordered_map<int, PointPtr> points;
  for(YAML::const_iterator it=data["points"].begin(); it!=data["points"].end(); ++it)
  {
    YAML::Node point = *it;
    PointPtr p = std::make_shared<Point>(
          point["point_id"].as<int>(),
          Vector3d(
            point["pos"][0].as<double>(),
            point["pos"][1].as<double>(),
            point["pos"][2].as<double>()));
    points.insert(std::make_pair(point["point_id"].as<int>(), p));
  }
  std::cout << "loaded " << points.size() << " points." << std::endl;

  // load features
  size_t n=0;
  for(YAML::const_iterator it=data["features"].begin(); it!=data["features"].end(); ++it)
  {
    YAML::Node ftr = *it;
    PointPtr point = points.find(ftr["point_id"].as<int>())->second;
    FramePtr frame = map->getKeyframeById(ftr["frame_id"].as<int>());
    Eigen::Vector2d px(ftr["px"][0].as<double>(), ftr["px"][1].as<double>());
    int level = ftr["level"].as<int>();

    Vector3d bearing_vector;
    cam->backProject3(px, &bearing_vector);
    CHECK(false);
    /* TODO(cfo)
    FeaturePtr f = std::make_shared<Feature>(
          frame, px, bearing_vector.normalized(), level);
    f->point = point;
    frame->addFeature(f);
    point->addObservation(frame, f);
    ++n;
    */
  }
  std::cout << "loaded " << n << " features." << std::endl;
  return true;
}

} // namespace io
} // namespace svo
