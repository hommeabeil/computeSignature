#ifndef CONVERT_FILE_H
#define CONVERT_FILE_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/our_cvfh.h>


#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCPointT;
typedef pcl::VFHSignature308 VFH;

class ConvertFile
{
public:
    ConvertFile();
    ConvertFile(const std::string& p_path, const std::string& p_destination);

    pcl::PointCloud<VFH>::Ptr calculateCVFHUS(PCPointT::Ptr p_cloud,
                                              pcl::PointCloud<pcl::Normal>::Ptr p_normal);

    void computeNormal(PCPointT::Ptr p_cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr p_normal);

    void computeUniformSampling(PCPointT::Ptr p_cloudIn,
                                PCPointT::Ptr p_cloudOuput);

    void startConvertion();

    void testOneFile(const std::string& p_path);


private:

    std::string m_path_directory;
    std::string m_current_file;
    std::string m_current_directory;
    std::string m_default_save_directory;

    boost::filesystem3::path m_boost_path;

    boost::filesystem3::recursive_directory_iterator m_boost_it;

};


#endif
