#include <convert_file.h>


ConvertFile::ConvertFile()
{

}

ConvertFile::ConvertFile(const std::string &p_path, const std::string& p_destination):
    m_path_directory(p_path),
    m_boost_path(p_path),
    m_boost_it(m_boost_path),
    m_default_save_directory(p_destination)
{

}


pcl::PointCloud<VFH>::Ptr ConvertFile::calculateCVFHUS(PCPointT::Ptr p_cloud,
                                                       pcl::PointCloud<pcl::Normal>::Ptr p_normal)
{
    pcl::OURCVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> ourcvfh;
    pcl::search::KdTree<PointT>::Ptr ktree (new pcl::search::KdTree<PointT>);
    ourcvfh.setInputCloud(p_cloud);
    ourcvfh.setInputNormals(p_normal);
    ourcvfh.setSearchMethod(ktree);
    ourcvfh.setRadiusSearch(0.05);
    ourcvfh.setKSearch(0);
    ourcvfh.setNormalizeBins(false);
    //ourcvfh.setAxisRatio(0.8);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr returnCloud(new pcl::PointCloud<pcl::VFHSignature308>);
    std::cout << returnCloud->size() << std::endl;
    ourcvfh.compute(*returnCloud);


    std::cout << "CVFH size = " << returnCloud->size() << std::endl;

    return returnCloud;
}


void ConvertFile::computeNormal(PCPointT::Ptr p_cloud,
                                pcl::PointCloud<pcl::Normal>::Ptr p_normal)
{
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ()); //search method
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(0.03);
    normal_estimation.setInputCloud(p_cloud);
    normal_estimation.compute(*p_normal);
}

void ConvertFile::computeUniformSampling(PCPointT::Ptr p_cloudIn,
                                         PCPointT::Ptr p_cloudOuput)
{

    std::cout << "US computation begin" << std::endl;

    pcl::UniformSampling<PointT> uniformSampling;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    boost::shared_ptr<std::vector<int> > point_cloud_indice (new std::vector<int> ());
    pcl::PointCloud<int> point_cloud_out;
    uniformSampling.setInputCloud(p_cloudIn);
    uniformSampling.setSearchMethod(tree);
    uniformSampling.setRadiusSearch(0.01);
    uniformSampling.compute(point_cloud_out);

    for(int i = 0; i < point_cloud_out.size(); i++)
    {
        point_cloud_indice->push_back(point_cloud_out.at(i));
    }


    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(p_cloudIn);
    extract.setIndices(point_cloud_indice);
    extract.setNegative(false);
    extract.filter(*p_cloudOuput);


    std::cout << "Point cloud out size = " << point_cloud_out.size() << std::endl;
    std::cout << "Keypoints Size = " << p_cloudOuput->size() << std::endl;

    point_cloud_indice.reset();
    tree.reset();


    //showPointCloud(p_cloudOuput);
}

void ConvertFile::startConvertion()
{



    while(m_boost_it != boost::filesystem3::recursive_directory_iterator())
    {
        boost::filesystem3::path path = *m_boost_it;
        PCPointT::Ptr loadedCloud_ptr (new PCPointT);
        PCPointT::Ptr USPointCloud_ptr(new PCPointT);
        pcl::PointCloud<VFH>::Ptr vfhPointCloud_ptr(new pcl::PointCloud<VFH>);
        pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);

        if(! boost::filesystem3::is_directory(path) and path.extension() == ".pcd")
        {
            std::cout << path.c_str() << std::endl;
            std::string fileName = path.c_str();
            pcl::io::loadPCDFile(fileName, *loadedCloud_ptr);
            computeNormal(loadedCloud_ptr, normal_ptr);
            if(USPointCloud_ptr->size() > 0)
            {
                *vfhPointCloud_ptr = *(calculateCVFHUS(loadedCloud_ptr, normal_ptr));


                boost::filesystem3::path::iterator pathIt = path.begin();
                std::vector<std::string> pathVector;
                while(pathIt != path.end())
                {
                    m_current_file = (*pathIt).c_str();
                    pathVector.push_back((*pathIt).c_str());
                    pathIt++;
                }
                std::cout << m_current_file << std::endl;
                boost::filesystem3::path savePath(m_default_save_directory);
                savePath /= pathVector.at(pathVector.size()-3);
                savePath /= pathVector.at(pathVector.size()-2);

                if(! boost::filesystem3::exists(savePath))
                {
                    boost::filesystem3::create_directories(savePath);
                }

                savePath /= pathVector.at(pathVector.size()-1);
                vfhPointCloud_ptr->height = 1;
                vfhPointCloud_ptr->width = vfhPointCloud_ptr->size();
                pcl::io::savePCDFile(savePath.c_str(), *vfhPointCloud_ptr);

                std::cout << savePath.c_str() << std::endl;

                m_boost_it++;
            }
            else
            {
                m_boost_it++;
            }
        }
        else
        {
            m_boost_it++;
        }
        loadedCloud_ptr.reset();
        USPointCloud_ptr.reset();
        vfhPointCloud_ptr.reset();
        normal_ptr.reset();

    }


}

void ConvertFile::testOneFile(const std::string& p_path)
{
    PCPointT::Ptr loadedCloud_ptr (new PCPointT);
    PCPointT::Ptr USPointCloud_ptr(new PCPointT);
    pcl::PointCloud<VFH>::Ptr vfhPointCloud_ptr(new pcl::PointCloud<VFH>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile(p_path, *loadedCloud_ptr);
    computeNormal(loadedCloud_ptr, normal_ptr);
    *vfhPointCloud_ptr = *(calculateCVFHUS(loadedCloud_ptr, normal_ptr));
}
