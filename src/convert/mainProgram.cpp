#include "convert_file.h"

int main(int argc, char** argv)
{

    //ConvertFile fileConverter("/home/robot/pcd_data_set/rgbd-dataset/");


        std::cout << "Root directory is --> " << argv[1] << std::endl;
        std::cout << "Save directory is --> " << argv[2] << std::endl;
        ConvertFile fileConverter(argv[1], argv[2]);
        fileConverter.startConvertion();


    //fileConverter.testOneFile("/home/robot/pcd_data_set/rgbd-dataset/cap/cap_1/cap_1_1_1.pcd");
    return 0;

}
