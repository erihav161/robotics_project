#include <ros/ros.h>
#include <urdf/model.h>
#include "lab2/hTransform.hpp"
#include "lab2/chain.hpp"


using namespace std;

class MyFKParser {
    private:
    // data
    public:
    Chain kinematicChain;
    urdf::Model model;
    string urdf_file;

    void parse(string path_to_urdf);
};
