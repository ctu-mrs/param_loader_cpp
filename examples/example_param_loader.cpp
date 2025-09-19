// clang: MatousFormat
/**  \file
     \brief Example file for the ParamLoader convenience class.
     \author Matouš Vrba - vrbamato@fel.cvut.cz

     This example may be run after building *param_loader_cpp* by executing `rosrun mrs_lib param_loader_example`.

     See \ref param_loader/example.cpp.
 */

/**  \example "param_loader/example.cpp"

     This example may be run after building *param_loader_cpp* by executing `rosrun mrs_lib param_loader_example`.
     It demonstrates loading of parameters from the `rosparam` server inside your node/nodelet using `ParamLoader`.

     To load parameters into the `rosparam` server, use a launchfile prefferably.
     See documentation of ROS launchfiles here: http://wiki.ros.org/roslaunch/XML.
     Specifically, the `param` XML tag is used for loading parameters directly from the launchfile: http://wiki.ros.org/roslaunch/XML/param,
     and the `rosparam` XML tag tag is used for loading parameters from a `yaml` file: http://wiki.ros.org/roslaunch/XML/rosparam.

     Example code for using the \ref ParamLoader (see documentation of the ParamLoader class):
 */

/* Include the ParamLoader header */
#include <param_loader_cpp/param_loader.hpp>
#include <vector>

int main(int argc, char **argv)
{
  param_loader_cpp::ParamLoader pl{true};
  /* So far, all the parameters were loaded from the ROS parameter server.
   * These must be loaded to the server using the param or rosparam commands
   * in the launchfile (or manually using the CLI tools). However, loading
   * a large number of parameters from the ROS server in parallel can be slow,
   * so we also offer the possibility to load parameters directly from a YAML file.
   * To do this, firstly you tell the ParamLoader which file to use: */
  pl.addYamlFile("/tmp/test.yaml");
  /* (Note that this file will have to be created manually in this case for this */
  /*  example to work.) */
  
  int p_int;
  pl.loadParam("int", p_int);

  double p_double = pl.loadParam2<double>("double");
  std::string p_str_1 = pl.loadParam2<std::string>("wrong-string", "default-string");
  std::string p_str_2 = pl.loadParam2<std::string>("string", "default-string");

  std::vector<std::string> p_str_array;
  pl.loadParam("string_list", p_str_array);

  auto p_double_array = pl.loadParam2<std::vector<double>>("double_list");

  Eigen::MatrixXd matxd;
  pl.loadMatrixDynamic("matrix_3x3", matxd, -1, 3);

  /* Check if all parameters were loaded successfully */ 
  if (!pl.loadedSuccessfully())
  {
    std::cout << "parameter loading failure!" << std::endl;
    return 1;
  }

  return 0;

}

