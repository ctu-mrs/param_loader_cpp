#include <param_loader_cpp/param_provider.hpp>

template<typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
    out << "{";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "}";
    return out;
}

template <typename T>
void test_load_param(const std::string& param_name, const param_loader_cpp::ParamProvider pp)
{
  T param;
  const bool success = pp.getParam(param_name, param);
  if (success)
    std::cout << "loading of parameter \"" << param_name << "\" was successful: " << param << std::endl;
  else
    std::cout << "loading of parameter \"" << param_name << "\" was NOT successful!" << std::endl;
}

int main(int argc, char **argv)
{
  param_loader_cpp::ParamProvider pp;

  pp.addYamlFile("/tmp/test.yaml");

  test_load_param<int>("int", pp);
  test_load_param<double>("double", pp);
  test_load_param<std::string>("string", pp);
  test_load_param<std::vector<std::string>>("string_list", pp);
  test_load_param<std::vector<double>>("double_list", pp);

  test_load_param<int>("namespace/int", pp);
  test_load_param<double>("namespace/double", pp);
  test_load_param<std::string>("namespace/string", pp);
  test_load_param<std::vector<std::string>>("namespace/string_list", pp);
  test_load_param<std::vector<double>>("namespace/double_list", pp);

  return 0;
}
