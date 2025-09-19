// clang: MatousFormat
#include <param_loader_cpp/param_loader.hpp>

namespace param_loader_cpp
{

  // Explicit instantiation of the tepmplated functions to precompile them into param_loader_cpp and speed up compilation of user program.
  // Instantiating these functions should be sufficient to invoke precompilation of all templated ParamLoader functions.

  template bool ParamLoader::loadParam<bool>(const std::string& name, bool& out_value, const bool& default_value);
  template bool ParamLoader::loadParamReusable<bool>(const std::string& name, bool& out_value, const bool& default_value);

  template bool ParamLoader::loadParam<int>(const std::string& name, int& out_value, const int& default_value);
  template bool ParamLoader::loadParamReusable<int>(const std::string& name, int& out_value, const int& default_value);

  template bool ParamLoader::loadParam<double>(const std::string& name, double& out_value, const double& default_value);
  template bool ParamLoader::loadParamReusable<double>(const std::string& name, double& out_value, const double& default_value);

  template bool ParamLoader::loadParam<std::string>(const std::string& name, std::string& out_value, const std::string& default_value);
  template bool ParamLoader::loadParamReusable<std::string>(const std::string& name, std::string& out_value, const std::string& default_value);

  ParamLoader::ParamLoader(bool printValues)
      : m_load_successful(true), m_print_values(printValues)
  {}

  /* Constructor overloads //{ */
  /*!
   * \brief Convenience overload to enable writing ParamLoader pl(nh, node_name);
   *
   * \param nh            The parameters will be loaded from rosparam using this node handle.
   * \param node_name     Optional node name used when printing the loaded values or loading errors.
   */
  ParamLoader::ParamLoader()
    : ParamLoader(true)
  {}

  //}

  void ParamLoader::setPrefix(const std::string& prefix)
  {
    m_pp.setPrefix(prefix);
  }

  std::string ParamLoader::getPrefix()
  {
    return m_pp.getPrefix();
  }

  bool ParamLoader::loadedSuccessfully() const
  {
    return m_load_successful;
  }

  void ParamLoader::resetLoadedSuccessfully()
  {
    m_load_successful = true;
  }

  void ParamLoader::resetUniques()
  {
    m_loaded_params.clear();
  }

  bool ParamLoader::addYamlFile(const std::string& filepath)
  {
    return m_pp.addYamlFile(filepath);
  }

  bool ParamLoader::addYamlFileFromParam(const std::string& param_name)
  {
    std::string filepath;
    if (!loadParam(param_name, filepath))
      return false;
    return m_pp.addYamlFile(filepath);
  }

  void ParamLoader::copyYamls(const ParamLoader& param_loader)
  {
    m_pp.copyYamls(param_loader.m_pp);
  }

  param_loader_cpp::ParamProvider& ParamLoader::getParamProvider()
  {
    return m_pp;
  }

  bool ParamLoader::check_duplicit_loading(const resolved_name_t& name)
  {
    if (m_loaded_params.count(name))
    {
      printError(std::string("Tried to load parameter \"") + name.str + std::string("\" twice"));
      m_load_successful = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  /* printError and printWarning functions //{*/
  void ParamLoader::printError(const std::string& str)
  {
    std::cout << str << std::endl;
  }
  void ParamLoader::printWarning(const std::string& str)
  {
    std::cout << str << std::endl;
  }
  //}

}
