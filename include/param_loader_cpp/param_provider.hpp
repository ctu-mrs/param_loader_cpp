#pragma once

#include <iostream>
#include <concepts>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace param_loader_cpp
{

  /*!
   * \brief Helper overload for printing of std::vectors.
   *
   * \param os          The output stream to send the printed vector to.
   * \param var         The vector to be printed.
   * \return            A reference to the output stream.
   */
  template <typename T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& var);

  /*!
   * \brief Helper overload for printing of std::maps.
   *
   * \param os          The output stream to send the printed map to.
   * \param var         The map to be printed.
   * \return            A reference to the output stream.
   */
  template <typename Key, typename Value>
  std::ostream& operator<<(std::ostream& os, const std::map<Key, Value>& var);


  /** \brief Convenience concept of a numeric value (i.e. either integral or floating point, and not bool). */
  template <typename T>
  concept numeric = (std::integral<T> || std::floating_point<T>) && !std::same_as<T, bool>;

  /* resolved_name_t //{ */
  
  struct resolved_name_t
  {
    std::string str;

    resolved_name_t() = default;
    resolved_name_t(std::string&& s) : str(s) {}
    resolved_name_t(const std::string& s) : str(s) {}

    // Explicit conversion
    explicit operator std::string() const
    {
      return str;
    }

    friend std::ostream& operator<<(std::ostream& os, const resolved_name_t& r)
    {
      return os << r.str;
    }

    friend bool operator==(const resolved_name_t& lhs, const resolved_name_t& rhs)
    {
      return lhs.str == rhs.str;
    }
  };
  
  //}
/*** ParamProvider CLASS //{ **/

/**
 * \brief Helper class for ParamLoader and DynparamMgr.
 *
 * This class abstracts away loading of parameters from ROS parameter server and directly from
 * YAML files ("static" parameters). The user can specify a number of YAML files that will be
 * parsed and when a parameter value is requested, these are checked first before attempting
 * to load the parameter from the ROS server (which can be slow). The YAML files are searched
 * in FIFO order and when a matching name is found in a file, its value is returned.
 *
 */
class ParamProvider
{
public:
  /** \brief Helper struct for a numeric range with named members to make the code a bit more readable. */
  template <typename T>
  struct range_t
  {
    /** \brief Minimal value of a parameter. */
    T minimum;
    /** \brief Maximal value of a parameter. */
    T maximum;
  };

  /*!
   * \brief Main constructor.
   */
  ParamProvider();

  /*!
   * \brief Add a YAML file to be parsed and used for loading parameters.
   *
   * The first file added will be the first file searched for a parameter when using getParam().
   *
   * \param filepath  Path to the YAML file.
   */
  bool addYamlFile(const std::string& filepath);

  /*!
   * \brief Copy parsed YAMLs from another ParamProvider.
   *
   * \param param_provider  The ParamProvider object to copy the YAMLs from.
   */
  void copyYamls(const ParamProvider& param_provider);

  /*!
   * \brief Gets the value of a parameter.
   *
   * Firstly, the parameter is attempted to be loaded from the YAML files added by the addYamlFile() method
   * in the same order that they were added. If the parameter is not found in any YAML file, and the use_rosparam
   * flag of the constructor is true, the ParamProvider will declare it in ROS and attempt to load it from ROS.
   *
   * \param param_name      Name of the parameter to be loaded. Namespaces should be separated with a forward slash '/'.
   * \param value_out       Output argument that will hold the value of the loaded parameter, if successfull. Not modified otherwise.
   * \return                true iff the parameter was successfully loaded.
   */
  template <typename T>
  bool getParam(const std::string& param_name, T& value_out) const;
  template <typename T>
  bool getParam(const resolved_name_t& resolved_name, T& value_out) const;
  template <typename T>
  bool getParam(const resolved_name_t& resolved_name, T& value_out, const T& default_value) const;

  /*!
   * \brief Sets the value of a parameter to ROS.
   *
   * This method sets the parameter to ROS with the desired value.
   *
   * \param param_name      Name of the parameter to be set. Namespaces should be separated with a forward slash '/'.
   * \param value           The desired value of the parameter.
   * \return                true iff the parameter was successfully set.
   */
  template <typename T>
  bool setParam(const std::string& param_name, const T& value) const;

  /*!
   * \brief Sets a prefix that will be applied to parameter names before subnode namespaces.
   *
   * The prefix will be applied as-is, so if you need to separate it from the parameter name
   * e.g. using a forward slash '/', you must include it in the prefix.
   *
   * \param prefix      The prefix to be applied to all parameter names.
   */
  void setPrefix(const std::string& prefix);

  /*!
   * \brief Returns the current parameter name prefix.
   *
   * \return The current prefix to be applied to all parameter names.
   */
  std::string getPrefix() const;

private:
  std::vector<std::shared_ptr<YAML::Node>> m_yamls;
  std::string m_prefix;

  std::optional<YAML::Node> findYamlNode(const resolved_name_t& resolved_name) const;

  template <typename T>
  bool loadFromYaml(const resolved_name_t& resolved_name, T& value_out) const;
};
//}

  template <typename T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& var)
  {
    for (size_t it = 0; it < var.size(); it++)
    {
      os << var.at(it);
      if (it < var.size() - 1)
        os << ", ";
    }
    return os;
  }

  template <typename Key, typename Value>
  std::ostream& operator<<(std::ostream& os, const std::map<Key, Value>& var)
  {
    size_t it = 0;
    for (const auto& pair : var) {
      os << pair.first << ": " << pair.second;
      if (it < var.size() - 1)
        os << std::endl;
      it++;
    }
    return os;
  }

  /* getParam() method and overloads //{ */
  template <typename T>
  bool ParamProvider::getParam(const std::string& param_name, T& value_out) const
  {
    return getParam(resolved_name_t(param_name), value_out);
  }

  template <typename T>
  bool ParamProvider::getParam(const resolved_name_t& resolved_name, T& value_out) const
  {
    return loadFromYaml(resolved_name, value_out);
  }

  template <typename T>
  bool ParamProvider::getParam(const resolved_name_t& resolved_name, T& value_out, const T& default_value) const
  {
    value_out = default_value;
    return loadFromYaml(resolved_name, value_out);
  }

  /* setParam() method //{ */
  template <typename T>
  bool ParamProvider::setParam(const std::string& param_name, const T& value) const
  {
    return setParam(param_name, value);
  }

  /* loadFromYaml() method //{ */
  template <typename T>
  bool ParamProvider::loadFromYaml(const resolved_name_t& resolved_name, T& value_out) const
  {
    T loaded_value;
  
    const auto found_node = findYamlNode(resolved_name);
    if (!found_node.has_value())
    {
      return false;
    }
  
    try
    {
      // try catch is the only type-generic option...
      loaded_value = found_node.value().as<T>();
    }
    catch (const YAML::BadConversion& e)
    {
      std::cout << "The YAML-loaded parameter has a wrong type: " << e.what() << std::endl;
      return false;
    }
    catch (const YAML::Exception& e)
    {
      std::cout << "YAML-CPP threw an unknown exception: " << e.what() << std::endl;
      return false;
    }
  
    // if all is OK, set the output value
    value_out = loaded_value;
    // the parameter value was successfully loaded and the parameter was declared if required, everything is done, return true
    return true;
  }
  //}
//}

  ParamProvider::ParamProvider() {};

  bool ParamProvider::addYamlFile(const std::string& filepath)
  {
    try
    {
      const auto loaded_yaml = YAML::LoadFile(filepath);
      auto root = std::make_shared<YAML::Node>();
      (*root)["root"] = loaded_yaml;
      m_yamls.emplace_back(root);
      return true;
    }
    catch (const YAML::ParserException& e)
    {
      std::cout << "Failed to parse file \"" << filepath << "\"! Parameters will not be loaded: " << e.what() << std::endl;
      return false;
    }
    catch (const YAML::BadFile& e)
    {
      std::cout << "File \"" << filepath << "\" does not exist! Parameters will not be loaded: " << e.what() << std::endl;
      return false;
    }
    catch (const YAML::Exception& e)
    {
      std::cout << "YAML-CPP threw an exception! Parameters will not be loaded: " << e.what() << std::endl;
      return false;
    }
    return false;
  }

  void ParamProvider::copyYamls(const ParamProvider& param_provider)
  {
    m_yamls.insert(std::end(m_yamls), std::begin(param_provider.m_yamls), std::end(param_provider.m_yamls));
  }

  std::optional<YAML::Node> ParamProvider::findYamlNode(const resolved_name_t& resolved_name) const
  {
    for (const auto& yaml : m_yamls)
    {
      // Try to load the parameter sequentially as a map.
      auto cur_node_it = std::cbegin(*yaml);
      // The root should always be a pam
      if (!cur_node_it->second.IsMap())
        continue;

      bool loaded = true;
      {
        constexpr char delimiter = '/';
        auto substr_start = std::cbegin(resolved_name.str);
        auto substr_end = substr_start;
        do
        {
          substr_end = std::find(substr_start, std::cend(resolved_name.str), delimiter);
          // why can't substr or string_view take iterators? :'(
          const auto start_pos = std::distance(std::cbegin(resolved_name.str), substr_start);
          const auto count = std::distance(substr_start, substr_end);
          const std::string param_substr = resolved_name.str.substr(start_pos, count);
          substr_start = substr_end+1;

          bool found = false;
          for (auto node_it = std::cbegin(cur_node_it->second); node_it != std::cend(cur_node_it->second); ++node_it)
          {
            if (node_it->first.as<std::string>() == param_substr)
            {
              cur_node_it = node_it;
              found = true;
              break;
            }
          }

          if (!found)
          {
            loaded = false;
            break;
          }
        }
        while (substr_end != std::end(resolved_name.str) && cur_node_it->second.IsMap());
      }

      if (loaded)
      {
        return cur_node_it->second;
      }
    }

    return std::nullopt;
  }

  void ParamProvider::setPrefix(const std::string& prefix)
  {
    m_prefix = prefix;
  }

  std::string ParamProvider::getPrefix() const
  {
    return m_prefix;
  }
}  // namespace param_loader_cpp

namespace std
{
  template <>
  struct hash<param_loader_cpp::resolved_name_t>
  {
    std::size_t operator()(const param_loader_cpp::resolved_name_t& r) const noexcept
    {
      return std::hash<std::string>{}(r.str);
    }
  };
}
