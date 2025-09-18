// clang: MatousFormat
#include <memory>
#include <param_loader_cpp/param_provider.hpp>

namespace param_loader_cpp
{
  ParamProvider::ParamProvider()
  {
  }

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
} // namespace param_loader_cpp
