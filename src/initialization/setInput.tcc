#include <iostream>

template <typename T> int
setInput::AddOption(const std::string& name, const std::string& desc,
                   const T& def)
{
  if (m_options.find(name) != m_options.end()) {
    std::cerr << "Option " << name << " already exists" << std::endl;
    return -1;
  }
  m_options.insert(std::make_pair(name, Option(name, desc, def)));
  return 0;
}
