#ifndef OPTION_H
#define OPTION_H

#include "eigenIncludes.h"
#include <string>

class Option
{
public:

  enum Type {BOOL, INT, DOUBLE, VEC, STRING};
  typedef std::string OptionKey;

  OptionKey name;
  Type type;
  bool b;
  int i;
  double r;
  Vector3d v;
  std::string s;
  std::string label;

  Option(const OptionKey& opt_name, const std::string& desc, const bool& def)
    : name(opt_name)
    , type(BOOL)
    , b(def)
    , i(0)
    , r(0.0)
    , label(desc)
  {}

  Option(const OptionKey& opt_name, const std::string& desc, const int& def)
    : name(opt_name)
    , type(INT)
    , b(false)
    , i(def)
    , r(0.0)
    , label(desc)
  {}

  Option(const OptionKey& opt_name, const std::string& desc, const double& def)
    : name(opt_name)
    , type(DOUBLE)
    , b(false)
    , i(0)
    , r(def)
    , label(desc)
  {}

  Option(const OptionKey& opt_name, const std::string& desc, const std::string& def)
    : name(opt_name)
    , type(STRING)
    , b(false)
    , i(0)
    , r(0.0)
    , s(def)
    , label(desc)
  {}

  Option(const OptionKey& opt_name, const std::string& desc, const Vector3d& def)
    : name(opt_name)
    , type(VEC)
    , b(false)
    , i(0)
    , r(0.0)
    , v(def)
    , label(desc)
  {}

  Option(const OptionKey& opt_name, const std::string& desc, const char* def)
    : name(opt_name)
    , type(STRING)
    , b(false)
    , i(0)
    , r(0.0)
    , s(def)
    , label(desc)
  {}
};

#endif // OPTION_H