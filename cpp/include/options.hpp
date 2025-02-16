#ifndef GRADY_BASIC_OPTIONS
#define GRADY_BASIC_OPTIONS

#include "option.hpp"

class Options {
public:
    Options() {}
    ~Options() {}

    const Option<int>& integer_option() const { return integer_option_; }
    Option<int>& integer_option() { return integer_option_; }
    const Option<double>& real_option() const { return real_option_; }
    Option<double>& real_option() { return real_option_; }
    const Option<std::string>& string_option() const { return string_option_; }
    Option<std::string>& string_option() { return string_option_; }

    void Clear();

private:
    Option<int> integer_option_;
    Option<double> real_option_;
    Option<std::string> string_option_;
};

#endif