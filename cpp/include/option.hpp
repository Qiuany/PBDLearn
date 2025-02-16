#ifndef BASIC_OPTION
#define BASIC_OPTION

#include "config.hpp"


template<typename DataType>
class Option {
public:
    Option() : option_() {}
    ~Option() {}

    const DataType& operator[](const std::string& key) const;
    DataType& operator[](const std::string& key);
    const std::map<std::string, DataType>& operator()() const { return option_; }
    void Clear() { return option_.clear(); }

    const bool HasKey(const std::string& key) const;

private:
    std::map<std::string, DataType> option_;
};

template<typename DataType>
std::ostream& operator<<(std::ostream& out, const Option<DataType>& option);

#endif