#include "option.hpp"

template<typename DataType>
const DataType& Option<DataType>::operator[](const std::string& key) const {
    return option_.at(key);
}

template<typename DataType>
DataType& Option<DataType>::operator[](const std::string& key) {
    return option_[key];
}

template<typename DataType>
const bool Option<DataType>::HasKey(const std::string& key) const {
    return option_.find(key) != option_.end();
}

template<typename DataType>
std::ostream& operator<<(std::ostream& out, const Option<DataType>& option) {
    bool leading_endl = false;
    for (const auto& pair : option()) {
        if (leading_endl) out << std::endl;
        out << pair.first << ": " << pair.second;
        leading_endl = true;
    }
    return out;
}

template class Option<int>;
template class Option<double>;
template class Option<std::string>;

template
std::ostream& operator<<<int>(std::ostream& out, const Option<int>& option);

template
std::ostream& operator<<<double>(std::ostream& out, const Option<double>& option);

template
std::ostream& operator<<<std::string>(std::ostream& out, const Option<std::string>& option);
