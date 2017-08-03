#ifndef STRING_UTILS_HPP__
#define STRING_UTILS_HPP__

#include <string>
#include <unordered_map>


std::string multi_replace (std::string str, std::unordered_map<std::string,std::string> needle_replacement) {
    for (auto it = needle_replacement.begin(); it!=needle_replacement.end(); ++it) {
        size_t index = 0;
        while ( (index = str.find (it->first, index)) != std::string::npos ) {
            str.replace(index, it->first.size(), it->second);
            index += it->second.size();
        }
     }
     return (str);
}

/*unordered_map<string,string> mocap_fname_replacements = { {"[ci]", to_string(camId)} };

#define mocap_parse_fname(fname)  multi_replace((fname),mocap_fname_replacements)*/


#endif
