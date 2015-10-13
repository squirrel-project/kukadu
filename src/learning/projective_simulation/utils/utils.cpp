#include "utils.h"

#include <dirent.h>

using namespace std;

std::vector<std::string> getFilesInDir(std::string directory) {

    DIR* dir;
    vector<string> retFiles;
    struct dirent* ent;
    if ((dir = opendir(directory.c_str())) != NULL) {
        while((ent = readdir(dir)) != NULL) {
            if(ent->d_type != DT_DIR)
                retFiles.push_back(string(ent->d_name));
        }
        closedir(dir);
    }
    return retFiles;

}

std::vector<std::string> filterByPrefix(std::vector<std::string>& vec, string prefix) {

    vector<string> retVec;
    for(string s : vec) {
        if(!s.find(prefix))
            retVec.push_back(s);
    }

    return retVec;

}
